#!/usr/bin/env python3
"""离线分阶段测量已录 MCAP 的 Zstd 压缩、文件 write 和 fsync 耗时，不连接机器人话题。"""

import argparse
import ctypes
import ctypes.util
from datetime import datetime, timedelta, timezone
import json
import os
from pathlib import Path
import struct
import tempfile
import time
import zlib


class Zstd:
    """通过系统 libzstd 复用压缩上下文，避免额外 Python 压缩依赖。"""

    def __init__(self, level):
        """加载 Zstd 并设置数值压缩级别；库不可用或初始化失败时抛出异常。"""
        self.lib = ctypes.CDLL(ctypes.util.find_library('zstd') or 'libzstd.so.1')
        signatures = {
            'ZSTD_createCCtx': (ctypes.c_void_p, []),
            'ZSTD_freeCCtx': (ctypes.c_size_t, [ctypes.c_void_p]),
            'ZSTD_compressBound': (ctypes.c_size_t, [ctypes.c_size_t]),
            'ZSTD_decompress': (ctypes.c_size_t, [ctypes.c_void_p, ctypes.c_size_t, ctypes.c_void_p, ctypes.c_size_t]),
            'ZSTD_CCtx_setParameter': (ctypes.c_size_t, [ctypes.c_void_p, ctypes.c_int, ctypes.c_int]),
            'ZSTD_CCtx_reset': (ctypes.c_size_t, [ctypes.c_void_p, ctypes.c_int]),
            'ZSTD_compress2': (ctypes.c_size_t, [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_size_t, ctypes.c_void_p, ctypes.c_size_t]),
            'ZSTD_isError': (ctypes.c_uint, [ctypes.c_size_t]),
            'ZSTD_getErrorName': (ctypes.c_char_p, [ctypes.c_size_t]),
            'ZSTD_versionString': (ctypes.c_char_p, []),
        }
        for name, (result, arguments) in signatures.items():
            function = getattr(self.lib, name)
            function.restype = result
            function.argtypes = arguments
        self.context = self.lib.ZSTD_createCCtx()
        if not self.context:
            raise MemoryError('无法创建 Zstd 压缩上下文')
        self.check(self.lib.ZSTD_CCtx_setParameter(self.context, 100, level))

    def check(self, value):
        """检查 Zstd 返回值；失败时抛出含库错误信息的 RuntimeError。"""
        if self.lib.ZSTD_isError(value):
            raise RuntimeError(self.lib.ZSTD_getErrorName(value).decode())
        return value

    def close(self):
        """释放压缩上下文；允许重复调用。"""
        if self.context:
            self.lib.ZSTD_freeCCtx(self.context)
            self.context = None

    def unpack(self, packed, size, mode):
        """在计时区间之外解压来源块，返回原始块缓冲；仅支持 Zstd 或未压缩块。"""
        raw = ctypes.create_string_buffer(size)
        if mode == 'zstd':
            actual = self.check(self.lib.ZSTD_decompress(raw, size, packed, len(packed)))
            if actual != size:
                raise ValueError('MCAP 解压后长度与声明不一致')
        elif mode == '' and len(packed) == size:
            ctypes.memmove(raw, packed, size)
        else:
            raise ValueError(f'本脚本不支持来源块压缩方式：{mode!r}')
        return raw


def chunks(paths, limit):
    """顺序读取来源文件的数据块，返回原长度、CRC、压缩方式及载荷；损坏结构抛出异常。"""
    count = 0
    for path in paths:
        with path.open('rb') as source:
            file_size = path.stat().st_size
            source.seek(-8, os.SEEK_END)
            if source.read(8) != b'\x89MCAP0\r\n':
                raise ValueError(f'{path} 尾部 magic 无效')
            source.seek(0)
            if source.read(8) != b'\x89MCAP0\r\n':
                raise ValueError(f'{path} 头部 magic 无效')
            while source.tell() < file_size - 8:
                header = source.read(9)
                op, length = struct.unpack('<BQ', header)
                if source.tell() + length > file_size - 8:
                    raise ValueError('MCAP 记录越过文件末尾')
                if op != 6:
                    source.seek(length, os.SEEK_CUR)
                    continue
                payload = source.read(length)
                raw_size, crc, name_length = struct.unpack_from('<QII', payload, 16)
                mode = payload[32:32+name_length].decode()
                offset = 32 + name_length
                packed_size = struct.unpack_from('<Q', payload, offset)[0]
                packed = payload[offset+8:offset+8+packed_size]
                if len(packed) != packed_size:
                    raise ValueError('MCAP 数据块截断')
                yield raw_size, crc, mode, packed
                count += 1
                if limit and count >= limit:
                    return


def measured(function, *args):
    """调用函数并返回结果、墙钟纳秒及当前线程 CPU 纳秒；异常原样传出。"""
    wall = time.perf_counter_ns()
    cpu = time.thread_time_ns()
    result = function(*args)
    return result, time.perf_counter_ns()-wall, time.thread_time_ns()-cpu


def write_all(fd, data):
    """将一个压缩块全部交给文件描述符，支持短写；不执行 fsync。"""
    offset = 0
    while offset < len(data):
        size = os.write(fd, data[offset:])
        if size == 0:
            raise OSError('文件 write 返回零')
        offset += size


def main():
    """解析参数并生成逐块耗时和汇总；临时写入文件在结束后删除，来源 MCAP 保持不变。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session_dir', type=Path)
    parser.add_argument('--output-dir', type=Path, help='结果与临时写入目录，默认位于来源会话内')
    parser.add_argument('--max-chunks', type=int, default=0, help='最多测量块数，0 表示全部')
    parser.add_argument('--zstd-level', type=int, default=-3, help='数值压缩级别，默认 -3，对应 MCAP Fast')
    options = parser.parse_args()
    if options.max_chunks < 0:
        parser.error('--max-chunks 不得为负')
    session = options.session_dir.resolve()
    paths = sorted((session/'raw').glob('*.mcap'))
    if not paths:
        parser.error('来源会话 raw/ 中没有 MCAP 文件')
    if options.output_dir:
        output = options.output_dir.resolve()
        output.mkdir(parents=True, exist_ok=False)
    else:
        label = datetime.now(timezone(timedelta(hours=8))).strftime('%Y%m%d_%H%M%S')
        index = 0
        while True:
            output = session / (label + (f'_{index}' if index else ''))
            try:
                output.mkdir()
                break
            except FileExistsError:
                index += 1
    codec = Zstd(options.zstd_level)
    totals = dict(chunks=0, raw_bytes=0, compressed_bytes=0,
                  compression_wall_ns=0, compression_cpu_ns=0,
                  file_write_wall_ns=0, file_write_cpu_ns=0)
    maxima = dict(compression_wall_ns=0, file_write_wall_ns=0)
    started = time.perf_counter_ns()
    try:
        with tempfile.TemporaryFile(dir=output) as scratch, (output/'chunks.jsonl').open('w', encoding='utf-8') as log:
            for raw_size, crc, mode, packed in chunks(paths, options.max_chunks):
                raw = codec.unpack(packed, raw_size, mode)
                if crc and zlib.crc32(memoryview(raw).cast('B')) != crc:
                    raise ValueError('来源 MCAP 数据块 CRC 校验失败')
                capacity = codec.lib.ZSTD_compressBound(raw_size)
                destination = ctypes.create_string_buffer(capacity)
                codec.check(codec.lib.ZSTD_CCtx_reset(codec.context, 1))
                result, compress_wall, compress_cpu = measured(
                    codec.lib.ZSTD_compress2, codec.context, destination, capacity, raw, raw_size)
                size = codec.check(result)
                _, write_wall, write_cpu = measured(write_all, scratch.fileno(), memoryview(destination).cast('B')[:size])
                row = dict(chunk_index=totals['chunks'], raw_bytes=raw_size, compressed_bytes=size,
                           compression_wall_ns=compress_wall, compression_cpu_ns=compress_cpu,
                           file_write_wall_ns=write_wall, file_write_cpu_ns=write_cpu)
                for key in totals:
                    totals[key] += 1 if key == 'chunks' else row[key]
                for key in maxima:
                    maxima[key] = max(maxima[key], row[key])
                log.write(json.dumps(row)+'\n')
                if totals['chunks'] % 100 == 0:
                    print(f"已测量 {totals['chunks']} 块", flush=True)
            _, sync_wall, sync_cpu = measured(os.fsync, scratch.fileno())
        wall = totals['compression_wall_ns']
        write_total = totals['file_write_wall_ns'] + sync_wall
        summary = dict(source_session=str(session), output_dir=str(output),
                       zstd_version=codec.lib.ZSTD_versionString().decode(), zstd_level=options.zstd_level,
                       file_system_device=os.stat(output).st_dev, source_device=os.stat(paths[0]).st_dev,
                       totals=totals, maxima=maxima, final_fsync_wall_ns=sync_wall, final_fsync_cpu_ns=sync_cpu,
                       whole_benchmark_wall_ns=time.perf_counter_ns()-started,
                       compression_input_MB_per_s=totals['raw_bytes']*1000/wall if wall else None,
                       write_output_MB_per_s_including_final_fsync=totals['compressed_bytes']*1000/write_total if write_total else None,
                       combined_stage_input_MB_per_s=totals['raw_bytes']*1000/(wall+write_total) if wall+write_total else None,
                       scope='offline_recompression_and_buffered_write_with_final_fsync',
                       note='读文件、解压、缓冲分配与诊断日志不计入阶段耗时；write 包含页缓存，fsync 单独统计；不是原录制过程的分阶段追踪。')
        (output/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
        print(json.dumps(summary,ensure_ascii=False,indent=2))
    finally:
        codec.close()


if __name__ == '__main__':
    main()
