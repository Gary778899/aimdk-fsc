#!/usr/bin/env python3
"""通过有限等待的 ZeroMQ REQ 请求控制独立采集器。"""

import argparse
import json
import sys


def parse_arguments():
    """解析 START／STOP／STATUS、服务端地址及毫秒超时；非法参数由 argparse 报错。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", type=str.upper, choices=("START", "STOP", "STATUS"))
    parser.add_argument("--endpoint", default="tcp://127.0.0.1:5555")
    parser.add_argument("--timeout-ms", type=int, default=5000)
    options = parser.parse_args()
    if options.timeout_ms <= 0:
        parser.error("--timeout-ms 必须大于 0")
    return options


def main():
    """发送一次请求并显示应答；成功返回 0，超时、错误或依赖缺失返回非零。"""
    options = parse_arguments()
    try:
        import zmq
    except ImportError as error:
        print(f"无法导入 pyzmq（zmq）：{error}", file=sys.stderr)
        print(f"当前 Python 解释器：{sys.executable}", file=sys.stderr)
        print(
            "请在同一机器人容器内安装 python3-zmq；若已安装，"
            "请检查当前解释器与系统 /usr/bin/python3 的环境是否一致。",
            file=sys.stderr,
        )
        return 2

    context = zmq.Context()
    socket = context.socket(zmq.REQ)
    socket.setsockopt(zmq.LINGER, 0)
    socket.setsockopt(zmq.SNDTIMEO, options.timeout_ms)
    socket.setsockopt(zmq.RCVTIMEO, options.timeout_ms)
    try:
        socket.connect(options.endpoint)
        socket.send_string(options.command)
        response = socket.recv_string()
        if response.startswith("ERROR"):
            print(response, file=sys.stderr)
            return 1
        if options.command == "STATUS":
            value = json.loads(response)
            if not isinstance(value, dict):
                raise ValueError("STATUS 应答必须是 JSON 对象")
            print(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False))
        elif response == "ACK":
            print(response)
        else:
            raise ValueError(f"未知应答：{response!r}")
        return 0
    except zmq.Again:
        print(
            "请求超时，服务端最终状态未知；STOP 超时不代表文件已关闭，请查询 STATUS。",
            file=sys.stderr,
        )
        return 1
    except (zmq.ZMQError, ValueError, UnicodeError) as error:
        print(f"采集控制失败：{error}", file=sys.stderr)
        return 1
    finally:
        socket.close()
        context.term()


if __name__ == "__main__":
    sys.exit(main())
