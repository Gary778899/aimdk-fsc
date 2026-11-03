"""有限位姿与正交基变换，支持输入坐标系的反射映射。"""

import numpy as np


def vector(value, size, label):
    """将value校验为size维有限向量；无效输入抛出ValueError。"""
    array = np.asarray(value, dtype=float)
    if array.shape != (size,) or not np.isfinite(array).all():
        raise ValueError(f'{label}须为{size}维有限向量')
    return array


def matrix(value, *, reflection=False):
    """校验3×3正交矩阵；reflection允许行列式为−1的坐标基变换。"""
    array = np.asarray(value, dtype=float)
    if array.shape != (3, 3) or not np.isfinite(array).all():
        raise ValueError('矩阵须为3×3有限数值')
    determinant = float(np.linalg.det(array))
    if (not np.allclose(array.T @ array, np.eye(3), atol=1e-6, rtol=0)
            or not np.isclose(abs(determinant) if reflection else determinant,
                              1., atol=1e-6, rtol=0)):
        raise ValueError('矩阵须为正交坐标基' if reflection else '旋转矩阵须正交且行列式为+1')
    return array


def quaternion_matrix(value, order):
    """将显式xyzw或wxyz顺序的有限非零四元数归一化并转为旋转矩阵。"""
    values = vector(value, 4, '四元数')
    if order not in ('xyzw', 'wxyz'):
        raise ValueError('四元数顺序须显式指定xyzw或wxyz')
    # 先按最大分量缩放，避免极小或极大输入归一化时溢出。
    largest = float(np.max(np.abs(values)))
    if largest == 0:
        raise ValueError('四元数不能为零')
    values = values / largest
    values = values / np.linalg.norm(values)
    if order == 'wxyz':
        values = values[[1, 2, 3, 0]]
    x, y, z, w = values
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ])


def immutable_matrix(value, *, reflection=False):
    """将校验后的矩阵转换为嵌套元组，防止外部修改已接受位姿。"""
    return tuple(tuple(float(item) for item in row)
                 for row in matrix(value, reflection=reflection))
