# 좌표 변환에 쓰는 최소한의 쿼터니언 연산.
#
# tf_transformations 나 numpy 를 끌어오지 않는 이유: 여기 필요한 건 곱·역·회전
# 세 가지뿐이고, 노드들은 시스템 python3.12 로 실행된다(colcon 이 만드는 콘솔
# 스크립트의 shebang 이 /usr/bin/python3 라 venv 를 못 탄다 — doc/AGENT.md).
# 의존성을 하나 늘리면 그 python 에도 설치되어 있어야 한다.
#
# 표기는 ROS 와 같은 (x, y, z, w) 순서다.

import math


def quaternion_from_rpy(roll, pitch, yaw):
    """roll-pitch-yaw 를 쿼터니언으로 (Z-Y-X 순, SDF·ROS 공통 규약)."""
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def quaternion_multiply(left, right):
    """쿼터니언 곱. left 뒤에 right 를 로컬 축 기준으로 이어 붙인다."""
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def quaternion_inverse(q):
    """단위 쿼터니언의 역 = 켤레. 정규화되지 않은 입력은 여기서 다루지 않는다."""
    x, y, z, w = q
    return (-x, -y, -z, w)


def rotate_vector(q, v):
    """벡터 v 를 쿼터니언 q 로 돌린다 (v' = q v q*)."""
    x, y, z, w = q
    vx, vy, vz = v
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def transform_inverse(translation, rotation):
    """변환 (t, q) 의 역변환을 돌려준다."""
    inv_q = quaternion_inverse(rotation)
    rx, ry, rz = rotate_vector(inv_q, translation)
    return ((-rx, -ry, -rz), inv_q)


def transform_multiply(first, second):
    """변환 두 개를 잇는다: first 좌표계 위에 second 를 얹는다."""
    (tx, ty, tz), q1 = first
    (ux, uy, uz), q2 = second
    rx, ry, rz = rotate_vector(q1, (ux, uy, uz))
    return ((tx + rx, ty + ry, tz + rz), quaternion_multiply(q1, q2))
