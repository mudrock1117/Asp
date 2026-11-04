# estun_mdh_fk.py
# 改进型DH（MDH）— ESTUN S3-60
# - d3 = -158 mm
# - d6 默认 138.5 mm
# - 零位偏置：q2-90°, q4-90°
# 依赖: numpy

import numpy as np

# ---- 基础变换 ----
def _eye4(): return np.eye(4, dtype=float)

def Rx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1,0,0,0],[0,c,-s,0],[0,s,c,0],[0,0,0,1]], float)

def Rz(t):
    c, s = np.cos(t), np.sin(t)
    return np.array([[c,-s,0,0],[s,c,0,0],[0,0,1,0],[0,0,0,1]], float)

def Tx(a):
    T = _eye4(); T[0,3] = a; return T

def Tz(d):
    T = _eye4(); T[2,3] = d; return T

# ---- 机械臂参数（单位：mm/deg）----
_A     = np.array([0, 0, 249.177, 220.852,   0,     0], dtype=float)
_ALPHA = np.deg2rad([0, -90, 0, 180, 90, 90]).astype(float)
_D     = np.array([161.8, 161.79, -158, -106, -105.908, 138.5], dtype=float)  # d3=-158, d6可覆盖
# theta 零位偏置（界面 theta 列）：q2-90°, q4-90°
_THETA_OFFSET = np.deg2rad([0, -90, 0, -90, 0, 0]).astype(float)

# ---- 改进型 DH：A = Rx(alpha) * Tx(a) * Rz(theta) * Tz(d) ----
def fk_modified(q, d6: float | None = None):
    """
    改进型DH(MDH)正运动学（基->法兰，不含工具偏移）
    q   : 长度6（弧度）
    d6  : 覆盖第6节 d 值(mm)，默认用 138.5
    返回: T06, R06, p06
    """
    q = np.asarray(q, float).reshape(6)
    a = _A.copy(); alpha = _ALPHA.copy(); d = _D.copy()
    if d6 is not None: d[5] = float(d6)
    theta = q + _THETA_OFFSET

    T = _eye4()
    for i in range(6):
        T = T @ (Rx(alpha[i]) @ Tx(a[i]) @ Rz(theta[i]) @ Tz(d[i]))
    return T, T[:3,:3], T[:3,3]

def mdh_fk_with_flange_tool(q, t6E_xyz, d6: float = 138.5):
    """
    法兰坐标系下的工具偏移（刚体随动）
    t6E_xyz : [tx,ty,tz] (mm) 在法兰坐标系
    返回: T0E, T06, R06, p06
    """
    T06, R06, p06 = fk_modified(q, d6=d6)
    T6E = _eye4(); T6E[:3,3] = np.asarray(t6E_xyz, float).ravel()
    T0E = T06 @ T6E
    return T0E, T06, R06, p06

def mdh_fk_with_base_offset(q, base_offset_at_zero, d6: float = 138.5,
                            base_sign=(1,1,1), rot90k: int = 0):
    """
    偏移是在 **q=0 时基坐标下**测得（情况B），自动转到法兰系再随动。
    - base_sign: 轴向更正 (sx,sy,sz)，如现场X/Y反向则(-1,-1,1)；不需更正用(1,1,1)
    - rot90k   : 基坐标绕Z的90°倍数（有些系统世界坐标偏转90°）：-1,0,1,2
    返回: T0E, T06, R06, p06   （T0E为TCP；T06为法兰）
    """
    # 先求 q 的法兰
    T06, R06, p06 = fk_modified(q, d6=d6)

    # 定义“更正后的基坐标” -> 在该坐标下做偏移测量与比较
    S3 = np.diag(base_sign[:3])
    k = int(rot90k) % 4
    Rz90 = np.array([[0,-1,0],[1,0,0],[0,0,1]], float)  # +90°
    Rz = np.linalg.matrix_power(Rz90, k)

    # q=0 的法兰姿态（用于把基系偏移转到法兰系）
    T06_0, R06_0, p06_0 = fk_modified(np.zeros(6), d6=d6)
    R06_corr0 = S3 @ Rz @ R06_0

    # 把“基坐标(q=0)下的偏移”先做轴向/旋转更正，再转到法兰系
    t_base = np.asarray(base_offset_at_zero, float).reshape(3)
    t_base_corr = S3 @ Rz @ t_base
    t6E = R06_corr0.T @ t_base_corr

    # 形成 TCP
    T6E = _eye4(); T6E[:3,3] = t6E
    T06_corr = T06.copy()
    T06_corr[:3,:3] = S3 @ Rz @ R06
    T06_corr[:3, 3] = S3 @ Rz @ p06
    T0E = T06_corr @ T6E
    return T0E, T06_corr, T06_corr[:3,:3], T06_corr[:3,3]

def mdh_fk_target_point(q, ext_len=230.0, d6: float = 138.5,
                        base_sign=(1,1,1), rot90k: int = 0):
    T06, R06, p06 = fk_modified(q, d6=d6)

    # 坐标系更正（与 mdh_fk_with_base_offset 相同）
    S3 = np.diag(base_sign[:3])
    k = int(rot90k) % 4
    Rz90 = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]], float)
    Rz = np.linalg.matrix_power(Rz90, k)

    # q=0 的法兰姿态（用于把“基(q=0)下的偏移”转到法兰系）
    T06_0, R06_0, p06_0 = fk_modified(np.zeros(6), d6=d6)
    R06_corr0 = S3 @ Rz @ R06_0

    # 目标点在“更正后的基坐标、q=0”下的偏移：[0, +ext_len, 0]
    t_base_corr = np.array([0.0, float(ext_len), 0.0])  # mm
    # 转到法兰系：刚体随动
    t6E = R06_corr0.T @ t_base_corr

    # 当前姿态下的“更正后基坐标”法兰位姿
    T06_corr = np.eye(4)
    T06_corr[:3, :3] = S3 @ Rz @ R06
    T06_corr[:3, 3] = S3 @ Rz @ p06

    # 右乘 ^6T_P 得到目标点位姿
    T6P = np.eye(4);
    T6P[:3, 3] = t6E
    T0P = T06_corr @ T6P
    return T0P, T06_corr, T06_corr[:3, :3], T06_corr[:3, 3]

def mdh_target_relative_to_tcp(
    q,
    *,
    base_offset_at_zero,          # 2b 的 TCP 偏移：在“基坐标(q=0)”下测得
    ext_len=230.0,                # 3 的目标点：在“更正后基坐标(q=0)”下 [0,+ext_len,0]
    d6=138.5,
    base_sign=(1,1,1),
    rot90k=0
):
    q = np.asarray(q, float).reshape(6)

    # 2b: TCP（基坐标@q=0测得偏移）——已处理轴向更正与旋转
    T0E_tcp, T06c, R06c, p06c = mdh_fk_with_base_offset(
        q, base_offset_at_zero=base_offset_at_zero, d6=d6,
        base_sign=base_sign, rot90k=rot90k
    )
    p_tcp = T0E_tcp[:3, 3]
    R_tcp = T0E_tcp[:3, :3]

    # 3: 目标点（在“更正后基坐标、q=0”下 [0, +ext_len, 0]，随动）
    T0P, _, _, _ = mdh_fk_target_point(
        q, ext_len=ext_len, d6=d6, base_sign=base_sign, rot90k=rot90k
    )
    p_target = T0P[:3, 3]

    # 差值
    rel_base = (p_target - p_tcp).reshape(3)
    rel_tcp = (R_tcp.T @ rel_base).reshape(3)

    return rel_base, rel_tcp, p_tcp.reshape(3), p_target.reshape(3)

def frame1_to_base_coords(
    q,
    p_in_frame1,
    *,
    base_offset_at_zero,   # 2b 的 TCP 偏移（在“基坐标@q=0”下测得）
    d6=138.5,
    base_sign=(1,1,1),
    rot90k=0
):

    # 1) 先得到 2b-TCP 在“更正后基坐标”下的位姿（随动）
    T0E, _, _, _ = mdh_fk_with_base_offset(
        q,
        base_offset_at_zero=base_offset_at_zero,
        d6=d6,
        base_sign=base_sign,
        rot90k=rot90k
    )
    R0E = T0E[:3, :3]
    p0E = T0E[:3,  3]

    # 2) q=0 时，“坐标系1”在更正后基坐标中的旋转 R01(0)
    R01_0 = np.array([[ 0, -1,  0],
                      [ 0,  0,  1],
                      [-1,  0,  0]], dtype=float)

    # 3) 计算常量 R_E1 = R0E(0)^T * R01(0)
    T0E_0, _, _, _ = mdh_fk_with_base_offset(
        np.zeros(6),
        base_offset_at_zero=base_offset_at_zero,
        d6=d6,
        base_sign=base_sign,
        rot90k=rot90k
    )
    R0E_0 = T0E_0[:3, :3]
    R_E1 = R0E_0.T @ R01_0

    # 4) 当前姿态下：R01(q) 与原点 p01(q)
    R01 = R0E @ R_E1      # 坐标系1 随 TCP 刚体随动
    p01 = p0E             # 坐标系1 原点 = TCP

    # 5) 点变换：p_base = p01 + R01 * p_in_frame1
    v = np.asarray(p_in_frame1, float).reshape(3)
    p_in_base = (p01 + R01 @ v).reshape(3)

    return p_in_base, T0E, R01



