#!/usr/bin/env python3
import math

def euler_to_quaternion_zyx(roll, pitch, yaw):
    """将欧拉角转换为四元数 (ZYX顺序)"""
    # ZYX顺序：先绕Z轴旋转(yaw)，再绕Y轴旋转(pitch)，最后绕X轴旋转(roll)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    
    w = cy * cp * cr + sy * sp * sr
    x = cy * cp * sr - sy * sp * cr
    y = sy * cp * sr + cy * sp * cr
    z = sy * cp * cr - cy * sp * sr
    
    return [x, y, z, w]

def euler_to_quaternion_xyz(roll, pitch, yaw):
    """将欧拉角转换为四元数 (XYZ顺序)"""
    # XYZ顺序：先绕X轴旋转(roll)，再绕Y轴旋转(pitch)，最后绕Z轴旋转(yaw)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    
    return [x, y, z, w]

# 测试数据
x_mm = 127.048
y_mm = 237.846  
z_mm = 653.825
roll_deg = -91.631
pitch_deg = -48.072
yaw_deg = 1.08

# 转换为米和弧度
x = x_mm / 1000.0
y = y_mm / 1000.0
z = z_mm / 1000.0
roll = math.radians(roll_deg)
pitch = math.radians(pitch_deg)
yaw = math.radians(yaw_deg)

print("=== 欧拉角到四元数转换测试 ===")
print(f"原始数据:")
print(f"  位置: x={x_mm}mm, y={y_mm}mm, z={z_mm}mm")
print(f"  欧拉角: roll={roll_deg}°, pitch={pitch_deg}°, yaw={yaw_deg}°")
print(f"转换后:")
print(f"  位置: x={x:.6f}m, y={y:.6f}m, z={z:.6f}m")

print("\n不同顺序的四元数结果:")
quat_zyx = euler_to_quaternion_zyx(roll, pitch, yaw)
print(f"ZYX顺序: x={quat_zyx[0]:.6f}, y={quat_zyx[1]:.6f}, z={quat_zyx[2]:.6f}, w={quat_zyx[3]:.6f}")

quat_xyz = euler_to_quaternion_xyz(roll, pitch, yaw)
print(f"XYZ顺序: x={quat_xyz[0]:.6f}, y={quat_xyz[1]:.6f}, z={quat_xyz[2]:.6f}, w={quat_xyz[3]:.6f}")

# 测试交换pitch和yaw的情况
print(f"\n交换pitch和yaw后的结果:")
quat_swapped = euler_to_quaternion_zyx(roll, yaw, pitch)  # 注意：这里交换了pitch和yaw
print(f"ZYX顺序(交换后): x={quat_swapped[0]:.6f}, y={quat_swapped[1]:.6f}, z={quat_swapped[2]:.6f}, w={quat_swapped[3]:.6f}")
