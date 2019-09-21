#!/usr/bin/env python3
import numpy as np
import math

"""
four equation
sx + vx t = tx
sy + vy t = ty
sz + vz t  - (9.8 / 2) * t * t = tz
vx * tan * vx * tan + vy * tan * vy * tan = vz * vz

vx t tan * vx t tan + vy t tan * vy t tan = vz t * vz t
((tx - sx) * tan) ** 2 + ((ty - sy) * tan) ** 2 = (tz - sz + (9.8 / 2) t ** 2) ** 2
t = sqrt(hypot(tx - sx, ty - sy) * tan - (tz - sz) / (9.8 / 2))
vx = (tx - sx) / t
vy = (ty - sy) / t
vz = ((tz - sz) + (9.8 / 2) * t * t) / t
"""

def calc_init_vel(start_x, start_y, start_z, pitch_angle, target_x, target_y, target_z):
    t = math.sqrt(math.hypot(target_x - start_x, target_y - start_y) * math.tan(pitch_angle) - (target_z - start_z) / (9.8 / 2.0))
    vx = (target_x - start_x) / t
    vy = (target_y - start_y) / t
    vz = ((target_z - start_z) + (9.8 / 2.0) * t * t) / t
    print("t")
    print(t)
    print("x y z vx vy vz")
    print("{} {} {} {} {} {}".format(start_x, start_y, start_z, vx, vy, vz))
    return [vx, vy, vz]


def calc_hit_state(now_x, now_y, now_z, vx, vy, vz, px, py, pz):
    k_hit = (vy * start_x - vx * start_y) / (vy * px - vx * py)
    hit_pos_x = k_hit * px
    hit_pos_y = k_hit * py
    ttc = (hit_pos_x - start_x) / vx
    print("confirm: hit point")
    print("ttc: {} k_hit: {}".format(ttc, k_hit))
    print("start_z: {} vz * ttc: {} -(9.8 / 2.0) * ttc * ttc: {}".format(start_z, vz * ttc, -(9.8 / 2.0) * ttc * ttc))
    print("x: {} y: {} z: {}".format(start_x + vx * ttc, start_y + vy * ttc, start_z + vz * ttc - (9.8 / 2.0) * ttc * ttc))
    print("vx: {} vy: {} vz: {}".format(vx, vy, vz - (9.8 / 2.0) * ttc))

start_x = 3.0
start_y = 2.0
start_z = 0.8
pitch_angle = np.pi / 16.0

target_x = 1.06
target_y = 0.385
target_z = 0.603
vx, vy, vz = calc_init_vel(start_x, start_y, start_z, pitch_angle, target_x, target_y, target_z)


# 上の部分とは別の計算
# confirm
#vx = -1.157
#vy = -0.9635
#vz = -4.674
px = 1.05138
py = 0.350219
pz = 0.561539
target_x = 1.06
target_y = 0.385
target_z = 0.603

now_x = start_x
now_y = start_y
now_z = start_z
calc_hit_state(now_x, now_y, now_z, vx, vy, vz, px, py, pz)
px = target_x
py = target_y
pz = target_z
calc_hit_state(now_x, now_y, now_z, vx, vy, vz, px, py, pz)
