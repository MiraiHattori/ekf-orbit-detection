#!/usr/bin/env python3

import re
import copy
import matplotlib.pyplot as plt


"""
# each step has (not existing object will be None)
time -> float
pos_camera x y z ->float x 3
q_camera w x y z -> float x 4
pixel lx ly rx ry -> float x 4
measured x y z -> float x 3
optical_point norm x y z -> float x 3
valid -> bool x 1
delta_t -> float x 1
estimated x y z vx vy vz -> float x 6
coeff -> float 6 x 6
"""


float_regex_pattern = r"^-*[0-9]+\.[0-9]+$"
float_regex = re.compile(float_regex_pattern)
timed_log = []

with open("m-hattori-log.log", "r") as f:
    t = 0
    coeff_row = 0 # 0~5
    timed_log_element_init = {"t": None, # time
                         "pos_camera": [None, None, None], # pos_camera
                         "q_camera": [None, None, None, None], # q_camera
                         "pixel": [None, None, None, None], # pixel
                         "measured": [None, None, None], # measured
                         "optical": [None, None, None, None], # optical_point
                         "valid": True, # valid
                         "delta_t": None, # delta_t
                         "estimated": [None, None, None, None, None, None], # estimated
                         "coeff": [
                             [None, None, None, None, None, None],
                             [None, None, None, None, None, None],
                             [None, None, None, None, None, None],
                             [None, None, None, None, None, None],
                             [None, None, None, None, None, None],
                             [None, None, None, None, None, None],
                         ], # coeff
                         }
    timed_log_element = copy.deepcopy(timed_log_element_init)
    for l in f:
        line = l.rstrip('\n').split()
        if len(line) <= 0:
            pass
        elif line[0] == "pos_camera:":
            coeff_row = 0
            timed_log_element["t"] = t
            timed_log_element["pos_camera"] = copy.deepcopy([float(le) for le in line[1:4]])
        elif line[0] == "q_camera:":
            timed_log_element["q_camera"] = copy.deepcopy([float(le) for le in line[1:5]])
        elif line[0] == "pixel:":
            timed_log_element["pixel"] = copy.deepcopy([float(le) for le in line[1:5]])
        elif line[0] == "measured:":
            timed_log_element["measured"] = copy.deepcopy([float(le) for le in line[1:4]])
        elif line[0] == "optical_point(norm,x,y,z):":
            timed_log_element["optical"] = copy.deepcopy([float(le) for le in line[1:5]])
        elif line[0] == "[ball_orbit_estimator]":
            timed_log_element["valid"] = False
            timed_log.append(copy.deepcopy(timed_log_element))
            timed_log_element = copy.deepcopy(timed_log_element_init)
        elif line[0] == "delta_t:":
            t += float(line[1])
            timed_log_element["delta_t"] = float(line[1])
        elif line[0] == "estimated:":
            timed_log_element["estimated"] = copy.deepcopy([float(le) for le in line[1:7]])
        elif line[0] == "coeff:":
            pass
        elif float_regex.match(line[0]) and len(line) == 6:
            timed_log_element["coeff"][coeff_row] = copy.deepcopy([float(le) for le in line])
            if (coeff_row == 5):
                timed_log.append(copy.deepcopy(timed_log_element))
                timed_log_element = copy.deepcopy(timed_log_element_init)
            coeff_row += 1
        elif float_regex.match(line[0]) and "camera" in line[1]:
            pass
        else:
            print("Unknown tag {}".format(line[0]))

for log in timed_log:
    if not log["valid"]:
        continue
    t = log["t"]
    measured = log["measured"]
    estimated = log["estimated"]
    coeff = log["coeff"]
    """
    print("{} {} {} {}".format(
        t,
        coeff[0][0] + coeff[1][1] + coeff[2][2],
        coeff[3][3] + coeff[4][4] + coeff[5][5],
        coeff[0][0] + coeff[1][1] + coeff[2][2] + coeff[3][3] + coeff[4][4] + coeff[5][5]
        ))
    """
    x = log["estimated"][0]
    y = log["estimated"][1]
    z = log["estimated"][2]
    vx = log["estimated"][3]
    vy = log["estimated"][4]
    vz = log["estimated"][5]
    px = 1.06441
    py = 0.382175
    k_hit = (vy * x - vx * y) / (vy * px - vx * py)
    hit_pos_x = k_hit * px
    hit_pos_y = k_hit * py
    ttc = (hit_pos_x - x) / vx
    target = [None, None, None, None, None, None]
    target[0] = x + vx * ttc
    target[1] = y + vy * ttc
    target[2] = z + vz * ttc - 9.8 / 2 * ttc * ttc
    target[3] = -0.95983
    target[4] = -1.14178
    target[5] = -0.502124
    print("{} {} {} {}".format(t, target[0], target[1], target[2]))

plt.figure()
