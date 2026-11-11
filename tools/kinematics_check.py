#!/usr/bin/env python3
"""Checks lib/kinematics (C++, float) against the Python reference in thinker-robot-kinematics.

usage: tools/kinematics_check.py [PATH_TO_thinker-robot-kinematics]  (default: ../thinker-robot-kinematics)
Builds tools/kinematics_check.cpp with g++, feeds it random cases, compares with arm.py.
"""
import math
import os
import random
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, sys.argv[1] if len(sys.argv) > 1 else os.path.join(REPO, '..', 'thinker-robot-kinematics'))
from thinker_kinematics import arm, vec  # noqa: E402

D = math.radians
exe = os.path.join(tempfile.mkdtemp(), 'kincheck')
subprocess.run(['g++', '-O2', '-std=c++17', '-I', os.path.join(REPO, 'lib', 'kinematics'),
                os.path.join(HERE, 'kinematics_check.cpp'), os.path.join(REPO, 'lib', 'kinematics', 'kinematics.cpp'),
                '-o', exe], check=True)
rnd = random.Random(7)
lines, checks = [], []

# Forward kinematics and commands.
for _ in range(2000):
    a = [D(rnd.uniform(-80, 80)) for _ in range(arm.N)]
    lines.append('FK ' + ' '.join('%.7f' % v for v in a))
    pose = arm.forward(a)
    cmd = arm.commands(a)
    checks.append(('FK', pose.tip, pose.claw_direction, [cmd[n] for n in arm.JOINT_NAMES]))

# Readings -> angles.
n_rd = 0
while n_rd < 500:
    a = [D(rnd.uniform(-80, 80)) for _ in range(arm.N)]
    a[0] = D(rnd.uniform(-130, 130))
    r = arm.imu_angles(arm.forward(a))
    lines.append('RD ' + ' '.join('%.7f' % r[n] for n in arm.JOINT_NAMES))
    checks.append(('RD', a))
    n_rd += 1

# Plain solves and claw poses, on targets known to be reachable (made by forward kinematics).
limits = arm.joint_limits()
n_ik = 0
while n_ik < 300:
    a = [rnd.uniform(max(lo, -1.3), min(hi, 1.3)) for lo, hi in limits]
    a[4] = rnd.uniform(-0.4, 0.4)
    pose = arm.forward(a)
    if pose.lowest() < arm.FLOOR_CLEARANCE + 10:
        continue
    d = pose.claw_direction
    pitch = math.degrees(math.acos(max(-1, min(1, -d[2]))))
    az = math.degrees(math.atan2(d[1], d[0]))
    seeded = rnd.random() < 0.5
    seed = [v + rnd.uniform(-0.15, 0.15) for v in a] if seeded else [0.0] * arm.N
    roll = rnd.choice([None, rnd.uniform(-90, 90)])
    lines.append('IK %.4f %.4f %.4f %.5f %.5f %s %d %s 0' % (pose.tip + (pitch, az, 'nan' if roll is None else '%.4f' % roll,
                                                              int(seeded), ' '.join('%.7f' % v for v in seed))))
    sol = arm.solve_pose(pose.tip, pitch, az, roll=roll, seed=seed if seeded else None)
    checks.append(('IK', sol, pose.tip))
    n_ik += 1

out = subprocess.run([exe], input='\n'.join(lines) + '\n', capture_output=True, text=True, check=True).stdout.splitlines()
assert len(out) == len(checks), (len(out), len(checks))
worst = {'fk_tip': 0, 'fk_dir': 0, 'fk_cmd': 0, 'rd': 0}
ik_ok_py = ik_ok_cpp = ik_both = 0
ik_angle = []
fails = []
for line, chk in zip(out, checks):
    v = [float(x) for x in line.split()]
    if chk[0] == 'FK':
        worst['fk_tip'] = max(worst['fk_tip'], vec.norm(vec.sub(chk[1], v[0:3])))
        worst['fk_dir'] = max(worst['fk_dir'], vec.norm(vec.sub(chk[2], v[3:6])))
        worst['fk_cmd'] = max(worst['fk_cmd'], max(abs(x - y) for x, y in zip(chk[3], v[6:13])))
    elif chk[0] == 'RD':
        worst['rd'] = max(worst['rd'], max(abs(vec.wrap(x - y)) for x, y in zip(chk[1], v[0:7])))
    else:
        sol, tip = chk[1], chk[2]
        pe, de = v[7], v[8]
        py_ok = sol.position_error < 1 and sol.direction_error < 1
        cpp_ok = pe < 1 and de < 1
        ik_ok_py += py_ok
        ik_ok_cpp += cpp_ok
        if py_ok and cpp_ok:
            ik_both += 1
            # Without a seed the two can pick mirror poses (shoulder-z + 180, every pitch and the side bend
            # negated) - the same tip and claw, both right. The host always has the arm's pose as a seed.
            mirror = [vec.wrap(v[0] + math.pi)] + [-x for x in v[1:6]]
            ik_angle.append(min(max(abs(math.degrees(vec.wrap(x - y))) for x, y in zip(sol.angles[:6], c))
                                for c in (v[0:6], mirror)))
        if py_ok and not cpp_ok:
            fails.append((tip, sol.position_error, pe, de))
print('forward: tip %.4f mm, direction %.6f, commands %.4f deg (worst)' % (worst['fk_tip'], worst['fk_dir'], worst['fk_cmd']))
print('readings -> angles: %.4f deg (worst)' % math.degrees(worst['rd']))
ik_angle.sort()
print('IK: python reached %d/300, C++ %d/300, both %d; joint angle difference median %.3f, p95 %.3f, max %.3f deg' % (
    ik_ok_py, ik_ok_cpp, ik_both, ik_angle[len(ik_angle) // 2], ik_angle[int(len(ik_angle) * 0.95)], ik_angle[-1]))
for f in fails[:5]:
    print('  C++ missed', f)
ok = worst['fk_tip'] < 0.1 and worst['fk_cmd'] < 0.05 and math.degrees(worst['rd']) < 0.05 and ik_ok_cpp >= ik_ok_py
print('PASS' if ok else 'FAIL')
sys.exit(0 if ok else 1)
