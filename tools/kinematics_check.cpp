// Checks lib/kinematics against the Python reference (thinker-robot-kinematics): reads cases on stdin,
// prints the C++ answers; tools/kinematics_check.py generates the cases and compares.
//   FK a0..a6                         -> tip, claw direction, commands (default homes)
//   RD r0..r6                         -> joint angles from module readings
//   IK tx ty tz pitch az roll|nan seeded(0/1) s0..s6 seedOnly(0/1) -> angles, errors, flags, azimuth turn
//   SV tx ty tz dx dy dz azfree(0/1)  -> plain solve(), no seed
// Build: g++ -O2 -std=c++17 -Ilib/kinematics tools/kinematics_check.cpp lib/kinematics/kinematics.cpp
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include "kinematics.h"

using namespace kin;

int main() {
  char op[8];
  while (std::scanf("%7s", op) == 1) {
    if (!std::strcmp(op, "FK")) {
      float a[N];
      for (float &v : a) std::scanf("%f", &v);
      Pose p;
      forward(a, p);
      V3 d = p.clawDirection();
      float c[N];
      commands(a, DEFAULT_HOMES, c);
      std::printf("%.4f %.4f %.4f %.6f %.6f %.6f", p.tip.x, p.tip.y, p.tip.z, d.x, d.y, d.z);
      for (float v : c) std::printf(" %.4f", v);
      std::printf("\n");
    } else if (!std::strcmp(op, "RD")) {
      float r[N], out[N];
      for (float &v : r) std::scanf("%f", &v);
      anglesFromReadings(r, nullptr, out);
      for (float v : out) std::printf("%.6f ", v);
      std::printf("\n");
    } else if (!std::strcmp(op, "IK")) {
      float t[3], pitch, az, roll, seed[N];
      char rollS[32];
      int seeded, seedOnly;
      std::scanf("%f %f %f %f %f %31s %d", &t[0], &t[1], &t[2], &pitch, &az, rollS, &seeded);
      for (float &v : seed) std::scanf("%f", &v);
      std::scanf("%d", &seedOnly);
      bool hasRoll = std::strcmp(rollS, "nan") != 0;
      roll = hasRoll ? std::strtof(rollS, nullptr) : 0;
      Solution s;
      solvePose({t[0], t[1], t[2]}, pitch, az, hasRoll ? &roll : nullptr, DEFAULT_HOMES, seeded ? seed : nullptr,
                seedOnly != 0, s);
      for (float v : s.angles) std::printf("%.6f ", v);
      std::printf("%.4f %.4f %u %.3f %d\n", s.positionError, s.directionError, s.flags, s.azimuthTurn, s.iterations);
    } else if (!std::strcmp(op, "SV")) {
      float t[3], d[3];
      int azfree;
      std::scanf("%f %f %f %f %f %f %d", &t[0], &t[1], &t[2], &d[0], &d[1], &d[2], &azfree);
      Solution s;
      V3 dir{d[0], d[1], d[2]};
      solve({t[0], t[1], t[2]}, &dir, DEFAULT_HOMES, nullptr, false, azfree != 0, s);
      for (float v : s.angles) std::printf("%.6f ", v);
      std::printf("%.4f %.4f %u\n", s.positionError, s.directionError, s.flags);
    }
  }
  return 0;
}
