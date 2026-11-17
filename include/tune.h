#pragma once

#include <stdint.h>

// Runtime tuning of the joint modules over CAN (CAN_TUNE): POST /tune
//   {"joint": "elbow-y", "set": {"dampingGain": 0, "ditherDeg": 0.3}}  - sets, then reads them all back
//   {"joint": "elbow-y"}                                              - reads them all
//   {"save": true}   - keeps the profile (every value set so far) across the host's reboots
//   {"clear": true}  - forgets the profile, saved too; the modules keep what they have until they reboot
// Joints: shoulder-y, shoulder-z, elbow-y, wrist-y, wrist-z. Every reply carries the profile.
// A module keeps a value in RAM only, so whenever one is heard again after a silence - it rebooted, or
// the host did - the host sends it its part of the profile (2026-11-17: no reflash for a parameter).
namespace Tune {

enum Module { SHOULDER = 0, ELBOW = 1, WRIST = 2, MODULES = 3 };

void begin();
// A module's answer (a *_TUNED frame's 8 bytes), from the CAN receive task.
void onReply(const uint8_t data[8]);
// A module's status frame came in, from the CAN receive task.
void heard(Module m);

}  // namespace Tune
