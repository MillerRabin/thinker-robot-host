#pragma once

// Kinematic model of the arm: forward and inverse kinematics and the /set commands that hold a pose.
// A port of thinker-robot-kinematics/thinker_kinematics/arm.py (the reference - keep them in step and
// re-run tools/kinematics_check against it after any change). Plain C++ on float, no Arduino: it
// builds and is checked on the PC too.
//
// Frame W, mm: origin on the platform's base under the shoulder-z axis, z up; y the common axis of the
// pitch joints; x = y cross z, the side a growing pitch command tilts a link toward - the platform's
// back. Home pose (all joint angles 0) is the calibration pose: every link vertical, claw up. Joint
// angles are relative (child to parent), radians, in Joint order.
//
// What a command means (module firmware, `use` mode): shoulder-y, elbow-y, wrist-y/z hold an ABSOLUTE
// link angle since calibration (command = home + that reading, see imuReadings()); shoulder-z and
// claw-y/x run open-loop (command = home + the joint angle). home = what the calibration measured.

#include <cmath>
#include <cstdint>

namespace kin {

constexpr int N = 7;
enum Joint { SHOULDER_Z = 0, SHOULDER_Y, ELBOW_Y, WRIST_Y, WRIST_Z, CLAW_Y, CLAW_X };
extern const char *const JOINT_NAMES[N];  // the host's /set keys

struct V3 {
  float x, y, z;
};
struct Q {  // unit quaternion
  float w, x, y, z;
};
struct Frame {  // p -> q p q* + t
  Q q;
  V3 t;
};

// Axis-to-axis distances, mm, measured by hand on 2026-11-05.
constexpr float BASE_TO_SHOULDER_Z = 156.0f;
constexpr float SHOULDER_Z_TO_SHOULDER_Y = 55.0f;
constexpr float SHOULDER_Y_TO_ELBOW_Y = 94.0f;
constexpr float ELBOW_Y_TO_WRIST_Y = 94.0f;
constexpr float WRIST_Y_TO_WRIST_Z = 94.0f;
constexpr float WRIST_Z_TO_CLAW_Y = 94.0f;
constexpr float CLAW_Y_TO_CLAW_X = 84.0f;
constexpr float CLAW_X_TO_TIP = 110.0f;

// The command (deg) that holds each joint's home - what the calibration measured.
struct Homes {
  float deg[N];
};
extern const Homes DEFAULT_HOMES;

struct Pose {
  Frame frames[N];  // frames[k] moves the link after joint k from home
  V3 joints[N];     // where each joint's axis passes now
  V3 axes[N];       // each joint's axis now
  V3 tip;
  V3 clawDirection() const;   // unit, from claw-x toward the tip
  float lowest() const;       // lowest joint (elbow on) or tip height
};

// Lowest any joint or the tip may go: z = 0 is the table the platform stands on.
constexpr float FLOOR_CLEARANCE = 20.0f;
// Stay this far inside each command range.
constexpr float RANGE_MARGIN_DEG = 2.0f;

// The rotation angle of q about axis (swing-twist), as the module firmware's Quaternion::twistAngle().
float twistAngle(const Q &q, const V3 &axis);
// The wrist firmware's (pitch, side bend) from its rotation q and the arm's turn yaw, with its frame's
// forward (its Z axis), side and up axes - W's x, y, z for the model, its own body axes for its broadcast.
void wristAngles(const Q &q, float yaw, const V3 &fwd, const V3 &side, const V3 &up, float &pitch, float &bend);

void forward(const float angles[N], Pose &pose);
// What each module reads for this pose (radians), as its firmware computes it; claw: its joint angles.
void imuReadings(const Pose &pose, float out[N]);
// The /set commands (deg) that hold these joint angles.
void commands(const float angles[N], const Homes &homes, float out[N]);
// -1 when every command is in its module's range, else the first joint that isn't.
int commandOutOfRange(const float cmds[N]);
// Joint angles from module readings - the inverse of imuReadings(). seed may be null.
void anglesFromReadings(const float readings[N], const float *seed, float out[N]);
void jointLimits(const Homes &homes, float lo[N], float hi[N]);

// --- Inverse kinematics ------------------------------------------------------------------------
enum SolveFlags : uint32_t {
  FLAG_FLOOR = 1,              // a joint or the tip goes below FLOOR_CLEARANCE
  FLAG_AZIMUTH_TURNED = 2,     // the claw was let turn in azimuth (see solvePose())
  FLAG_ROLL_UNDEFINED = 4,     // the claw points along the asked jaw line
  FLAG_ROLL_OUT_OF_RANGE = 8,  // claw-x can't turn the jaws that far
};

struct Solution {
  float angles[N];
  Pose pose;
  float positionError = 0;    // mm
  float directionError = 0;   // deg (the tilt's, when the azimuth was left free)
  float azimuthTurn = 0;      // deg the claw turned in azimuth, with FLAG_AZIMUTH_TURNED
  int iterations = 0;
  uint32_t flags = 0;
  bool ok() const { return positionError < 1.0f && directionError < 1.0f && !(flags & FLAG_FLOOR); }
};

// The claw axis for a tilt from straight down (deg: 0 down, 90 horizontal, 180 up) toward an azimuth
// (deg from +x toward +y).
V3 clawDirection(float pitchDeg, float azimuthDeg);

// Joint angles that put the tip at target and, with direction, point the claw along it. seed: where
// the arm is (null for none); seedOnly: start from it alone (a path's next point mustn't jump to another
// posture); azimuthFree: only the claw's tilt from the vertical is held to direction.
void solve(const V3 &target, const V3 *direction, const Homes &homes, const float *seed, bool seedOnly,
           bool azimuthFree, Solution &out, int maxIter = 400);

// Turn the jaws' line (claw-x) to a heading on the table (deg from +x). Returns the flag that stopped
// it, or 0.
uint32_t jawHeading(float angles[N], float headingDeg, const Homes &homes);

// The tip at target, the claw tilted pitch toward azimuth, the jaws along roll (null = leave claw-x).
// Exact unless that costs more than MAX_TURN_PER_MM of turning per mm the tip moves from seed - then the
// claw may turn in azimuth (FLAG_AZIMUTH_TURNED, azimuthTurn).
void solvePose(const V3 &target, float pitchDeg, float azimuthDeg, const float *rollDeg, const Homes &homes,
               const float *seed, bool seedOnly, Solution &out);

}  // namespace kin
