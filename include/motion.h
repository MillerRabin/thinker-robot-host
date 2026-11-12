#pragma once

// Moves the claw by itself: the host takes a target for the claw (POST /move), works out the joints with
// lib/kinematics, plans the whole path, checks every point, and streams it to the modules over CAN as
// stream waypoints (the modules' Servo::streamTo(): a set frame with a short timeMS). Over WiFi only the
// target travels - the waypoints never do (a 518 ms WiFi stall broke a PC-streamed line, 2026-11-06).

#include <ArduinoJson.h>
#include "kinematics.h"

namespace Motion {

struct Request {
  kin::V3 target;    // the claw tip, mm, platform frame (+x the platform's back, z up from the table)
  float pitch;       // the claw's tilt from straight down, deg (0 down, 90 horizontal)
  float azimuth;     // toward where it tilts, deg from +x toward +y
  bool hasRoll;      // whether to turn the jaws
  float roll;        // the jaws' line on the table, deg from +x
  uint32_t timeMS;   // how long the move takes
  bool line;         // the tip along a straight line; else the joints all at once (a free path)
  // The share of the time spent braking (0.5..0.85); NaN = a minimum-jerk curve (brakes as hard as
  // it starts). A longer, gentler stop for the arm's ~4 Hz swing, which is worst while braking
  // (2026-11-12).
  float brake;
  // Instead of a claw target: joint angles (deg from the calibration pose, kin::Joint order, NaN =
  // stay), always a joint path. The way back to vertical (all 0): a module's own planned move keeps
  // the command-vs-IMU offset it starts with, which holds its parent's tilt and its gravity sag, so
  // each joint overshot its target on the way up (2026-11-07); this one is planned and streamed like
  // any host move.
  bool jointTarget;
  float jointDeg[kin::N];
};

// Where the arm is: the calibrated homes and its joint angles, from the modules' broadcasts. False, with
// why, when a module isn't calibrated or isn't heard.
bool readArm(kin::Homes &homes, float angles[kin::N], const char **why);

// Plans the move; when every point is fine, starts it (replacing a running one). The reply says either
// way.
bool start(const Request &request, JsonObject reply);
void stop();
bool running();

// The streaming task and the HTTP endpoints: POST /move, /stop, /pose.
void begin();

}  // namespace Motion
