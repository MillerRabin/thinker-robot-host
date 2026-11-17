#include "motion.h"

#include <Arduino.h>
#include <AsyncJson.h>
#include <cmath>

#include "arm.h"
#include "config.h"
#include "server.h"

using namespace kin;

namespace {

constexpr uint32_t INTERVAL_MS = 40;        // a waypoint every 40 ms
constexpr uint16_t STEP_TIME_MS = 60;       // its timeMS: over the interval (a late one doesn't stop the
                                            // joint), under the modules' STREAM_MAX_MS 250
constexpr int MAX_POINTS = 400;             // 16 s
constexpr float MAX_JOINT_SPEED = 60.0f;    // deg/s of any command between waypoints
constexpr uint64_t STATUS_CALIBRATED = 0x100;
// The module's IMU-held Y joint is calming after a stream, its command held (the modules'
// ARM_STREAM_WAITING).
constexpr uint64_t STATUS_STREAM_WAITING = 0x800;
constexpr float SHOULDER_Z_HOME = 135.0f;   // shoulder-z's home never changes
constexpr float DEG = 3.14159265358979f / 180.0f;

// The wrist's frame in its own IMU's corrected frame (module firmware: WRIST_Z_JOINT_AXIS,
// WRIST_UP_AXIS, WRIST_SIDE_AXIS = up x Z) - to read its broadcast the way it does.
const V3 WRIST_FWD = {-0.221f, 0.208f, 0.953f};
const V3 WRIST_UP = {-0.8026f, -0.5938f, -0.0565f};
const V3 WRIST_SIDE = {WRIST_UP.y * WRIST_FWD.z - WRIST_UP.z * WRIST_FWD.y,
                       WRIST_UP.z * WRIST_FWD.x - WRIST_UP.x * WRIST_FWD.z,
                       WRIST_UP.x * WRIST_FWD.y - WRIST_UP.y * WRIST_FWD.x};

// The plan: each waypoint's commands (deg, NaN = leave that joint). Written by start() under the lock,
// read by the streaming task.
float plan[MAX_POINTS][N];
volatile int planCount = 0;
volatile int planNext = 0;
volatile bool planActive = false;
portMUX_TYPE planLock = portMUX_INITIALIZER_UNLOCKED;
// claw-x runs open-loop and isn't published: the last command this module sent it, NaN = never.
float lastClawX = NAN;

// End-of-move overshoot, learned per joint and direction ([0] toward growing angles, [1] falling) in
// joint angles, deg: how far each joint ran past its goal once the move ended (+ = toward growing
// angles). Sent with every waypoint, so the module ends that joint's setpoint short by it (see the
// modules' Servo::setStreamEndShift()). Measured live (2026-11-11/12): braking, a link rolls across its
// backlash 1-4.5deg past where the command stops; each module learning it from the absolute angle it
// holds took its parent's overrun for its own - the wrist's shift flipped sign move to move.
// Taken as the mean over the time all three modules hold their commands while their links calm (the
// modules' Servo::streamWaitTick(), 0.5-3 s), up to the first one starting to settle - which moves
// its children's joint angles. Measured live (2026-11-13), a single reading 300 ms after the last
// waypoint landed on a random phase of the ~2 s swing at the top of a move back to vertical: the
// shift never converged and the bigger it got, the further past vertical it drove the arm.
// The rate: at 0.7 (2026-11-17) the shifts chased each move's own scatter - a joint's end varies by
// ~+-0.5deg move to move, the shoulder's carrying into the elbow's and the wrist's joint angles - the
// elbow's shift back up went 1.46, 1.20, 0.55 in three moves.
constexpr float LEARN_RATE = 0.4f;
constexpr float LEARN_MIN_TRAVEL_DEG = 4.0f;
constexpr float LEARN_MAX_DEG = 6.0f;
constexpr uint32_t LEARN_START_MS = 1500;  // after the last waypoint, the modules have long reported
                                           // waiting, or never will (old firmware)
constexpr uint32_t LEARN_MAX_MS = 5000;    // a window still open this long ends anyway
constexpr int LEARN_MIN_SAMPLES = 5;       // INTERVAL_MS each
const int LEARN_JOINTS[] = {SHOULDER_Y, ELBOW_Y, WRIST_Y};
float learned[N][2] = {};
// The parking pose ({"home": true}): the calibration pose with the forearm and the claw bent forward
// this far, so every Y joint carries a share of the weight beyond it and its gears stay pressed to
// one side. Measured live (2026-11-17): stopping at vertical, the links rolled across their backlash
// as the load changed sides - in the last 1.5 s of a move back up the elbow swung -4..+6deg about its
// setpoint and the wrist +-7, where the arm stopped depended on the phase of that swing (2 of 4 moves
// stopped 2-4deg short, the settle then threw the tip from +69 to -79 mm), and no learned shift could
// fix it. The whole arm leaned 4.5deg forward still left the elbow and the wrist nearly unloaded: one
// move stopped with the wrist 7.5deg short and its settle took the tip 99 mm. Bent 15+15 the joints
// calmed within 0.5 s of the stop and the tip ended within 9 mm of the target, without learning.
constexpr float PARK_ELBOW_DEG = 15.0f;
constexpr float PARK_WRIST_DEG = 15.0f;
float lastOverrun[N] = {NAN, NAN, NAN, NAN, NAN, NAN, NAN};
const char *learnWhy = nullptr;  // why the last move taught nothing
// The window: joint angles summed while all three modules report waiting.
double learnSum[N] = {};
int learnSamples = 0;
int lastLearnSamples = 0;
bool learnWindowOpen = false;
// The running move's: the shift each joint gets, its goal (rad) and which way it travels (-1: not
// learned this move). Written by start() under the lock with the plan.
float planShift[N] = {};
float planGoal[N] = {};
int planWay[N] = {-1, -1, -1, -1, -1, -1, -1};
volatile bool learnPending = false;
TickType_t lastPointAt = 0;

float slot(uint64_t status, int shift) {
  const uint16_t raw = (status >> shift) & 0xFFFF;
  return raw == 0xFFFF ? NAN : raw / 100.0f;
}

Q toQ(const Quaternion &q) { return {q.real, q.i, q.j, q.k}; }

float minJerk(float t) { return t * t * t * (10.0f + t * (-15.0f + 6.0f * t)); }

// Progress 0..1 at time t (0..1): a minimum-jerk curve, or with brake (the share of the time spent
// slowing down) a speed that rises over 1 - brake and falls over brake, each along a smoothstep - zero
// acceleration at the start, the peak and the end. Peak speed 2, deceleration 3 / brake (min-jerk:
// 1.875 and 5.77); acceleration 3 / (1 - brake).
float profile(float t, float brake) {
  if (std::isnan(brake)) {
    return minJerk(t);
  }
  auto area = [](float x) { return x * x * x - 0.5f * x * x * x * x; };  // integral of 3x^2 - 2x^3
  const float rise = 1.0f - brake;
  if (t <= rise) {
    return 2.0f * rise * area(t / rise);
  }
  const float x = (t - rise) / brake;
  return std::min(1.0f, rise + 2.0f * (t - rise) - 2.0f * brake * area(x));
}

int16_t angle10(float deg) { return std::isnan(deg) ? (int16_t)PARAMETER_IS_NAN : (int16_t)lroundf(deg * 10.0f); }

// A shift in 0.01deg; -0.01 would read as the NaN marker, so it goes as 0.
int16_t shift100(float deg) {
  const long v = lroundf(deg * 100.0f);
  return (int16_t)((v == -1) ? 0 : std::max(-30000L, std::min(30000L, v)));
}

// One waypoint to every module: the same set frames as /set (ArmDataFrame, angles x10), with a stream
// timeMS.
void sendWaypoint(const float c[N]) {
  ArmDataFrame f{};
  // The modules' joint angles add up along the arm with the same sign as theirs, so a joint's overrun
  // in joint angles is its module's too.
  f.values = {angle10(c[SHOULDER_Y]), angle10(c[SHOULDER_Z]), (int16_t)STEP_TIME_MS, shift100(planShift[SHOULDER_Y])};
  Arm::twai.sendData(CAN_SHOULDER_SET_YZ_DEGREE, f.bytes);
  f.values = {angle10(c[ELBOW_Y]), (int16_t)STEP_TIME_MS, shift100(planShift[ELBOW_Y]), (int16_t)PARAMETER_IS_NAN};
  Arm::twai.sendData(CAN_ELBOW_SET_Y_DEGREE, f.bytes);
  f.values = {angle10(c[WRIST_Y]), angle10(c[WRIST_Z]), (int16_t)STEP_TIME_MS, shift100(planShift[WRIST_Y])};
  Arm::twai.sendData(CAN_WRIST_SET_YZ_DEGREE, f.bytes);
  f.values = {angle10(c[CLAW_Y]), angle10(c[CLAW_X]), (int16_t)PARAMETER_IS_NAN, (int16_t)STEP_TIME_MS};
  Arm::twai.sendData(CAN_CLAW_SET_XYG_DEGREE, f.bytes);
  if (!std::isnan(c[CLAW_X])) {
    lastClawX = c[CLAW_X];
  }
}

bool learnTick();

void streamTask(void *) {
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&last, pdMS_TO_TICKS(INTERVAL_MS));
    float c[N];
    bool send = false;
    bool learn = false;
    portENTER_CRITICAL(&planLock);
    if (planActive) {
      if (!PowerManagement::getEnginesStatus() || planNext >= planCount) {
        planActive = false;
      } else {
        std::copy(plan[planNext], plan[planNext] + N, c);
        planNext = planNext + 1;
        send = true;
        if (planNext >= planCount) {
          lastPointAt = xTaskGetTickCount();
          learnPending = PowerManagement::getEnginesStatus();
          learnWindowOpen = false;
          learnSamples = 0;
          std::fill(learnSum, learnSum + N, 0.0);
        }
      }
    } else if (learnPending) {
      learn = true;
    }
    portEXIT_CRITICAL(&planLock);
    if (send) {
      sendWaypoint(c);
    }
    if (learn && !learnTick()) {
      learnPending = false;
    }
  }
}

// One step of the window after a move: sums the joint angles while all three modules report waiting
// (see LEARN_RATE), and once the first stops, learns how far each joint ended past its goal into
// learned[]. False once the window is over, either way.
bool learnTick() {
  const uint64_t waiting = Arm::status.shoulderStatuses & Arm::status.elbowStatuses &
                           Arm::status.wristStatuses & STATUS_STREAM_WAITING;
  const uint32_t sinceMs = (xTaskGetTickCount() - lastPointAt) * portTICK_PERIOD_MS;
  if (waiting && sinceMs < LEARN_MAX_MS) {
    Homes homes;
    float a[N];
    const char *why = nullptr;
    if (!Motion::readArm(homes, a, &why)) {
      learnWhy = why;  // a status dropout: skip the sample
      return true;
    }
    learnWindowOpen = true;
    for (int j : LEARN_JOINTS) {
      learnSum[j] += a[j];
    }
    learnSamples++;
    return true;
  }
  if (!learnWindowOpen) {
    if (sinceMs < LEARN_START_MS) {
      return true;
    }
    learnWhy = "the modules never reported waiting for the arm to calm (old firmware?)";
    return false;
  }
  lastLearnSamples = learnSamples;
  if (learnSamples < LEARN_MIN_SAMPLES) {
    learnWhy = "the arm calmed too briefly to learn from";
    return false;
  }
  learnWhy = nullptr;
  for (int j : LEARN_JOINTS) {
    if (planWay[j] < 0) {
      continue;
    }
    const float overrun = (float)(learnSum[j] / learnSamples - planGoal[j]) / DEG;
    lastOverrun[j] = overrun;
    float &l = learned[j][planWay[j]];
    l = std::max(-LEARN_MAX_DEG, std::min(LEARN_MAX_DEG, l + LEARN_RATE * overrun));
  }
  return false;
}

bool fail(JsonObject reply, const char *why) {
  reply["ok"] = false;
  reply["error"] = why;
  return false;
}

void putV3(JsonObject o, const char *key, const V3 &v) {
  JsonArray a = o.createNestedArray(key);
  a.add(v.x), a.add(v.y), a.add(v.z);
}

bool getFloat(JsonObject o, const char *key, float &out) {
  if (!o.containsKey(key) || o[key].isNull()) return false;
  out = o[key].as<float>();
  return !std::isnan(out);
}

}  // namespace

namespace Motion {

bool readArm(Homes &homes, float angles[N], const char **why) {
  const uint64_t s = Arm::status.shoulderStatuses, e = Arm::status.elbowStatuses,
                 w = Arm::status.wristStatuses, c = Arm::status.clawStatuses;
  if (!(s & e & w & c & STATUS_CALIBRATED)) {
    *why = "a module isn't calibrated";
    return false;
  }
  if (!Arm::status.shoulderQuaternionOK || !Arm::status.elbowQuaternionOK || !Arm::status.wristQuaternionOK) {
    *why = "a module's IMU isn't heard";
    return false;
  }
  homes = DEFAULT_HOMES;
  homes.deg[SHOULDER_Z] = SHOULDER_Z_HOME;
  homes.deg[SHOULDER_Y] = slot(s, 32);
  homes.deg[ELBOW_Y] = slot(e, 32);
  homes.deg[WRIST_Y] = slot(w, 32);
  homes.deg[WRIST_Z] = slot(w, 48);
  homes.deg[CLAW_Y] = slot(c, 32);
  // Shoulder Z and claw Y run open-loop: their commands are their angles (their IMUs drift or only
  // estimate them), published in the status's second slot.
  const float zCommand = slot(s, 48), clawYCommand = slot(c, 48);
  for (float v : homes.deg) {
    if (std::isnan(v)) {
      *why = "a module doesn't publish its homes (old firmware?)";
      return false;
    }
  }
  if (std::isnan(zCommand) || std::isnan(clawYCommand)) {
    *why = "shoulder-z's or claw-y's command isn't published";
    return false;
  }
  const Q sq = toQ(Arm::shoulder.imu.quaternion), eq = toQ(Arm::elbow.imu.quaternion),
          wq = toQ(Arm::wrist.imu.quaternion);
  float r[N];
  r[SHOULDER_Z] = (zCommand - SHOULDER_Z_HOME) * DEG;
  r[SHOULDER_Y] = twistAngle(sq, {0, 1, 0});
  r[ELBOW_Y] = twistAngle(eq, {0, 1, 0});
  // The wrist takes its angles after undoing the shoulder IMU's yaw, as its firmware does.
  wristAngles(wq, twistAngle(sq, {0, 0, 1}), WRIST_FWD, WRIST_SIDE, WRIST_UP, r[WRIST_Y], r[WRIST_Z]);
  r[CLAW_Y] = (clawYCommand - homes.deg[CLAW_Y]) * DEG;
  r[CLAW_X] = std::isnan(lastClawX) ? 0.0f : (lastClawX - homes.deg[CLAW_X]) * DEG;
  anglesFromReadings(r, nullptr, angles);
  return true;
}

bool start(const Request &q, JsonObject reply) {
  if (!PowerManagement::getEnginesStatus()) {
    return fail(reply, "the engines are off");
  }
  Homes homes;
  float a0[N];
  const char *why = nullptr;
  if (!readArm(homes, a0, &why)) {
    return fail(reply, why);
  }
  Solution goal;
  bool moveClawX = q.hasRoll;
  if (q.jointTarget) {
    for (int j = 0; j < N; j++) {
      goal.angles[j] = std::isnan(q.jointDeg[j]) ? a0[j] : q.jointDeg[j] * DEG;
    }
    moveClawX = !std::isnan(q.jointDeg[CLAW_X]);
    forward(goal.angles, goal.pose);
    if (goal.pose.lowest() < FLOOR_CLEARANCE) {
      return fail(reply, "a joint or the tip would go below the table clearance");
    }
  } else {
    solvePose(q.target, q.pitch, q.azimuth, q.hasRoll ? &q.roll : nullptr, homes, a0, false, goal);
    reply["positionError"] = goal.positionError;
    reply["directionError"] = goal.directionError;
    if (goal.flags & FLAG_AZIMUTH_TURNED) {
      reply["azimuthTurn"] = goal.azimuthTurn;
    }
    if (!goal.ok()) {
      return fail(reply, (goal.flags & FLAG_FLOOR) ? "a joint or the tip would go below the table clearance"
                                                   : "out of reach");
    }
    if (q.hasRoll && (goal.flags & (FLAG_ROLL_UNDEFINED | FLAG_ROLL_OUT_OF_RANGE))) {
      return fail(reply, (goal.flags & FLAG_ROLL_UNDEFINED) ? "the claw points along the asked jaw line"
                                                            : "claw-x can't turn the jaws that far");
    }
  }
  const int n = std::max<int>(2, (int)std::ceil(q.timeMS / (float)INTERVAL_MS));
  if (n > MAX_POINTS) {
    return fail(reply, "the move is too long");
  }
  Pose start;
  forward(a0, start);
  const V3 tip0 = start.tip, d0 = start.clawDirection(), d1 = goal.pose.clawDirection();
  static float next[MAX_POINTS][N];
  float prev[N], prevCmd[N];
  std::copy(a0, a0 + N, prev);
  commands(a0, homes, prevCmd);
  float topSpeed = 0;
  for (int k = 1; k <= n; k++) {
    const float s = profile(k / (float)n, q.brake);
    float a[N];
    if (q.line && !q.jointTarget) {
      // The tip along the line, the claw turning from where it points now to the goal's direction (the
      // goal already has any azimuth turn in it).
      const V3 p = {tip0.x + (q.target.x - tip0.x) * s, tip0.y + (q.target.y - tip0.y) * s,
                    tip0.z + (q.target.z - tip0.z) * s};
      V3 d = {d0.x + (d1.x - d0.x) * s, d0.y + (d1.y - d0.y) * s, d0.z + (d1.z - d0.z) * s};
      const float len = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
      d = {d.x / len, d.y / len, d.z / len};
      Solution sol;
      solve(p, &d, homes, prev, true, false, sol, 100);
      if (!sol.ok()) {
        reply["point"] = k;
        return fail(reply, "a point on the line is out of reach");
      }
      std::copy(sol.angles, sol.angles + N, a);
      a[CLAW_X] = a0[CLAW_X] + (goal.angles[CLAW_X] - a0[CLAW_X]) * s;
    } else {
      for (int j = 0; j < N; j++) a[j] = a0[j] + (goal.angles[j] - a0[j]) * s;
      Pose p;
      forward(a, p);
      if (p.lowest() < FLOOR_CLEARANCE) {
        reply["point"] = k;
        return fail(reply, "the path goes below the table clearance - try path \"line\"");
      }
    }
    float c[N];
    commands(a, homes, c);
    const int bad = commandOutOfRange(c);
    if (bad >= 0) {
      reply["point"] = k;
      reply["joint"] = JOINT_NAMES[bad];
      return fail(reply, "a command is out of its module's range");
    }
    for (int j = 0; j < N; j++) {
      topSpeed = std::max(topSpeed, std::fabs(c[j] - prevCmd[j]) / (INTERVAL_MS / 1000.0f));
    }
    if (!moveClawX) {
      c[CLAW_X] = NAN;  // leave the jaws as they are
    }
    std::copy(c, c + N, next[k - 1]);
    std::copy(a, a + N, prev);
    commands(a, homes, prevCmd);
  }
  if (topSpeed > MAX_JOINT_SPEED) {
    reply["topSpeed"] = topSpeed;
    return fail(reply, "a joint would move too fast - give it more time");
  }
  portENTER_CRITICAL(&planLock);
  std::copy(&next[0][0], &next[0][0] + n * N, &plan[0][0]);
  planCount = n;
  planNext = 0;
  planActive = true;
  learnPending = false;
  for (int j = 0; j < N; j++) {
    planShift[j] = 0.0f;
    planWay[j] = -1;
    planGoal[j] = goal.angles[j];
  }
  for (int j : LEARN_JOINTS) {
    const float travel = (goal.angles[j] - a0[j]) / DEG;
    // Not the shoulder on the way back up (falling angles): measured live (2026-11-17), it ended
    // 0.8-0.9deg past whatever the shift - 0, 0.32, 0.69, 1.00 - until at 1.97 it stopped 2.2deg short:
    // the shift only moved it across its backlash, and the settle after it took the claw 40 mm away.
    if (j == SHOULDER_Y && travel < 0.0f) {
      continue;
    }
    if (q.learn && std::fabs(travel) >= LEARN_MIN_TRAVEL_DEG) {
      planWay[j] = travel >= 0.0f ? 0 : 1;
      planShift[j] = learned[j][planWay[j]];
    }
  }
  portEXIT_CRITICAL(&planLock);
  JsonObject shifts = reply.createNestedObject("endShift");
  for (int j : LEARN_JOINTS) {
    if (planWay[j] >= 0) {
      shifts[JOINT_NAMES[j]] = planShift[j];
    }
  }
  reply["ok"] = true;
  reply["points"] = n;
  reply["topSpeed"] = topSpeed;
  putV3(reply, "from", tip0);
  putV3(reply, "to", goal.pose.tip);
  return true;
}

void stop() {
  portENTER_CRITICAL(&planLock);
  planActive = false;
  portEXIT_CRITICAL(&planLock);
}

bool running() { return planActive; }

void begin() {
  xTaskCreatePinnedToCore(streamTask, "Motion::stream", 4096, nullptr, 4, nullptr, 1);

  for (const char *path : {"/move", "/stop", "/pose", "/learn"}) {
    Server.on(path, HTTP_OPTIONS, [](AsyncWebServerRequest *request) { request->send(200); });
  }
  // {"x", "y", "z", "pitch", "azimuth" (default: toward the point), "roll" (optional), "timeMS",
  //  "path": "line" | "joint", "brake" (the share of the time spent braking, 0.5..0.85, optional)},
  //  or instead of the claw target {"home": true} (the parking pose: every joint but claw-x to its
  //  calibration pose, the elbow and the wrist bent forward - PARK_ELBOW_DEG), {"vertical": true}
  //  (the calibration pose itself - the arm vertical; never learned from) or {"angles": {"elbow-y": deg, ...}} (joint
  //  angles from the calibration pose, the joints not named stay) - all a joint path, all taking
  //  "timeMS" and "brake" too.
  //  "learn": false - no learned end shift and no learning from this move (Request::learn).
  // POST /learn sets the learned shifts (see below).
  auto *move = new AsyncCallbackJsonWebHandler("/move", [](AsyncWebServerRequest *request, JsonVariant json) {
    DynamicJsonDocument doc(768);
    JsonObject reply = doc.to<JsonObject>();
    JsonObject in = json.as<JsonObject>();
    Request q{};
    q.learn = in.isNull() || !in.containsKey("learn") || in["learn"].as<bool>();
    float timeMS = 3000;
    const bool home = !in.isNull() && in["home"].as<bool>();
    const bool vertical = !in.isNull() && in["vertical"].as<bool>();
    const bool angles = !in.isNull() && in["angles"].is<JsonObject>();
    if (home || vertical || angles) {
      q.jointTarget = true;
      for (int j = 0; j < N; j++) {
        q.jointDeg[j] = (home || vertical) && j != CLAW_X ? 0.0f : NAN;
      }
      if (home && !vertical) {
        q.jointDeg[ELBOW_Y] = PARK_ELBOW_DEG;
        q.jointDeg[WRIST_Y] = PARK_WRIST_DEG;
      }
      if (vertical) {
        q.learn = false;  // where a move to vertical stops is chance (see PARK_ELBOW_DEG)
      }
      if (angles) {
        JsonObject a = in["angles"].as<JsonObject>();
        for (int j = 0; j < N; j++) {
          getFloat(a, JOINT_NAMES[j], q.jointDeg[j]);
        }
      }
      getFloat(in, "timeMS", timeMS);
      q.timeMS = (uint32_t)std::max(200.0f, timeMS);
      if (!getFloat(in, "brake", q.brake)) {
        q.brake = NAN;
      } else {
        q.brake = std::min(0.85f, std::max(0.5f, q.brake));
      }
      const uint32_t t0 = millis();
      start(q, reply);
      reply["planMs"] = millis() - t0;
    } else if (in.isNull() || !getFloat(in, "x", q.target.x) || !getFloat(in, "y", q.target.y) ||
               !getFloat(in, "z", q.target.z) || !getFloat(in, "pitch", q.pitch)) {
      fail(reply, "need x, y, z and pitch, or home, or angles");
    } else {
      if (!getFloat(in, "azimuth", q.azimuth)) {
        q.azimuth = std::atan2(q.target.y, q.target.x) / DEG;
      }
      q.hasRoll = getFloat(in, "roll", q.roll);
      getFloat(in, "timeMS", timeMS);
      q.timeMS = (uint32_t)std::max(200.0f, timeMS);
      if (!getFloat(in, "brake", q.brake)) {
        q.brake = NAN;
      } else {
        q.brake = std::min(0.85f, std::max(0.5f, q.brake));
      }
      q.line = !(in.containsKey("path") && String(in["path"].as<const char *>()) == "joint");
      const uint32_t t0 = millis();
      start(q, reply);
      reply["planMs"] = millis() - t0;
    }
    AsyncResponseStream *resp = request->beginResponseStream("application/json");
    serializeJson(doc, *resp);
    request->send(resp);
  });
  Server.addHandler(move);

  // Sets the learned end-of-move overshoot (it lives in RAM, so a reboot forgets it): {"shoulder-y":
  // [growing, falling], "elbow-y": [...], "wrist-y": [...]}, any of them; replies with all of it.
  auto *learn = new AsyncCallbackJsonWebHandler("/learn", [](AsyncWebServerRequest *request, JsonVariant json) {
    DynamicJsonDocument doc(512);
    JsonObject reply = doc.to<JsonObject>();
    JsonObject in = json.as<JsonObject>();
    for (int j : LEARN_JOINTS) {
      if (in.isNull() || !in[JOINT_NAMES[j]].is<JsonArray>()) {
        continue;
      }
      JsonArray w = in[JOINT_NAMES[j]].as<JsonArray>();
      for (int k = 0; k < 2 && k < (int)w.size(); k++) {
        const float v = w[k].as<float>();
        if (!std::isnan(v)) {
          learned[j][k] = std::max(-LEARN_MAX_DEG, std::min(LEARN_MAX_DEG, v));
        }
      }
    }
    reply["ok"] = true;
    JsonObject l = reply.createNestedObject("learned");
    for (int j : LEARN_JOINTS) {
      JsonArray w = l.createNestedArray(JOINT_NAMES[j]);
      w.add(learned[j][0]);
      w.add(learned[j][1]);
    }
    AsyncResponseStream *resp = request->beginResponseStream("application/json");
    serializeJson(doc, *resp);
    request->send(resp);
  });
  Server.addHandler(learn);

  Server.on("/stop", HTTP_POST, [](AsyncWebServerRequest *request) {
    stop();
    request->send(200, "application/json", "{\"ok\":true}");
  });

  Server.on("/pose", HTTP_POST, [](AsyncWebServerRequest *request) {
    DynamicJsonDocument doc(1280);
    JsonObject reply = doc.to<JsonObject>();
    Homes homes;
    float a[N];
    const char *why = nullptr;
    if (!readArm(homes, a, &why)) {
      fail(reply, why);
    } else {
      Pose p;
      forward(a, p);
      reply["ok"] = true;
      reply["running"] = running();
      putV3(reply, "tip", p.tip);
      putV3(reply, "clawDirection", p.clawDirection());
      JsonObject angles = reply.createNestedObject("angles");
      JsonObject h = reply.createNestedObject("homes");
      for (int k = 0; k < N; k++) {
        angles[JOINT_NAMES[k]] = a[k] / DEG;
        h[JOINT_NAMES[k]] = homes.deg[k];
      }
      // The end-of-move overshoot learned per joint: [toward growing angles, falling], and the last
      // move's overrun (deg, joint angles) with how many readings it's the mean of.
      if (learnWhy != nullptr) {
        reply["learnError"] = learnWhy;
      }
      reply["learnSamples"] = lastLearnSamples;
      JsonObject l = reply.createNestedObject("learned");
      JsonObject o = reply.createNestedObject("lastOverrun");
      for (int j : LEARN_JOINTS) {
        JsonArray w = l.createNestedArray(JOINT_NAMES[j]);
        w.add(learned[j][0]);
        w.add(learned[j][1]);
        if (!std::isnan(lastOverrun[j])) {
          o[JOINT_NAMES[j]] = lastOverrun[j];
        }
      }
    }
    AsyncResponseStream *resp = request->beginResponseStream("application/json");
    serializeJson(doc, *resp);
    request->send(resp);
  });
}

}  // namespace Motion
