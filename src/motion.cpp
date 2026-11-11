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

float slot(uint64_t status, int shift) {
  const uint16_t raw = (status >> shift) & 0xFFFF;
  return raw == 0xFFFF ? NAN : raw / 100.0f;
}

Q toQ(const Quaternion &q) { return {q.real, q.i, q.j, q.k}; }

float minJerk(float t) { return t * t * t * (10.0f + t * (-15.0f + 6.0f * t)); }

int16_t angle10(float deg) { return std::isnan(deg) ? (int16_t)PARAMETER_IS_NAN : (int16_t)lroundf(deg * 10.0f); }

// One waypoint to every module: the same set frames as /set (ArmDataFrame, angles x10), with a stream
// timeMS.
void sendWaypoint(const float c[N]) {
  ArmDataFrame f{};
  f.values = {angle10(c[SHOULDER_Y]), angle10(c[SHOULDER_Z]), (int16_t)STEP_TIME_MS, (int16_t)PARAMETER_IS_NAN};
  Arm::twai.sendData(CAN_SHOULDER_SET_YZ_DEGREE, f.bytes);
  f.values = {angle10(c[ELBOW_Y]), (int16_t)STEP_TIME_MS, (int16_t)PARAMETER_IS_NAN, (int16_t)PARAMETER_IS_NAN};
  Arm::twai.sendData(CAN_ELBOW_SET_Y_DEGREE, f.bytes);
  f.values = {angle10(c[WRIST_Y]), angle10(c[WRIST_Z]), (int16_t)STEP_TIME_MS, (int16_t)PARAMETER_IS_NAN};
  Arm::twai.sendData(CAN_WRIST_SET_YZ_DEGREE, f.bytes);
  f.values = {angle10(c[CLAW_Y]), angle10(c[CLAW_X]), (int16_t)PARAMETER_IS_NAN, (int16_t)STEP_TIME_MS};
  Arm::twai.sendData(CAN_CLAW_SET_XYG_DEGREE, f.bytes);
  if (!std::isnan(c[CLAW_X])) {
    lastClawX = c[CLAW_X];
  }
}

void streamTask(void *) {
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&last, pdMS_TO_TICKS(INTERVAL_MS));
    float c[N];
    bool send = false;
    portENTER_CRITICAL(&planLock);
    if (planActive) {
      if (!PowerManagement::getEnginesStatus() || planNext >= planCount) {
        planActive = false;
      } else {
        std::copy(plan[planNext], plan[planNext] + N, c);
        planNext = planNext + 1;
        send = true;
      }
    }
    portEXIT_CRITICAL(&planLock);
    if (send) {
      sendWaypoint(c);
    }
  }
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
    const float s = minJerk(k / (float)n);
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
  portEXIT_CRITICAL(&planLock);
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

  for (const char *path : {"/move", "/stop", "/pose"}) {
    Server.on(path, HTTP_OPTIONS, [](AsyncWebServerRequest *request) { request->send(200); });
  }
  // {"x", "y", "z", "pitch", "azimuth" (default: toward the point), "roll" (optional), "timeMS",
  //  "path": "line" | "joint"}, or instead of the claw target {"home": true} (every joint but claw-x
  //  to its calibration pose - the arm vertical) or {"angles": {"elbow-y": deg, ...}} (joint angles
  //  from the calibration pose, the joints not named stay) - both a joint path.
  auto *move = new AsyncCallbackJsonWebHandler("/move", [](AsyncWebServerRequest *request, JsonVariant json) {
    DynamicJsonDocument doc(512);
    JsonObject reply = doc.to<JsonObject>();
    JsonObject in = json.as<JsonObject>();
    Request q{};
    float timeMS = 3000;
    const bool home = !in.isNull() && in["home"].as<bool>();
    const bool angles = !in.isNull() && in["angles"].is<JsonObject>();
    if (home || angles) {
      q.jointTarget = true;
      for (int j = 0; j < N; j++) {
        q.jointDeg[j] = home && j != CLAW_X ? 0.0f : NAN;
      }
      if (angles) {
        JsonObject a = in["angles"].as<JsonObject>();
        for (int j = 0; j < N; j++) {
          getFloat(a, JOINT_NAMES[j], q.jointDeg[j]);
        }
      }
      getFloat(in, "timeMS", timeMS);
      q.timeMS = (uint32_t)std::max(200.0f, timeMS);
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

  Server.on("/stop", HTTP_POST, [](AsyncWebServerRequest *request) {
    stop();
    request->send(200, "application/json", "{\"ok\":true}");
  });

  Server.on("/pose", HTTP_POST, [](AsyncWebServerRequest *request) {
    DynamicJsonDocument doc(768);
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
    }
    AsyncResponseStream *resp = request->beginResponseStream("application/json");
    serializeJson(doc, *resp);
    request->send(resp);
  });
}

}  // namespace Motion
