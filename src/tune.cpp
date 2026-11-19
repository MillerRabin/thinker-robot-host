#include "tune.h"

#include <Arduino.h>
#include <AsyncJson.h>
#include <Preferences.h>
#include <cmath>
#include <cstring>

#include "arm.h"
#include "config.h"
#include "server.h"

namespace Tune {
namespace {

// The modules' common/tuning/tuning.h numbering.
struct Name {
  const char *name;
  uint8_t id;
};
const Name JOINTS[] = {{"shoulder-y", 1}, {"shoulder-z", 2}, {"elbow-y", 3}, {"wrist-y", 4}, {"wrist-z", 5}};
constexpr int JOINT_IDS = 6;   // 1..5
constexpr int PARAM_IDS = 32;  // 1..31: room for parameters to come without changing the saved size
constexpr int PARAM_IDS_V1 = 18;  // the first saved profiles (2026-11-17, parameters 1..17)
Module moduleOf(int joint) { return joint <= 2 ? SHOULDER : joint == 3 ? ELBOW : WRIST; }
const Name PARAMS[] = {
    {"dampingGain", 1},      {"dampingMax", 2},       {"dampingDeadband", 3}, {"dampingRateTau", 4},
    {"ditherDeg", 5},        {"ditherHalfPeriodMs", 6}, {"streamGain", 7},    {"streamErrorTau", 8},
    {"settleTolerance", 9},  {"settleSpeed", 10},     {"calmSpan", 11},       {"calmBlockMs", 12},
    {"calmWaitMaxMs", 13},   {"holdWakeDeg", 14},     {"holdWakeMs", 15},     {"holdWakeNowDeg", 16},
    {"parentWait", 17},      {"parentFollow", 18},    {"entrySec", 19},       {"correctionMax", 20},
    {"resetIMU", 21},        {"fusionGate", 22},
};
// Parameters that are actions, not settings: never kept in the profile (resent at a module's boot).
bool isAction(int param) { return param == 21; }
// A module answers every request on its own *_TUNED id, but its send buffer keeps only the latest frame
// per id - so one request at a time, each waiting for its answer.
constexpr uint32_t REPLY_TIMEOUT_MS = 100;
// A module silent this long and heard again gets its profile again (its boot holds its statuses ~3 s).
constexpr uint32_t SILENCE_MS = 1000;

// The profile: the value each joint's parameter is to have, NaN = the module's own default. Guarded by
// askLock with the requests (the HTTP handler and the resend task both ask).
float profile[JOINT_IDS][PARAM_IDS];
SemaphoreHandle_t askLock = nullptr;
volatile uint32_t lastHeard[MODULES] = {};
bool applied[MODULES] = {};
Preferences prefs;

portMUX_TYPE replyLock = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t replySeq = 0;
uint8_t replyJoint = 0, replyParam = 0;
float replyValue = NAN;

int idOf(const Name *names, size_t n, const char *name) {
  for (size_t k = 0; k < n; k++) {
    if (strcmp(names[k].name, name) == 0) return names[k].id;
  }
  return -1;
}

// Sends one request and waits for its answer. False on no answer; value NaN = the joint has no such
// parameter.
bool ask(uint8_t joint, uint8_t param, bool set, float value, float &out) {
  uint8_t data[8] = {joint, param, (uint8_t)(set ? 1 : 0), 0};
  memcpy(data + 4, &value, sizeof(float));
  const uint32_t seq0 = replySeq;
  if (!Arm::twai.sendData(CAN_TUNE, data)) return false;
  const uint32_t t0 = millis();
  while (millis() - t0 < REPLY_TIMEOUT_MS) {
    delay(2);
    portENTER_CRITICAL(&replyLock);
    const bool got = replySeq != seq0 && replyJoint == joint && replyParam == param;
    const float v = replyValue;
    portEXIT_CRITICAL(&replyLock);
    if (got) {
      out = v;
      return true;
    }
  }
  return false;
}

void loadProfile() {
  for (auto &row : profile) {
    for (float &v : row) v = NAN;
  }
  if (prefs.begin("tune", true)) {
    const size_t len = prefs.getBytesLength("profile");
    if (len == sizeof(profile)) {
      prefs.getBytes("profile", profile, sizeof(profile));
    } else if (len == sizeof(float) * JOINT_IDS * PARAM_IDS_V1) {
      float old[JOINT_IDS][PARAM_IDS_V1];
      prefs.getBytes("profile", old, sizeof(old));
      for (int j = 0; j < JOINT_IDS; j++) {
        for (int q = 0; q < PARAM_IDS_V1; q++) profile[j][q] = old[j][q];
      }
    }
    prefs.end();
  }
}

// Flash writes stall both cores for a moment - only on a request, never by itself.
bool saveProfile(bool clear) {
  if (!prefs.begin("tune", false)) return false;
  const bool ok = clear ? prefs.remove("profile") || !prefs.isKey("profile")
                        : prefs.putBytes("profile", profile, sizeof(profile)) == sizeof(profile);
  prefs.end();
  return ok;
}

void putProfile(JsonObject reply) {
  JsonObject p = reply.createNestedObject("profile");
  for (const Name &j : JOINTS) {
    JsonObject o;
    for (const Name &q : PARAMS) {
      const float v = profile[j.id][q.id];
      if (std::isnan(v)) continue;
      if (o.isNull()) o = p.createNestedObject(j.name);
      o[q.name] = v;
    }
  }
}

// Sends a module its part of the profile; true when every value was answered.
bool applyProfile(Module m) {
  bool ok = true;
  for (const Name &j : JOINTS) {
    if (moduleOf(j.id) != m) continue;
    for (const Name &q : PARAMS) {
      const float v = profile[j.id][q.id];
      if (std::isnan(v)) continue;
      float out;
      ok = ask(j.id, q.id, true, v, out) && ok;
    }
  }
  return ok;
}

void resendTask(void *) {
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(200));
    for (int m = 0; m < MODULES; m++) {
      // The receive task stamps a module ~every 5 ms and can do it between a millis() and the read of
      // its stamp, a stamp can also be 1 ms ahead (the | 1), and unsigned the difference then wrapped to
      // ~49 days: the profile went out again every few seconds, in moves too (2026-11-19).
      const uint32_t heardAt = lastHeard[m];
      const int32_t since = (int32_t)(millis() - heardAt);
      if (heardAt == 0 || since > (int32_t)SILENCE_MS) {
        applied[m] = false;
        continue;
      }
      if (!applied[m]) {
        xSemaphoreTake(askLock, portMAX_DELAY);
        applied[m] = applyProfile((Module)m);
        xSemaphoreGive(askLock);
      }
    }
  }
}

}  // namespace

void heard(Module m) { lastHeard[m] = millis() | 1; }

void onReply(const uint8_t data[8]) {
  float v;
  memcpy(&v, data + 4, sizeof(float));
  portENTER_CRITICAL(&replyLock);
  replyJoint = data[0];
  replyParam = data[1];
  replyValue = (data[2] & 1) ? v : NAN;
  replySeq = replySeq + 1;
  portEXIT_CRITICAL(&replyLock);
}

void begin() {
  loadProfile();
  askLock = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(resendTask, "Tune::resend", 3072, nullptr, 1, nullptr, 1);
  Server.on("/tune", HTTP_OPTIONS, [](AsyncWebServerRequest *request) { request->send(200); });
  auto *tune = new AsyncCallbackJsonWebHandler("/tune", [](AsyncWebServerRequest *request, JsonVariant json) {
    DynamicJsonDocument doc(3072);
    JsonObject reply = doc.to<JsonObject>();
    JsonObject in = json.as<JsonObject>();
    const char *jointName = in.isNull() ? nullptr : in["joint"].as<const char *>();
    const int joint = jointName == nullptr ? -1 : idOf(JOINTS, sizeof(JOINTS) / sizeof(JOINTS[0]), jointName);
    const bool save = !in.isNull() && in["save"].as<bool>();
    const bool clear = !in.isNull() && in["clear"].as<bool>();
    xSemaphoreTake(askLock, portMAX_DELAY);
    if (clear) {
      for (auto &row : profile) {
        for (float &v : row) v = NAN;
      }
      reply["cleared"] = saveProfile(true);
    }
    if (joint < 0) {
      if (!save && !clear) {
        reply["ok"] = false;
        reply["error"] = "need \"joint\": shoulder-y, shoulder-z, elbow-y, wrist-y or wrist-z - or save, clear";
      } else {
        reply["ok"] = true;
      }
    } else {
      bool ok = true;
      JsonArray errors = reply.createNestedArray("errors");
      if (in["set"].is<JsonObject>()) {
        for (JsonPair kv : in["set"].as<JsonObject>()) {
          const int param = idOf(PARAMS, sizeof(PARAMS) / sizeof(PARAMS[0]), kv.key().c_str());
          float out = NAN;
          if (param < 0) {
            errors.add(String("no parameter ") + kv.key().c_str());
            ok = false;
          } else if (!ask((uint8_t)joint, (uint8_t)param, true, kv.value().as<float>(), out)) {
            errors.add(String("no answer setting ") + kv.key().c_str());
            ok = false;
          } else if (!isAction(param)) {
            profile[joint][param] = out;  // as the module clamped it
          }
        }
      }
      JsonObject values = reply.createNestedObject("values");
      for (const Name &p : PARAMS) {
        float out = NAN;
        if (isAction(p.id)) continue;
        if (ask((uint8_t)joint, p.id, false, NAN, out)) {
          if (!std::isnan(out)) values[p.name] = out;
        } else {
          errors.add(String("no answer reading ") + p.name);
          ok = false;
        }
      }
      reply["ok"] = ok;
      reply["joint"] = jointName;
    }
    if (save) {
      reply["saved"] = saveProfile(false);
    }
    putProfile(reply);
    xSemaphoreGive(askLock);
    AsyncResponseStream *resp = request->beginResponseStream("application/json");
    serializeJson(doc, *resp);
    request->send(resp);
  });
  Server.addHandler(tune);
}

}  // namespace Tune
