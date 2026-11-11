#include "kinematics.h"

#include <algorithm>

// See kinematics.h - a port of thinker_kinematics/arm.py (and vec.py), function for function.

namespace kin {

const char *const JOINT_NAMES[N] = {"shoulder-z", "shoulder-y", "elbow-y", "wrist-y",
                                    "wrist-z",    "claw-y",     "claw-x"};

namespace {

constexpr float PI_F = 3.14159265358979f;
constexpr float DEG = PI_F / 180.0f;

constexpr V3 X{1, 0, 0}, Y{0, 1, 0}, Z{0, 0, 1};
constexpr Q QI{1, 0, 0, 0};

constexpr float H_SHOULDER_Y = BASE_TO_SHOULDER_Z + SHOULDER_Z_TO_SHOULDER_Y;
constexpr float H_ELBOW_Y = H_SHOULDER_Y + SHOULDER_Y_TO_ELBOW_Y;
constexpr float H_WRIST_Y = H_ELBOW_Y + ELBOW_Y_TO_WRIST_Y;
constexpr float H_WRIST_Z = H_WRIST_Y + WRIST_Y_TO_WRIST_Z;
constexpr float H_CLAW_Y = H_WRIST_Z + WRIST_Z_TO_CLAW_Y;
constexpr float H_CLAW_X = H_CLAW_Y + CLAW_Y_TO_CLAW_X;
constexpr float H_TIP = H_CLAW_X + CLAW_X_TO_TIP;
constexpr V3 TIP_HOME{0, 0, H_TIP};

// Each joint at home: its axis (a growing command turns about it right-handed), a point on it, and the
// command range its module accepts. wrist-z is the side bend (+x at home); claw-x the jaws' roll about
// the claw's own axis. claw-y: the servo's whole 0-180, +-90 from in line (2026-11-06).
struct JointDef {
  V3 axis;
  V3 point;
  float lo, hi;
};
constexpr JointDef JOINTS[N] = {
    {Z, {0, 0, BASE_TO_SHOULDER_Z}, 0, 270},  //
    {Y, {0, 0, H_SHOULDER_Y}, 0, 180},        //
    {Y, {0, 0, H_ELBOW_Y}, 8, 270},           //
    {Y, {0, 0, H_WRIST_Y}, 0, 270},           //
    {X, {0, 0, H_WRIST_Z}, 0, 270},           //
    {Y, {0, 0, H_CLAW_Y}, 0, 180},            //
    {Z, {0, 0, H_CLAW_X}, 0, 270},            //
};

// The jaws' line in the claw frame with claw-x at 0: across the claw-y axis (user-checked 2026-11-06).
constexpr V3 JAW_AXIS{1, 0, 0};

// --- vectors and quaternions (vec.py) ---
inline V3 add(V3 a, V3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline V3 sub(V3 a, V3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline V3 scale(V3 a, float s) { return {a.x * s, a.y * s, a.z * s}; }
inline float dot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline V3 cross(V3 a, V3 b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
inline float norm(V3 a) { return std::sqrt(dot(a, a)); }
inline V3 unit(V3 a) { return scale(a, 1.0f / norm(a)); }
inline float angleBetween(V3 a, V3 b) { return std::atan2(norm(cross(a, b)), dot(a, b)); }
inline float wrap(float a) { return std::atan2(std::sin(a), std::cos(a)); }

inline Q qaxis(V3 axis, float angle) {
  V3 a = unit(axis);
  float s = std::sin(angle / 2);
  return {std::cos(angle / 2), a.x * s, a.y * s, a.z * s};
}
inline Q qmul(Q a, Q b) {
  return {a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z, a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
          a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x, a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}
inline V3 qrot(Q q, V3 v) {
  // q v q*, expanded: v + 2 w (u x v) + 2 u x (u x v)
  V3 u{q.x, q.y, q.z};
  V3 t = scale(cross(u, v), 2.0f);
  return add(add(v, scale(t, q.w)), cross(u, t));
}
inline float twist(Q q, V3 axis) {
  float w = q.w, x = q.x, y = q.y, z = q.z;
  if (w < 0) {
    w = -w, x = -x, y = -y, z = -z;
  }
  float p = x * axis.x + y * axis.y + z * axis.z;
  if (std::fabs(p) < 1e-12f && std::fabs(w) < 1e-12f) {
    return 0;
  }
  return 2.0f * std::atan2(p, w);
}
inline V3 apply(const Frame &T, V3 p) { return add(qrot(T.q, p), T.t); }
inline Frame compose(const Frame &A, const Frame &B) { return {qmul(A.q, B.q), add(qrot(A.q, B.t), A.t)}; }

// Solve the square system A x = b in place (Gaussian elimination, partial pivoting). n <= 7.
bool solveLinear(float A[][8], float *b, int n, float *x) {
  for (int c = 0; c < n; c++) {
    int p = c;
    for (int r = c + 1; r < n; r++) {
      if (std::fabs(A[r][c]) > std::fabs(A[p][c])) p = r;
    }
    if (std::fabs(A[p][c]) < 1e-20f) return false;
    if (p != c) {
      for (int k = 0; k < n; k++) std::swap(A[c][k], A[p][k]);
      std::swap(b[c], b[p]);
    }
    for (int r = c + 1; r < n; r++) {
      float f = A[r][c] / A[c][c];
      if (f == 0) continue;
      for (int k = c; k < n; k++) A[r][k] -= f * A[c][k];
      b[r] -= f * b[c];
    }
  }
  for (int r = n - 1; r >= 0; r--) {
    float s = b[r];
    for (int k = r + 1; k < n; k++) s -= A[r][k] * x[k];
    x[r] = s / A[r][r];
  }
  return true;
}

}  // namespace

float twistAngle(const Q &q, const V3 &axis) { return twist(q, axis); }

// Undoing the turn leaves Ry(pitch) Rx(bend), whose matrix in (fwd, side, up) gives both exactly.
void wristAngles(const Q &q, float yaw, const V3 &fwd, const V3 &side, const V3 &up, float &pitch, float &bend) {
  Q r = qmul(qaxis(up, -yaw), q);
  V3 rx = qrot(r, fwd), ry = qrot(r, side), rz = qrot(r, up);
  pitch = std::atan2(-dot(up, rx), dot(fwd, rx));
  bend = std::atan2(-dot(side, rz), dot(side, ry));
}

const Homes DEFAULT_HOMES = {{135.0f, 87.0f, 137.0f, 141.0f, 135.0f, 92.0f, 129.0f}};

V3 Pose::clawDirection() const { return qrot(frames[N - 1].q, Z); }

float Pose::lowest() const {
  float m = tip.z;
  for (int k = ELBOW_Y; k < N; k++) m = std::min(m, joints[k].z);
  return m;
}

void forward(const float angles[N], Pose &pose) {
  Frame T{QI, {0, 0, 0}};
  for (int k = 0; k < N; k++) {
    const JointDef &j = JOINTS[k];
    pose.joints[k] = apply(T, j.point);
    pose.axes[k] = qrot(T.q, j.axis);
    Q q = qaxis(j.axis, angles[k]);
    T = compose(T, Frame{q, sub(j.point, qrot(q, j.point))});
    pose.frames[k] = T;
  }
  pose.tip = apply(T, TIP_HOME);
}

void imuReadings(const Pose &pose, float out[N]) {
  out[SHOULDER_Z] = twist(pose.frames[SHOULDER_Y].q, Z);
  out[SHOULDER_Y] = twist(pose.frames[SHOULDER_Y].q, Y);
  out[ELBOW_Y] = twist(pose.frames[ELBOW_Y].q, Y);
  wristAngles(pose.frames[WRIST_Z].q, out[SHOULDER_Z], X, Y, Z, out[WRIST_Y], out[WRIST_Z]);
  // The claw runs open-loop on its joint angles - filled in by the callers that know them.
  out[CLAW_Y] = 0;
  out[CLAW_X] = 0;
}

void commands(const float angles[N], const Homes &homes, float out[N]) {
  Pose pose;
  forward(angles, pose);
  imuReadings(pose, out);
  out[CLAW_Y] = angles[CLAW_Y];
  out[CLAW_X] = angles[CLAW_X];
  for (int k = 0; k < N; k++) out[k] = homes.deg[k] + out[k] / DEG;
}

int commandOutOfRange(const float cmds[N]) {
  for (int k = 0; k < N; k++) {
    if (!(cmds[k] >= JOINTS[k].lo && cmds[k] <= JOINTS[k].hi)) return k;
  }
  return -1;
}

void jointLimits(const Homes &homes, float lo[N], float hi[N]) {
  for (int k = 0; k < N; k++) {
    lo[k] = (JOINTS[k].lo + RANGE_MARGIN_DEG - homes.deg[k]) * DEG;
    hi[k] = (JOINTS[k].hi - RANGE_MARGIN_DEG - homes.deg[k]) * DEG;
  }
}

void anglesFromReadings(const float r[N], const float *seed, float out[N]) {
  // The pitches are absolute, so the joint angles are their differences; Newton polishes.
  float t[N];
  if (seed) {
    for (int k = 0; k < 5; k++) t[k] = seed[k];
  } else {
    t[0] = r[0], t[1] = r[1], t[2] = r[2] - r[1], t[3] = r[3] - r[2], t[4] = r[4];
  }
  t[CLAW_Y] = r[CLAW_Y];
  t[CLAW_X] = r[CLAW_X];
  auto residual = [&](const float *a, float *e) {
    Pose p;
    forward(a, p);
    float got[N];
    imuReadings(p, got);
    for (int k = 0; k < 5; k++) e[k] = wrap(got[k] - r[k]);
  };
  for (int it = 0; it < 30; it++) {
    float e[5];
    residual(t, e);
    float worst = 0;
    for (float v : e) worst = std::max(worst, std::fabs(v));
    if (worst < 1e-6f) break;
    const float h = 1e-3f;
    float J[8][8] = {};
    for (int k = 0; k < 5; k++) {
      float t2[N];
      std::copy(t, t + N, t2);
      t2[k] += h;
      float e2[5];
      residual(t2, e2);
      for (int i = 0; i < 5; i++) J[i][k] = (e2[i] - e[i]) / h;
    }
    float b[5], d[5];
    for (int i = 0; i < 5; i++) b[i] = -e[i];
    if (!solveLinear(J, b, 5, d)) break;
    for (int k = 0; k < 5; k++) t[k] += d[k];
  }
  for (int k = 0; k < 5; k++) out[k] = wrap(t[k]);
  out[CLAW_Y] = t[CLAW_Y];
  out[CLAW_X] = t[CLAW_X];
}

// --- Inverse kinematics (see arm.py for the reasoning behind each constant) ---

namespace {

constexpr float DIRECTION_WEIGHT = 100.0f;  // rad -> mm
constexpr float DAMPING_MIN = 0.01f;         // mm^2; adaptive: 0.1 * |error|^2 between these
constexpr float DAMPING_MAX = 100.0f;
constexpr float MAX_STEP = 0.15f;            // rad per iteration
// What turning each joint costs: the base is dear (heavy, it swings the arm, shoulder-z backlash),
// the wrist and the claw cheap - weighted least norm, W = diag(MOTION_COST).
constexpr float MOTION_COST[N] = {4.0f, 4.0f, 2.0f, 1.0f, 1.0f, 0.5f, 0.5f};
constexpr float POSTURE_GAIN = 0.2f;  // null-space pull toward rest
constexpr float WRIST_Z_PULL = 0.3f;  // and of wrist-z toward 0
// A target asking more turning than this per mm the tip moves is near a degenerate pose: the claw
// may turn in azimuth instead (user, 2026-11-06).
constexpr float MAX_TURN_PER_MM = 1.0f;

int iterate(const V3 &target, const V3 *direction, float theta[N], const float rest[N], const float lo[N],
            const float hi[N], int maxIter, bool pitchOnly) {
  float inv[N];
  for (int k = 0; k < N; k++) inv[k] = 1.0f / MOTION_COST[k];
  int it = 0;
  for (it = 1; it <= maxIter; it++) {
    Pose pose;
    forward(theta, pose);
    const V3 cd = pose.clawDirection();
    // Rows: position (3), then the claw's direction (3, or its vertical component alone).
    float e[6];
    float cols[N][6];
    int m = 3;
    V3 ep = sub(target, pose.tip);
    e[0] = ep.x, e[1] = ep.y, e[2] = ep.z;
    for (int k = 0; k < N; k++) {
      V3 c = cross(pose.axes[k], sub(pose.tip, pose.joints[k]));
      cols[k][0] = c.x, cols[k][1] = c.y, cols[k][2] = c.z;
    }
    if (direction) {
      if (pitchOnly) {
        e[3] = DIRECTION_WEIGHT * (direction->z - cd.z);
        for (int k = 0; k < N; k++) cols[k][3] = DIRECTION_WEIGHT * cross(pose.axes[k], cd).z;
        m = 4;
      } else {
        V3 ed = scale(sub(*direction, cd), DIRECTION_WEIGHT);
        e[3] = ed.x, e[4] = ed.y, e[5] = ed.z;
        for (int k = 0; k < N; k++) {
          V3 t = scale(cross(pose.axes[k], cd), DIRECTION_WEIGHT);
          cols[k][3] = t.x, cols[k][4] = t.y, cols[k][5] = t.z;
        }
        m = 6;
      }
    }
    float e2sum = 0;
    for (int i = 0; i < m; i++) e2sum += e[i] * e[i];
    const float damping = std::min(std::max(0.1f * e2sum, DAMPING_MIN), DAMPING_MAX);
    float z0[N];
    for (int k = 0; k < N; k++) z0[k] = POSTURE_GAIN * (rest[k] - theta[k]);
    z0[WRIST_Z] -= POSTURE_GAIN * WRIST_Z_PULL * theta[WRIST_Z];

    // A joint pressed against its limit is taken out of the step, or the others would move as if it
    // had gone along and the solver would stall short of the target.
    bool locked[N] = {};
    float d[N];
    for (;;) {
      float A[8][8], y[6], y2[6], b[6], Jz[6];
      auto Jat = [&](int r, int k) { return locked[k] ? 0.0f : cols[k][r]; };
      for (int i = 0; i < m; i++) {
        for (int j = 0; j < m; j++) {
          float s = 0;
          for (int k = 0; k < N; k++) s += Jat(i, k) * inv[k] * Jat(j, k);
          A[i][j] = s + (i == j ? damping : 0.0f);
        }
      }
      float A2[8][8];
      std::copy(&A[0][0], &A[0][0] + 64, &A2[0][0]);
      std::copy(e, e + m, b);
      if (!solveLinear(A, b, m, y)) return it;
      // Task step: weighted damped least squares, W^-1 J^T (J W^-1 J^T + damping)^-1 e.
      for (int k = 0; k < N; k++) {
        float s = 0;
        for (int r = 0; r < m; r++) s += Jat(r, k) * y[r];
        d[k] = inv[k] * s;
      }
      // Posture step projected onto the task's null space in the same metric.
      float z[N];
      for (int k = 0; k < N; k++) z[k] = locked[k] ? 0.0f : z0[k];
      for (int r = 0; r < m; r++) {
        float s = 0;
        for (int k = 0; k < N; k++) s += Jat(r, k) * z[k];
        Jz[r] = s;
      }
      if (!solveLinear(A2, Jz, m, y2)) return it;
      for (int k = 0; k < N; k++) {
        float s = 0;
        for (int r = 0; r < m; r++) s += Jat(r, k) * y2[r];
        d[k] += z[k] - inv[k] * s;
      }
      bool pushing = false;
      for (int i = 0; i < N; i++) {
        if (!locked[i] && ((theta[i] <= lo[i] + 1e-6f && d[i] < 0) || (theta[i] >= hi[i] - 1e-6f && d[i] > 0))) {
          locked[i] = pushing = true;
        }
      }
      if (!pushing) break;
    }
    float big = 0;
    for (float v : d) big = std::max(big, std::fabs(v));
    const float k = big > MAX_STEP ? MAX_STEP / big : 1.0f;
    for (int i = 0; i < N; i++) theta[i] = std::min(std::max(theta[i] + d[i] * k, lo[i]), hi[i]);
    if (big < 1e-6f) break;
  }
  return it;
}

}  // namespace

V3 clawDirection(float pitchDeg, float azimuthDeg) {
  float t = pitchDeg * DEG, f = azimuthDeg * DEG;
  return {std::sin(t) * std::cos(f), std::sin(t) * std::sin(f), -std::cos(t)};
}

void solve(const V3 &target, const V3 *directionIn, const Homes &homes, const float *seedIn, bool seedOnly,
           bool azimuthFree, Solution &out, int maxIter) {
  float lo[N], hi[N];
  jointLimits(homes, lo, hi);
  V3 dir{};
  const V3 *direction = nullptr;
  if (directionIn) {
    dir = unit(*directionIn);
    direction = &dir;
  }
  const float yaw = std::hypot(target.x, target.y) > 1e-6f ? std::atan2(target.y, target.x) : 0.0f;
  float starts[3][N];
  int nStarts = 0;
  if (!(seedOnly && seedIn)) {
    // Bent "ready" postures, folding either way: off the straight-arm singularity.
    const float bends[2] = {1.0f, -1.0f};
    const float yaws[2] = {yaw, wrap(yaw + PI_F)};
    for (int s = 0; s < 2; s++) {
      const float r[N] = {yaws[s], bends[s] * 0.4f, bends[s] * 0.8f, bends[s] * 0.6f, 0, 0, 0};
      for (int k = 0; k < N; k++) starts[nStarts][k] = std::min(std::max(r[k], lo[k]), hi[k]);
      nStarts++;
    }
  }
  float seed[N];
  if (seedIn) {
    // From where the arm is: rest there, so the posture term asks for the least motion.
    for (int k = 0; k < N; k++) seed[k] = std::min(std::max(seedIn[k], lo[k]), hi[k]);
    std::copy(seed, seed + N, starts[nStarts++]);
  }
  bool haveBest = false;
  float bestA = 0, bestB = 0;
  for (int s = 0; s < nStarts; s++) {
    float theta[N];
    std::copy(starts[s], starts[s] + N, theta);
    const int it = iterate(target, direction, theta, starts[s], lo, hi, maxIter, azimuthFree);
    Pose pose;
    forward(theta, pose);
    const float pe = norm(sub(target, pose.tip));
    float de = 0;
    if (direction) {
      const V3 cd = pose.clawDirection();
      if (azimuthFree) {
        de = std::fabs(std::acos(std::min(1.0f, std::max(-1.0f, -direction->z))) -
                       std::acos(std::min(1.0f, std::max(-1.0f, -cd.z)))) / DEG;
      } else {
        de = angleBetween(*direction, cd) / DEG;
      }
    }
    // Among the reaching ones: closest to the seed (the least motion) when there is one, else to its
    // own resting posture.
    const float *ref = seedIn ? seed : starts[s];
    const float a = std::round((pe + de) * 10.0f) / 10.0f;
    float b = 0;
    for (int k = 0; k < N; k++) b += (theta[k] - ref[k]) * (theta[k] - ref[k]);
    if (!haveBest || a < bestA || (a == bestA && b < bestB)) {
      haveBest = true, bestA = a, bestB = b;
      std::copy(theta, theta + N, out.angles);
      out.pose = pose;
      out.positionError = pe;
      out.directionError = de;
      out.iterations = it;
    }
  }
  out.flags = out.pose.lowest() < FLOOR_CLEARANCE ? (uint32_t)FLAG_FLOOR : 0u;
  out.azimuthTurn = 0;
}

uint32_t jawHeading(float angles[N], float headingDeg, const Homes &homes) {
  Pose pose;
  forward(angles, pose);
  const V3 d = pose.clawDirection();
  const V3 h{std::cos(headingDeg * DEG), std::sin(headingDeg * DEG), 0};
  V3 want = sub(h, scale(d, dot(h, d)));
  if (norm(want) < 0.05f) return FLAG_ROLL_UNDEFINED;
  want = unit(want);
  const V3 have = qrot(pose.frames[N - 1].q, JAW_AXIS);
  const float turn = std::atan2(dot(d, cross(have, want)), dot(have, want));
  float lo[N], hi[N];
  jointLimits(homes, lo, hi);
  // The jaws are symmetric: a half turn either way is the same grip - the nearest one in range.
  float options[3] = {wrap(angles[CLAW_X] + turn), wrap(angles[CLAW_X] + turn + PI_F),
                      wrap(angles[CLAW_X] + turn - PI_F)};
  std::sort(options, options + 3, [&](float a, float b) {
    return std::fabs(a - angles[CLAW_X]) < std::fabs(b - angles[CLAW_X]);
  });
  for (float a : options) {
    if (a >= lo[CLAW_X] && a <= hi[CLAW_X]) {
      angles[CLAW_X] = a;
      return 0;
    }
  }
  return FLAG_ROLL_OUT_OF_RANGE;
}

void solvePose(const V3 &target, float pitchDeg, float azimuthDeg, const float *rollDeg, const Homes &homes,
               const float *seed, bool seedOnly, Solution &out) {
  const V3 d = clawDirection(pitchDeg, azimuthDeg);
  solve(target, &d, homes, seed, seedOnly, false, out);
  if (seed) {
    Pose start;
    forward(seed, start);
    const float moved = norm(sub(target, start.tip));
    float turned = 0;
    for (int k = 0; k < CLAW_X; k++) turned = std::max(turned, std::fabs(out.angles[k] - seed[k]));
    if (turned / DEG > MAX_TURN_PER_MM * std::max(moved, 1.0f) || !out.ok()) {
      Solution free;
      solve(target, &d, homes, seed, seedOnly, true, free);
      if (free.positionError < 1.0f && free.directionError < 1.0f) {
        const V3 got = free.pose.clawDirection();
        if (std::hypot(got.x, got.y) > 0.05f && std::hypot(d.x, d.y) > 0.05f) {
          free.azimuthTurn = wrap(std::atan2(got.y, got.x) - std::atan2(d.y, d.x)) / DEG;
          free.flags |= FLAG_AZIMUTH_TURNED;
        }
        out = free;
      }
    }
  }
  if (rollDeg) {
    out.flags |= jawHeading(out.angles, *rollDeg, homes);
    forward(out.angles, out.pose);
  }
}

}  // namespace kin
