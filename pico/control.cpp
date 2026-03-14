#include "control.hpp"
#include "structs.hpp"
#include <cmath>
#include "config.hpp"
//Quaternions give global attitude error, but we want local attitude error for control, 
//we need to convert the global error to local error using quaternion math.
//call quaternion math here to avoid separate quaternion math file and to keep all the control related code in one place
static inline void quatConjugate(
    float qw, float qx, float qy, float qz,
    float& ow, float& ox, float& oy, float& oz
) {
    ow = qw;
    ox = -qx;
    oy = -qy;
    oz = -qz;
}

static inline void quatMultiply(
    float aw, float ax, float ay, float az,
    float bw, float bx, float by, float bz,
    float& ow, float& ox, float& oy, float& oz
) {
    ow = aw * bw - ax * bx - ay * by - az * bz;
    ox = aw * bx + ax * bw + ay * bz - az * by;
    oy = aw * by - ax * bz + ay * bw + az * bx;
    oz = aw * bz + ax * by - ay * bx + az * bw;
}

static inline void quatNormalize(
    float& qw, float& qx, float& qy, float& qz
) {
    float n = sqrtf(qw * qw + qx * qx + qy * qy + qz * qz);
    if (n > 0.0f) {
        qw /= n;
        qx /= n;
        qy /= n;
        qz /= n;
    }
}
//all the functions quatmultiply,quatconjugate and quatnormalize that are implemented in quaternion.hpp
extern State state;
extern Throttle throttle;

const float dt = 0.1;   // 100 ms control loop

//outer loop PID
float Kp_ang = 5.0;
float Ki_ang = 1.0;
float rollInt = 0;
float pitchInt = 0;

//inner loop LQR
float K_lqr[3][2] = {
  { 1.5, -1},
  { -1.5, -1 },
  { 0.0, 2.0 }
};
const float U_MAX = 1.0;
float u_smooth[3] = { 0, 0, 0 };
const float beta = 0.2; // LQR output smoothing factor

static inline float constrain(float v, float lo, float hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

int clampDSHOT(int value) {
    if (value > 0)
        return constrain(value + 48, 0, 1000);//limits motor command values to safe range
    else if (value <= 0)
        return constrain(-value + 1049, 1001, 2000);
    return 48;
}
static bool ref_initialized = false;
static float qw_ref = 1.0f;
static float qx_ref = 0.0f;
static float qy_ref = 0.0f;
static float qz_ref = 0.0f;
void control::update() {

    /* -------- Quaternion attitude error -------- */

   /* -------- Capture startup reference -------- */
    if (!ref_initialized) {
        qw_ref = state.qw;
        qx_ref = state.qx;
        qy_ref = state.qy;
        qz_ref = state.qz;
        ref_initialized = true;
    }

    /* -------- Quaternion attitude error -------- */

    /* Conjugate of reference quaternion */
    float qw_ref_c, qx_ref_c, qy_ref_c, qz_ref_c;
    quatConjugate(
        qw_ref, qx_ref, qy_ref, qz_ref,
        qw_ref_c, qx_ref_c, qy_ref_c, qz_ref_c
    );

    /* q_err = conj(q_ref) ⊗ q_current */
    float qw_err, qx_err, qy_err, qz_err;
    quatMultiply(
        qw_ref_c, qx_ref_c, qy_ref_c, qz_ref_c,
        state.qw, state.qx, state.qy, state.qz,
        qw_err, qx_err, qy_err, qz_err
    );

    quatNormalize(qw_err, qx_err, qy_err, qz_err);
    //for smoother unit rotation
    if (qw_err < 0.0f) {
    qw_err = -qw_err;
    qx_err = -qx_err;
    qy_err = -qy_err;
    qz_err = -qz_err;
    }//q and -q represent the same rotation, the controller might suddenly command a 180 degree flip instead of a 2 degree correction
     //this might cause violent motor jumps
    /* -------- Small-angle approximation -------- */
    //local coordinates
    //float roll_err = 2.0f * qx_err;   // radians
    //float pitch_err = 2.0f * qy_err;   // radians
    //Around the current operating point, the attitude error behaves like a small rotation about X and Y
    float angle = 2.0f * acosf(qw_err);
    float s = sqrtf(1.0f - qw_err * qw_err);

      float ex = 0.0f;
      float ey = 0.0f;

          if (s > 0.001f) {
           ex = qx_err / s;
           ey = qy_err / s;
        }

          float roll_err  = angle * ex;
          float pitch_err = angle * ey;
   /* -------- Outer PID -------- */

    rollInt += roll_err * dt;
    pitchInt += pitch_err * dt;

    rollInt = constrain(rollInt, -0.02f, 0.02f);
    pitchInt = constrain(pitchInt, -0.02f, 0.02f);

    float wx_ref = Kp_ang * roll_err + Ki_ang * rollInt;
    float wy_ref = Kp_ang * pitch_err + Ki_ang * pitchInt;

    /* -------- Inner LQR -------- */

    float omega_err[2] = { state.wx - wx_ref,state.wy - wy_ref };

    for (int i = 0; i < 2; i++) {
        if (fabsf(omega_err[i]) < 0.01f)
            omega_err[i] = 0.0f;
    }

    float u[3];
    u[0] = -(K_lqr[0][0] * omega_err[0] + K_lqr[0][1] * omega_err[1]);
    u[1] = -(K_lqr[1][0] * omega_err[0] + K_lqr[1][1] * omega_err[1]);
    u[2] = -(K_lqr[2][0] * omega_err[0] + K_lqr[2][1] * omega_err[1]);

    float umax = fmaxf(fmaxf(fabsf(u[0]), fabsf(u[1])), fabsf(u[2]));
    if (umax > U_MAX) {
        float s = U_MAX / umax;
        for (int i = 0; i < 3; i++) u[i] *= s;
    }

    for (int i = 0; i < 3; i++) {
        u_smooth[i] = beta * u[i] + (1.0f - beta) * u_smooth[i];
    }
// Deadband on final motor output — prevents dithering across the
// forward/reverse boundary when the control output is near zero.
// Values within ±MOTOR_DEADBAND are treated as stopped (DSHOT 1049 = zero throttle 3D).
const float MOTOR_DEADBAND = 0.05f;  // tune this: start at 0.05, increase if dithering persists

int vl, vr, vb;

if (fabsf(u_smooth[0]) < MOTOR_DEADBAND) vl = 1049;
else vl = clampDSHOT(u_smooth[0] * 150.0f);

if (fabsf(u_smooth[1]) < MOTOR_DEADBAND) vr = 1049;
else vr = clampDSHOT(u_smooth[1] * 150.0f);

if (fabsf(u_smooth[2]) < MOTOR_DEADBAND) vb = 1049;
else vb = clampDSHOT(u_smooth[2] * 150.0f);

throttle.VL = (uint16_t)vl;
throttle.VR = (uint16_t)vr;
throttle.VB = (uint16_t)vb;

   // throttle.VL = clampDSHOT(u_smooth[0] * 150);
   // throttle.VR = clampDSHOT(u_smooth[1] * 150);
    //throttle.VB = clampDSHOT(u_smooth[2] * 150);
}