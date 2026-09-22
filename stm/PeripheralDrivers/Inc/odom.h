#ifndef __ODOM_H
#define __ODOM_H

#include "stm32f4xx_hal.h"

/* ---------------------------------------------------------------------------
 * Odometry and heading hold for the WHEELTEC C30D-V2.
 *
 * PHASE 3 CHANGE: heading is now held by the STEERING SERVO, not by trimming
 * one rear wheel against the other.
 *
 * This chassis is Ackermann. The front wheels are locked at whatever angle
 * the servo holds, so a wheel-speed difference cannot change the heading -
 * it only yaws the body against the front tyres and scrubs them sideways.
 * The old trim approach worked, but only by dragging the tyres, and it
 * needed a permanent offset to hold a straight line.
 *
 * The per-wheel speed PID stays. Its job is now purely to make Motor A and
 * Motor B run at the SAME speed. Steering is the servo's job alone.
 *
 * Odom_Update() runs from the TIM6 interrupt, after Encoders_Update() and
 * before PID_Update().
 * ------------------------------------------------------------------------- */

/* Geometry. WHEEL_BASE_MM here is the TRACK width (distance between the two
 * rear wheel centres) and is used only to infer heading from the encoder
 * difference. On an Ackermann chassis that inference is weak - the rear
 * wheels barely differ on a gentle curve - so treat encoder heading as a
 * rough estimate until the IMU is available. */
/* Effective rolling diameter of wheel A, mm. MEASURED - do not "correct"
 * this back to a catalogue figure.
 *
 * The wheel is sold as 2.4 inch (60.96 mm). That is the RIM. The tread
 * calipers at 65-66 mm, and four road tests agreed on 65.87 to 66.06 with
 * the wheels averaged.
 *
 * There was briefly a case for 65.6, to compensate a per-wheel count change
 * on encoder B. That change was reverted - both wheels are back at 1560, see
 * below - so the compensation went with it. 65.9 is the measured figure and
 * is what A.3 was proven at, 0.0 to 0.3% error at 1000 and 1200 mm. */
#define WHEEL_DIAMETER_MM   65.9f
#define WHEEL_BASE_MM       127.0f

/* From the ten-revolution calibration. */
/* Counts per wheel revolution, PER WHEEL. B is NOT 1560, and that is
 * deliberate.
 *
 * Measured over four driven runs, wheel B counted 99.0, 99.2, 99.5 and 99.2
 * percent of wheel A. Steady, not scattering - the signature of a fixed
 * scale error rather than mechanical slip. (Slip was a real problem earlier
 * and was cured by tightening the hub grub screw; it showed up as a 32%
 * spread WITHIN a single run.)
 *
 * Why it matters far more than 0.8% sounds: odometry infers heading from
 * (dA - dB) / track. A steady 0.8% shortfall on B is read as a continuous
 * LEFTWARD yaw - about 3.5 degrees accumulated over a metre - and the
 * heading loop obediently steers RIGHT to cancel a rotation that is not
 * happening. That was the visible rightward drift.
 *
 * The control experiment: one run went visibly straight and STILL read
 * 99.2%. A genuinely straight run must give 100%, so the mismatch causes
 * the curve rather than resulting from it.
 *
 * 1560 x 0.9922 = 1548. Residual scatter is +-0.25%, which leaves about
 * 1.1 degrees of phantom yaw instead of 3.5.
 */
#define ODOM_COUNTS_PER_REV_A  1560.0f
/* Back to 1560, deliberately.
 *
 * 1548 was a correction for wheel B under-reporting by 0.8%, which odometry
 * was reading as a phantom leftward yaw and the servo was counter-steering
 * against. With heading now coming from the gyro, the encoder difference no
 * longer steers anything, so that correction has nothing left to fix - and
 * it WOULD disturb the distance calibration, which was measured at 0.0%
 * error with both wheels at 1560 and the diameter at 65.9.
 *
 * Change one thing at a time. The proven distance setup stays. */
#define ODOM_COUNTS_PER_REV_B  1560.0f

/* Kept for anything still referencing the single-wheel name. */
#define ODOM_COUNTS_PER_REV ODOM_COUNTS_PER_REV_A

/* 65.9 * pi / 1560 -- about 0.133 mm of travel per count. */
#define MM_PER_COUNT_A ((WHEEL_DIAMETER_MM * 3.14159265f) / ODOM_COUNTS_PER_REV_A)
#define MM_PER_COUNT_B ((WHEEL_DIAMETER_MM * 3.14159265f) / ODOM_COUNTS_PER_REV_B)
#define MM_PER_COUNT   MM_PER_COUNT_A

/* ---------------------------------------------------------------------------
 * Which wheel measures how far the robot went.
 *
 *   ODOM_DIST_AVERAGE  mean of both. Correct when both encoders are trusted:
 *                      the mean is the axle centre, which is what a distance
 *                      command means. Use this once B is healthy.
 *
 *   ODOM_DIST_A_ONLY   wheel A alone. Use when B cannot be trusted. On a
 *                      straight line A and the axle centre travel the same
 *                      distance, so nothing is lost for A.3 - and one good
 *                      encoder beats the average of a good one and a bad one,
 *                      because averaging does not cancel B's error, it halves
 *                      it and then hides it.
 *
 *   ODOM_DIST_B_ONLY   for swapping the test around.
 *
 * IN A TURN this matters: A is the left wheel, so on a right-hand arc it runs
 * wide and A_ONLY over-reports the centre's travel by (R + track/2)/R. The
 * arc primitives are not usable on A_ONLY without that correction.
 * ------------------------------------------------------------------------- */
#define ODOM_DIST_AVERAGE   0
#define ODOM_DIST_A_ONLY    1
#define ODOM_DIST_B_ONLY    2

#define ODOM_DIST_SOURCE    ODOM_DIST_AVERAGE

/* ---------------------------------------------------------------------------
 * Where heading comes from.
 *
 *   ODOM_HEADING_ENCODER  from (dA - dB) / track. Works, but weak on an
 *                         Ackermann chassis: the two rear wheels barely
 *                         differ on a gentle curve, so it is a small angle
 *                         inferred from the difference of two large numbers.
 *                         Any per-wheel scale mismatch shows up as a
 *                         constant phantom yaw. A measured 0.8% mismatch
 *                         produced 3.5 degrees of phantom yaw per metre.
 *
 *   ODOM_HEADING_IMU      integrated gyro Z. Measures yaw directly, so wheel
 *                         scale, slip and tyre scrub cannot corrupt it.
 *                         Measured on this robot: 90 degrees by hand read
 *                         90.5, and drift after zeroing is a fraction of a
 *                         degree - against 3.5 degrees of encoder phantom
 *                         yaw over the same run.
 *
 * The IMU setting falls back to the encoder automatically if IMU_IsReady()
 * is false, so a failed sensor degrades to the old behaviour instead of
 * leaving the robot with no heading at all.
 * ------------------------------------------------------------------------- */
#define ODOM_HEADING_ENCODER  0
#define ODOM_HEADING_IMU      1

#define ODOM_HEADING_SOURCE   ODOM_HEADING_IMU

/* ---------------------------------------------------------------------------
 * Heading hold tuning
 * ------------------------------------------------------------------------- */

/* Microseconds of servo deflection per degree of heading error.
 *
 * You measured roughly 19 us per degree of wheel steer. A gain of 8 means a
 * 5 degree heading error commands about 2 degrees of corrective steer, which
 * is a gentle correction. Raise until it corrects briskly; back off if it
 * weaves. */
#define HEADING_KP_US       8.0f

/* Ceiling on the correction, in microseconds either side of centre. Keeps a
 * large error from slamming the steering to full lock mid-run. 150 us is
 * about 8 degrees of steer. */
#define HEADING_MAX_US      150.0f

/* Below this the correction is suppressed entirely. Without it the servo
 * hunts around centre and buzzes. */
#define HEADING_DEADBAND_DEG  0.3f

/* Backlash compensation, microseconds.
 *
 * You measured roughly 50 us of slack between commanding a direction change
 * and the wheels responding. A plain proportional term produces corrections
 * smaller than that whenever the error is under ~2.5 degrees, so nothing
 * happens, the error grows, and then it over-corrects - the robot weaves.
 *
 * Once the correction is non-zero we add half the measured slack in the same
 * direction, which pushes through it. Set to 0 to disable and see the
 * difference. */
#define SERVO_BACKLASH_US   25.0f

/* ---------------------------------------------------------------------------
 * LEARNED CENTRE TRIM
 *
 * SERVO_CENTER_US is where the steering is BELIEVED to be straight. If it is
 * off, P alone can never fix it: P only reacts to an error that already
 * exists, so the robot drifts until the error is big enough to correct, gets
 * pushed back, and drifts again. The deadband hides small offsets entirely.
 *
 * THIS IS LEARNED BETWEEN RUNS, NOT DURING THEM.
 *
 * The obvious answer is an integral term in the heading loop. I tried that
 * and it made both straightness AND distance worse, for one reason: this
 * linkage has about 50 us of backlash. An integral winds up while the slack
 * means nothing mechanical is happening, the linkage then engages all at
 * once, it over-corrects, and the robot weaves. A weaving path is also longer
 * than a straight one, so the odometry over-reads against a tape measure -
 * which is why a steering change appeared to break distance accuracy.
 *
 * So instead: accumulate the mean heading error across a whole run, and adjust
 * the trim ONCE when the run finishes. The trim is a fixed servo offset while
 * driving, so the inner loop is exactly what it was before and cannot be
 * destabilised. Same shape as the arc deceleration learning.
 *
 * The trim is in SERVO space, so it is NOT flipped when reversing, while the
 * proportional term is. A mechanical offset is the same offset either way. */
/* ---------------------------------------------------------------------------
 * CROSS-TRACK CORRECTION
 *
 * Heading hold controls DIRECTION, not POSITION. If the robot yaws a degree
 * and the loop corrects it back to zero, the robot is pointing straight again
 * - but it is now travelling on a line PARALLEL to the original one, offset
 * sideways. Every transient yaw leaves a permanent displacement and a
 * heading-only loop never brings it back. That is why the robot finishes
 * straight but shifted.
 *
 * The fix is to steer toward the LINE rather than toward a heading. odom
 * already tracks y_mm, so the loop aims at a heading leaning back to zero:
 *
 *      aim = -CROSS_KP_DEG_PER_MM * y_mm      (clamped)
 *
 * 10 mm off asks for 0.8 degrees of lean, 50 mm asks for 4.
 *
 * KEEP THE GAIN LOW. Steering angle integrates into heading and heading
 * integrates into position, so this closes a loop around TWO integrators -
 * inherently more prone to oscillation than heading hold alone. Too much gain
 * and the robot weaves across the line instead of settling on it, and a
 * weaving path is longer than a straight one so it costs distance accuracy
 * as well as straightness.
 *
 * IF IT WEAVES: halve CROSS_KP_DEG_PER_MM before changing anything else, and
 * re-check the tape measure as well as the line. Set CROSS_TRACK_ENABLE to 0
 * to remove it entirely.
 *
 * y_mm is dead reckoning, integrated from heading and distance. Over a metre
 * with this calibration that is reliable to a few mm, but it is not an
 * absolute position - it can only return the robot to where it BELIEVES the
 * line was. */
#define CROSS_TRACK_ENABLE      1
#define CROSS_KP_DEG_PER_MM     0.08f
#define CROSS_MAX_DEG           8.0f

#define HEADING_TRIM_GAIN   4.0f    /* us of trim per degree of mean error */
#define HEADING_TRIM_MAX_US 80.0f

/* ---------------------------------------------------------------------------
 * HOW THE TRIM IS LEARNED
 *
 * 1 - from the mean CORRECTION the steering loop put out, in servo us.
 * 0 - from the mean heading ERROR, the original rule, via HEADING_TRIM_GAIN.
 *
 * Output space is a measurement; error space is a search. When the robot is
 * being held straight, the average of what the servo had to do IS the bias
 * the centre is missing, already in the right units - so the update is the
 * answer rather than a fraction of it, and two runs get there where the old
 * rule took about ten.
 *
 * The bigger difference is that output space has somewhere to STOP. The
 * heading loop ignores any error inside HEADING_DEADBAND_DEG, but the old
 * rule integrated those errors anyway: a robot sitting steadily at 0.2
 * degrees moved the trim 0.8 us every run and never converged, because it was
 * chasing something the controller had already ruled close enough. A
 * deadband tick contributes a correction of exactly zero, and zero is the
 * right evidence - no steering needed means no trim needed.
 *
 * Both estimators are accumulated on every run, so flipping this is a
 * one-line rebuild and an honest A/B on the same robot. */
#define TRIM_LEARN_FROM_OUTPUT  1

/* Fraction of the measured bias folded in per run. 1.0 is deadbeat and lands
 * in a single run; it also hands the whole trim to one bumped or wheel-slipped
 * run. 0.7 costs about one extra run and does not. */
#define TRIM_LEARN_GAIN      0.7f

/* Ceiling on one run's step, us. Same argument as the gain, for the single
 * wild run that the gain alone would still let through. Generous enough that
 * a genuine cold start still converges in two runs. */
#define TRIM_MAX_STEP_US     25.0f

/* Ticks of a straight discarded before the accumulators start, at 100 Hz.
 * The servo slams from centre at launch and the first stretch is transient,
 * not evidence. Throwing it away is a better estimate from the SAME run -
 * cheaper than driving further. */
#define TRIM_WARMUP_TICKS    50U    /* 0.5 s */

/* Samples required AFTER the warm-up window before the trim is touched at
 * all. Aborted and very short moves teach nothing. */
#define TRIM_MIN_SAMPLES     50U    /* 0.5 s */

/* TRIM_WARMUP_TICKS + TRIM_MIN_SAMPLES is the real minimum: 100 ticks, one
 * second of HOLDING, which needs a straight of roughly 800 mm once the accel
 * and brake ramps are paid for. It used to be half that, which is where
 * PROTOCOL.md §7's old "a straight >= 500 mm" came from - that figure moved
 * with this one.
 *
 * A straight that falls short does not fail: Odom_LearnTrim() returns having
 * changed nothing, and a trim that did not move looks exactly like a trim
 * that has converged. Odom_TrimWasUpdated() below is how you tell. */
#define ODOM_DT_S           0.01f

/* Sign convention: which way a positive heading error should steer.
 *
 * CHANGED FROM +1 TO -1 WHEN HEADING MOVED TO THE GYRO. The two heading
 * sources count in OPPOSITE directions, and this is the constant that
 * absorbs it:
 *
 *   Encoder:  d_theta = (dA - dB) / track, and Motor A is the LEFT rear.
 *             On a left (counter-clockwise) turn the left wheel is on the
 *             inside, so dA < dB and d_theta comes out NEGATIVE.
 *             The encoder path is therefore CLOCKWISE-positive, whatever
 *             the "CCW positive" comment on Odom_Pose_t says.
 *
 *   Gyro:     IMU_Z_SIGN +1 was verified on the robot by rotating it
 *             anticlockwise and watching the heading INCREASE.
 *             Counter-clockwise-positive.
 *
 * So switching source inverted the sign of every heading error, and a loop
 * with the wrong sign steers INTO the error rather than out of it - which
 * looks like the robot suddenly steering hard one way.
 *
 * If you ever switch ODOM_HEADING_SOURCE back to the encoder, flip this
 * back to +1 at the same time.
 *
 * VERIFY, do not trust: start a straight run, let it settle, nudge the nose
 * a few degrees by hand. The front wheels must turn to steer BACK toward the
 * original line. If they turn the same way you nudged, flip this. */
#define HEADING_SIGN        (-1)

typedef struct
{
    float x_mm;         /* forward from the start pose   */
    float y_mm;         /* left of the start pose        */
    float heading_deg;  /* sign follows ODOM_HEADING_SOURCE; wraps to +-180 */
    float distance_mm;  /* total path length travelled   */
} Odom_Pose_t;

/* Zeroes the pose and syncs to the current encoder counts. Call at startup
 * after Encoders_Init() and Servos_Init(). */
void Odom_Init(void);

/* Clears the pose to the origin without disturbing the encoders. */
void Odom_Reset(void);

/* One integration step. Call from the TIM6 ISR, after Encoders_Update()
 * and before PID_Update(). */
void Odom_Update(void);

/* Current pose. Safe to call from the main loop. */
void Odom_GetPose(Odom_Pose_t *out);

float Odom_GetHeading(void);

/* Heading since the last Odom_Reset(), degrees, NOT wrapped. Positive follows
 * whichever source ODOM_HEADING_SOURCE selects. Use this for turns - a 270
 * degree arc must read 270, and the wrapped value would read -90. */
float Odom_GetHeadingTotal(void);

/* Yaw rate from the last tick, degrees per second, signed. Comes from
 * whichever source ODOM_HEADING_SOURCE selects, so it works with the encoder
 * fallback too. Used to predict how far the robot will coast after braking. */
float Odom_GetRateDps(void);
float Odom_GetDistance(void);

/* ------------------------------------------------------------------ */
/* Heading hold                                                        */
/* ------------------------------------------------------------------ */

/* Drive at the given speed holding the current heading. Negative rpm
 * reverses; the correction sign is flipped automatically. */
void Odom_DriveStraight(int16_t rpm);

/* Drive at the given speed holding a specific heading in degrees. */
void Odom_DriveHeading(int16_t rpm, float heading_deg);

/* Change speed WITHOUT disturbing the active drive mode.
 *
 * Odom_DriveHeading() latches a new heading target, zeroes the accumulated
 * error and recentres the servo. That is right when a move starts and wrong
 * in the middle of one: calling it to slow down for the approach throws away
 * the correction the loop had settled on and snaps the steering to centre,
 * which shows up as a visible twitch in the last 150 mm of the run - exactly
 * the part the supervisor is watching for A.3.
 *
 * This changes only the setpoint. For an arc the inner/outer split is
 * recomputed at the new speed and the servo is left where it is. */
void Odom_SetSpeed(int16_t rpm);

/* ------------------------------------------------------------------ */
/* Arc driving                                                         */
/* ------------------------------------------------------------------ */

/* Hold a fixed steering deflection and drive an arc.
 *
 * Heading hold is OFF for the duration - on an Ackermann chassis the heading
 * is set by the steering angle, and a heading loop fighting a deliberate turn
 * just cancels it.
 *
 * The two rear wheels are commanded at DIFFERENT speeds. There is no
 * mechanical differential here; both rear wheels are driven independently, so
 * if they are told to run at the same RPM through a turn the inner one is
 * dragged and both tyres scrub. Speeds are split from the geometry:
 *
 *     v_outer / v_inner = (R + track/2) / (R - track/2)
 *
 * radius_mm is measured at the CENTRE of the rear axle, which is also what
 * Odom_GetDistance() reports, so arc length = radius * angle.
 *
 * right: 1 to curve right (Motor B inner), 0 to curve left (Motor A inner).
 * Negative rpm reverses. */
void Odom_DriveArc(int16_t rpm, uint16_t servo_us, float radius_mm,
                   uint8_t right, float diff_boost);

/* ---------------------------------------------------------------------------
 * Differential assist - deliberate over-differential to tighten a turn.
 *
 * 1.0 splits the two rear wheel speeds to match the steering geometry
 * exactly, so neither tyre scrubs. That is the "correct" value and it leaves
 * the turn radius entirely to the front wheels.
 *
 * Above 1.0 the outer wheel is driven faster than the geometry asks and the
 * inner slower. The pair now exerts a yaw moment on the chassis - the same
 * thing a skid-steer uses to turn - and the robot rotates faster than the
 * steering alone would carry it. The turn tightens.
 *
 * WHAT IT COSTS
 *   Tyre scrub, because the rear wheels are no longer rolling along the paths
 *   the geometry says they should. On a smooth floor that shows up as a
 *   slight rear slide.
 *
 *   Encoder odometry gets less honest for the same reason - the wheels turn
 *   further than the ground travelled. Distance during an arc is affected;
 *   the ANGLE is not, because the gyro measures the body directly. That is
 *   what makes this safe to use at all.
 *
 * WHERE IT STOPS
 *   The inner wheel slows as boost rises, and below about 55 rpm the deadband
 *   clamp takes over and the PID stops regulating it. ODOM_ARC_DIFF_MAX caps
 *   the split so the inner wheel can never be commanded below half the centre
 *   speed however large the boost is set.
 *
 * Try 1.5, measure R, then 2.0. Expect diminishing returns and a rear end
 * that feels progressively looser. */
#define ODOM_ARC_DIFF_MAX     0.50f

/* Stops the robot, recentres the steering, disables heading hold. */
void Odom_Stop(void);

/* The servo pulse width the heading loop is currently commanding, in
 * microseconds. Equals SERVO_CENTER_US when the robot is on course.
 * Exposed for tuning - put it on the OLED. */
uint16_t Odom_GetServoUs(void);

/* Heading error the loop is currently acting on, degrees. Also for tuning. */
float Odom_GetHeadingError(void);

/* Learned steering centre offset in microseconds. Once it settles, add it to
 * SERVO_CENTER_US in motors.h and it will start from zero again. */
float Odom_GetHeadingTrim(void);

/* Restore a saved trim. Rejects anything beyond HEADING_TRIM_MAX_US rather
 * than clamping - a value out there is a sender bug or a mechanical fault,
 * and neither should arrive looking like success.
 *
 * Now that SERVO_CENTER_US carries the measured centre the trim hovers near
 * zero, so restoring it matters far less than restoring the braking values.
 * It is here for completeness and as a way to prove a centre has drifted. */
uint8_t Odom_SetHeadingTrim(float us);

/* Signed distance left of the line the current move started on, mm. This is
 * the quantity the cross-track term drives to zero. */
float Odom_GetCrossTrack(void);

/* Fold this run's measured steering bias into the trim. Call ONCE when a
 * straight move finishes, from the motion layer. Does nothing if the run was
 * too short to have collected a meaningful average - see the note on
 * TRIM_MIN_SAMPLES. */
void Odom_LearnTrim(void);

/* 1 if the last straight was long enough to actually move the trim.
 *
 * Read it after a straight, before the next one starts. Exists because "did
 * not learn" and "has converged" produce the identical trim value, and a run
 * that silently taught nothing can otherwise be saved into a calibration
 * profile as though it had been measured. */
uint8_t Odom_TrimWasUpdated(void);

/* ------------------------------------------------------------------ */
/* Calibration helper                                                  */
/* ------------------------------------------------------------------ */

/* Spin-test support for refining WHEEL_BASE_MM. Note this is of limited use
 * on an Ackermann chassis, which cannot turn on the spot - it is kept for
 * the hand-pushed bench check only. */
void  Odom_CalibrateSpinStart(void);
float Odom_CalibrateSpinResult(void);

#endif /* __ODOM_H */
