/* Host regression test: real odometry/parser with fake encoder and gyro inputs. */
#include <assert.h>
#include <math.h>
#include "odom.h"
#include "commands.h"

static int32_t count_a, count_b;
static float gyro;
int32_t Encoder_A_GetCount(void) { return count_a; }
int32_t Encoder_B_GetCount(void) { return count_b; }
float IMU_GetHeading(void) { return gyro; }
uint8_t IMU_IsReady(void) { return 1; }
void IMU_ResetHeading(void) { gyro = 0; }
void PID_SetTargets(int16_t a, int16_t b) { (void)a; (void)b; }
void PID_Stop(void) {}
void Servo_SetMicroseconds(uint16_t us) { (void)us; }

static void straight(int counts)
{
    count_a += counts;
    count_b += counts;
    Odom_Update();
}

int main(void)
{
    Odom_Pose_t first, second, local;
    Command_t command = Cmd_ParseToken("?WPOSE");
    assert(command.op == CMD_Q_WPOSE);
    assert(Cmd_IsImmediate(command.op));
    assert(!Cmd_IsImmediate(CMD_FWD_UNTIL_US));
    Cmd_Init();
    assert(!Cmd_ParseLine("F10,?WPOSE"));
    assert(Cmd_QueueCount() == 0);

    Odom_Init();
    straight(100);
    Odom_GetWorldPose(&first);
    assert(first.x_mm > 0);
    Odom_Reset();
    Odom_GetPose(&local);
    assert(local.x_mm == 0);
    Odom_GetWorldPose(&second);
    assert(second.x_mm == first.x_mm);

    /* Several reset-separated approach passes accumulate, including reverse. */
    straight(100);
    Odom_Reset();
    straight(-50);
    Odom_GetWorldPose(&second);
    assert(fabsf(second.x_mm - 1.5f * first.x_mm) < 0.01f);

    /* A right turn followed by a reset and forward travel retains orientation. */
    gyro = -90;
    Odom_Update();
    Odom_Reset();
    straight(100);
    Odom_GetWorldPose(&second);
    assert(fabsf(second.heading_deg + 90) < 0.01f);
    assert(fabsf(second.y_mm + first.x_mm) < 0.01f);
    assert(fabsf(second.x_mm - 1.5f * first.x_mm) < 0.01f);
    return 0;
}
