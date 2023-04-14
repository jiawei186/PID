#include <stdio.h>
#include <math.h>

#define TIMES 300 // 迭代次数
#define MAX_SEPARATION 30 // 积分分离PID阈值
#define MAX_ANTISATURATION 30 // 抗积分饱和PID阈值
#define MIN_ANTIALLERGY 0.00001 // 抗过敏PID阈值

struct _pid
{
    float SetSpeed;    // 给定值
    float ActualSpeed; // 实际值

    float err;         // 误差值
    float err_k1;    // e(k - 1)
    float err_k2;    // e(k - 2)
    float integral;    // 积分控制值

    float output;         // 控制器输出值
    float output_last;    // 上一次的控制器输出值

    float Kp, Ki, Kd; // 比例、积分、微分系数

} pid;

void PID_init()
{
    printf("PID_init begin \n");

    pid.SetSpeed = 0.0;
    pid.ActualSpeed = 0.0;

    pid.err = 0.0;
    pid.err_k1 = 0.0;
    pid.err_k2 = 0.0;
    pid.integral = 0.0;

    pid.output = 0.0;
    pid.output_last = 0.0;

    pid.Kp = 0.2;
    pid.Ki = 0.015;
    pid.Kd = 0.2;
    
    printf("PID_init end \n");
}

float PID_position(float speed)
{

    PID_init();

    printf("PID_position:\n");

    pid.SetSpeed = speed;

    int count = 0;
    while (count < TIMES)
    {
        pid.err = pid.SetSpeed - pid.ActualSpeed;

        pid.integral += pid.err; // 积分控制值
        pid.output = pid.Kp * pid.err + pid.Ki * pid.integral + pid.Kd * (pid.err - pid.err_k1); // 控制器输出量

        pid.err_k1 = pid.err; // 更新误差值

        pid.ActualSpeed += pid.output * 1.0; // 设控制器输出量与电机转速的转换比为1:1

        printf("err:%-10f voltage:%-10f ActualSpeed:%-10f \n", pid.err, pid.output, pid.ActualSpeed);

        count++;
    }

    return pid.ActualSpeed;
}

float PID_incremental(float speed)
{
    PID_init();

    printf("PID_incremental:\n");

    pid.SetSpeed = speed;

    int count = 0;
    while (count < TIMES)
    {
        pid.err = pid.SetSpeed - pid.ActualSpeed;

        float delta = pid.Kp * (pid.err - pid.err_k1) + pid.Ki * pid.err + pid.Kd * (pid.err - 2 * pid.err_k1 + pid.err_k2); // 控制器输出量

        pid.err_k2 = pid.err_k1;
        pid.err_k1 = pid.err;

        pid.output += delta;
        pid.ActualSpeed += pid.output * 1.0; // 设控制器输出量与电机转速的转换比为1:1

        printf("err:%-10f delta:%-10f voltage:%-10f ActualSpeed:%-10f \n", pid.err, delta, pid.output, pid.ActualSpeed);

        count++;
    }

    return pid.ActualSpeed;
}

float PID_separation(float speed)
{
    PID_init();

    printf("PID_separation:\n");

    pid.Kp = 0.18;
    pid.Ki = 0.15;
    pid.Kd = 0.2;

    pid.SetSpeed = speed;

    int count = 0;
    while (count < TIMES)
    {
        pid.err = pid.SetSpeed - pid.ActualSpeed;

        float Gate;
        if(fabs(pid.err) > MAX_SEPARATION)
        {
            Gate = 0.0;
        } else
        {
            Gate = 1.0;
        }

        // Gate = 1.0;

        pid.integral += pid.err; // 积分控制值
        pid.output = pid.Kp * pid.err + Gate * pid.Ki * pid.integral + pid.Kd * (pid.err - pid.err_k1); // 控制器输出量

        pid.err_k1 = pid.err; // 更新误差值

        pid.ActualSpeed += pid.output * 1.0; // 设控制器输出量与电机转速的转换比为1:1

        printf("err:%-10f voltage:%-10f ActualSpeed:%-10f \n", pid.err, pid.output, pid.ActualSpeed);

        count++;
    }

    pid.Kp = 0.18;
    pid.Ki = 0.015;
    pid.Kd = 0.2;

    return pid.ActualSpeed;
}

float PID_antiSaturation(float speed)
{
    PID_init();

    printf("PID_antiSaturation:\n");

    pid.SetSpeed = speed;

    int count = 0;
    while (count < TIMES)
    {
        pid.err = pid.SetSpeed - pid.ActualSpeed;

        if (pid.output_last > MAX_ANTISATURATION)
        {
            if (pid.err < 0)
            {
                pid.integral += pid.err; // 只累加负误差
            }
        } else
        {
            pid.integral += pid.err;
        }

        pid.output = pid.Kp * pid.err + pid.Ki * pid.integral + pid.Kd * (pid.err - pid.err_k1); // 控制器输出量

        pid.output_last = pid.output; // 更新控制器输出量

        pid.err_k1 = pid.err; // 更新误差值

        pid.ActualSpeed += pid.output * 1.0; // 设控制器输出量与电机转速的转换比为1:1

        printf("err:%-10f voltage:%-10f ActualSpeed:%-10f \n", pid.err, pid.output, pid.ActualSpeed);

        count++;
    }

    return pid.ActualSpeed;
}

float PID_antiAllergy(float speed)
{
    PID_init();

    printf("PID_antiAllergy:\n");

    pid.SetSpeed = speed;

    int count = 0;
    while (count < TIMES)
    {
        pid.err = pid.SetSpeed - pid.ActualSpeed;

        pid.integral += pid.err; // 积分控制值
        pid.output = pid.Kp * pid.err + pid.Ki * pid.integral + pid.Kd * (pid.err - pid.err_k1); // 控制器输出量

        pid.err_k1 = pid.err; // 更新误差值

        if (fabs(pid.output) <= MIN_ANTIALLERGY)
        {
            pid.output = 0.0;
        }

        pid.ActualSpeed += pid.output * 1.0; // 设控制器输出量与电机转速的转换比为1:1

        printf("err:%-10f voltage:%-10f ActualSpeed:%-10f \n", pid.err, pid.output, pid.ActualSpeed);

        count++;
    }

    return pid.ActualSpeed;
}
