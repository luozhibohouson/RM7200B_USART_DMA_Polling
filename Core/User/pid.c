  /* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <ctype.h>

#include "main.h"
#include "rm_math.h"
#include "controller.h"
#include "pid.h"

/**
* @brief Instance structure for the floating-point PID Control.
*/
typedef struct
{
    float A0;          /**< The derived gain, A0 = Kp + Ki + Kd . */
    float A1;          /**< The derived gain, A1 = -Kp - 2Kd. */
    float A2;          /**< The derived gain, A2 = Kd . */
    float state[3];    /**< The state array of length 3. */
    float Kp;          /**< The proportional gain. */
    float Ki;          /**< The integral gain. */
    float Kd;          /**< The derivative gain. */
} arm_rm_pid_instance_f32;

typedef struct
{
    int16_t A0;          /**< The derived gain, A0 = Kp + Ki + Kd . */
    int16_t A1;          /**< The derived gain, A1 = -Kp - 2Kd. */
    int16_t A2;          /**< The derived gain, A2 = Kd . */
    int16_t state[3];    /**< The state array of length 3. */
    int16_t Kp;          /**< The proportional gain. */
    int16_t Ki;          /**< The integral gain. */
    int16_t Kd;          /**< The derivative gain. */
} arm_rm_pid_instance_q15;

#if MAGIC_COOL_PID

arm_rm_pid_instance_q15 pid_rmx_q15;
arm_rm_pid_instance_f32 pid_rmx_f32;

void arm_rm_pid_init_f32(arm_rm_pid_instance_f32 * S, int32_t resetStateFlag)
{

  /* Derived coefficient A0 */
  S->A0 = S->Kp + S->Ki + S->Kd;

  /* Derived coefficient A1 */
  S->A1 = (-S->Kp) - ((float) 2.0 * S->Kd);

  /* Derived coefficient A2 */
  S->A2 = S->Kd;

  /* Check whether state needs reset or not */
  if (resetStateFlag)
  {
    /* Clear the state buffer.  The size will be always 3 samples */
    memset(S->state, 0, 3U * sizeof(float));
  }

}

float ref_pid_f32(arm_rm_pid_instance_f32 * S, float in)
{
	float out;

	/* y[n] = y[n-1] + A0 * x[n] + A1 * x[n-1] + A2 * x[n-2]  */
	out = S->state[2] + S->A0 * in + S->A1 * S->state[0] + S->A2 * S->state[1];

	/* Update state */
	S->state[1] = S->state[0];
	S->state[0] = in;
	S->state[2] = out;

	/* return to application */
	return (out);
}

void arm_rm_pid_init_q15(arm_rm_pid_instance_q15 * S, int32_t resetStateFlag)
{
    int32_t  temp;                                    /*to store the sum */

    /* Derived coefficient A0 */
    temp = S->Kp + S->Ki + S->Kd;
    if (temp > 0x7fff) {
        S->A0 = 0x7fff;
    } else if (temp < (int32_t)0xffff8000) {
        S->A0 = -0x8000;
    } else {
        S->A0 = temp;
    }

    /* Derived coefficients and pack into A1 */
    temp = -(S->Kd + S->Kd + S->Kp);
    if (temp > 0x7fff) {
        S->A1 = 0x7fff;
    } else if (temp < (int32_t)0xffff8000) {
        S->A1 = -0x8000;
    } else {
        S->A1 = temp;
    }
    S->A2 = S->Kd;

    /* Check whether state needs reset or not */
    if (resetStateFlag)
    {
        /* Clear the state buffer.  The size will be always 3 samples */
        memset(S->state, 0, 3U * sizeof(int16_t));
    }
}

int16_t ref_pid_q15(arm_rm_pid_instance_q15 * S, int16_t in)
{
	int32_t acc;
    int32_t acc1;
	int16_t out;
	int16_t A1, A2;

	A1 = S->A1;
	A2 = S->A2;

	/* acc = A0 * x[n]  */
	acc = ((int32_t) S->A0) * in;

	/* acc += A1 * x[n-1] + A2 * x[n-2]  */
	acc += (int32_t) A1 * S->state[0];
	acc += (int32_t) A2 * S->state[1];

	/* acc += y[n-1] */
	acc += (int32_t) S->state[2] << 15;

//    printf("================%d\r\n", acc);
	/* saturate the output */
    acc1 = acc;
    acc = acc >> 15;
    if (acc > (int32_t)0x7fff) {
        out = 0x7fff;
    } else if (acc < (int32_t)0xffff8000) {
        out = -0x8000;
    } else {
        out = acc;
    }

	/* Update state */
	S->state[1] = S->state[0];
	S->state[0] = in;
	S->state[2] = out;

	/* return to application */
	return (out);
}


int16_t ref_pid_q15_delta(arm_rm_pid_instance_q15 * S, int16_t in)
{
	int32_t acc;
	int16_t out;
	int16_t A1, A2;

	A1 = S->A1;
	A2 = S->A2;

	/* acc = A0 * x[n]  */
	acc = ((int32_t) S->A0) * in;

	/* acc += A1 * x[n-1] + A2 * x[n-2]  */
	acc += (int32_t) A1 * S->state[0];
	acc += (int32_t) A2 * S->state[1];

//    printf("================%d\r\n", acc);
	/* saturate the output */
    acc = acc >> 15;
    if (acc > (int32_t)0x7fff) {
        out = 0x7fff;
    } else if (acc < (int32_t)0xffff8000) {
        out = -0x8000;
    } else {
        out = acc;
    }

	/* Update state */
	S->state[1] = S->state[0];
	S->state[0] = in;

	/* return to application */
	return (out);
}

void pid_init(void)
{
//    pid_rmx_f32.Kp = 0.2;
//    pid_rmx_f32.Ki = 0.005;
//    pid_rmx_f32.Kd = 0;
//
//    arm_rm_pid_init_f32(&pid_rmx_f32, 1);
    // VPP
//    pid_rmx_q15.Kp = 3800;
//    pid_rmx_q15.Ki = 300;
//    pid_rmx_q15.Kd = 0;

    // dcdc
    pid_rmx_q15.Kp = 5800;
    pid_rmx_q15.Ki = 300;
    pid_rmx_q15.Kd = 0;

    arm_rm_pid_init_q15(&pid_rmx_q15, 1);
//    printf("asdf--- p:%d, i:%d, d:%d, A0:%d, A1:%d, A2:%d\r\n",
//        pid_rmx_q15.Kp, pid_rmx_q15.Ki, pid_rmx_q15.Kd,
//        pid_rmx_q15.A0, pid_rmx_q15.A1, pid_rmx_q15.A2);
}

float rm_pid_f(float in)
{
    return ref_pid_f32(&pid_rmx_f32, in);
}

int16_t rm_pid_d(int16_t in)
{
    return ref_pid_q15(&pid_rmx_q15, in);
//    return ref_pid_q15xs(&pid_rmx_q15, in);
}

int16_t rm_pid_delta(int16_t in)
{
    return ref_pid_q15_delta(&pid_rmx_q15, in);
}

#endif
