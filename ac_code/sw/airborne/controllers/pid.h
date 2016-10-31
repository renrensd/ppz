#ifndef _PID_H_
#define _PID_H_

//#define DYNAMIC_INTEGRATOR_CLAMPING

struct _s_pid
{
	float Kp;
	float Ki;
	float Kd;

	float dT;
	float Fs;

	float d_ref_Fc;
	float d_fb_Fc;
	float d_ref_f_coef;
	float d_fb_f_coef;

	float Up;
	float Ui;
	float Ud;

	float err;
	float ref;
	float fb;
	float last_ref;
	float last_fb;
	float d_ref;
	float d_fb;
	float df_ref;
	float df_fb;

	float out;

	float UiMin;
	float UiMax;
	float outMin;
	float outMax;
};

static inline void pid_set_pid_coef(struct _s_pid *pid, float kp, float ki,
		float kd)
{
	pid->Kp = kp;
	pid->Ki = ki;
	pid->Kd = kd;
}

static inline void pid_set_out_range(struct _s_pid *pid, float outmin,
		float outmax)
{
	pid->outMin = outmin;
	pid->outMax = outmax;
}

static inline void pid_set_Ui_range(struct _s_pid *pid, float uimin,
		float uimax)
{
	pid->UiMin = uimin;
	pid->UiMax = uimax;
}

static inline float pid_calc_filter_coef(float dT, float Fc)
{
	float coef;

	coef = 2.0f * 3.1415926535897932f * Fc * dT;

	if (coef > 1.0f)
	{
		coef = 1.0f;
	}

	return coef;
}

static inline float pid_simple_filter(float coef, float last_out, float in)
{
	return (last_out + (in - last_out) * coef);
}

static inline void pid_set_filter_Fc(struct _s_pid *pid, float d_ref_Fc,
		float d_fb_Fc)
{
	pid->d_ref_Fc = d_ref_Fc;
	pid->d_fb_Fc = d_fb_Fc;
	pid->d_ref_f_coef = pid_calc_filter_coef(pid->dT, d_ref_Fc);
	pid->d_fb_f_coef = pid_calc_filter_coef(pid->dT, d_fb_Fc);
}

static inline void pid_ini(struct _s_pid *pid, float Fs)
{
	pid->Ui = 0;

	pid->UiMin = 0;
	pid->UiMax = 0;
	pid->outMin = 0;
	pid->outMax = 0;

	pid->last_ref = 0;
	pid->last_fb = 0;
	pid->out = 0;

	pid->Fs = Fs;
	pid->dT = 1.0f / Fs;

	pid->d_ref_f_coef = 1.0f;
	pid->d_fb_f_coef = 1.0f;
}

static inline void pid_reset(struct _s_pid *pid)
{
	pid->last_ref = 0;
	pid->last_fb = 0;
	pid->Ui = 0;
}

// using PID controller build in derivative calculation and simple 1 order filter
static inline float pid_loop_calc_1(struct _s_pid *pid, float ref, float fb)
{
	float Usum, limit;

	pid->ref = ref;
	pid->fb = fb;

	// calc P
	pid->err = ref - fb;
	pid->Up = pid->Kp * pid->err;

	// calc I
	pid->Ui += pid->Ki * pid->err * pid->dT;

#ifdef DYNAMIC_INTEGRATOR_CLAMPING
	limit = pid->outMax - pid->Up;
	if (limit > 0)
	{
		pid->UiMax = limit;
	}
	else
	{
		pid->UiMax = 0;
	}

	limit = pid->outMin - pid->Up;
	if (limit < 0)
	{
		pid->UiMin = limit;
	}
	else
	{
		pid->UiMin = 0;
	}
#endif

	if (pid->Ui > pid->UiMax)
	{
		pid->Ui = pid->UiMax;
	}
	else if (pid->Ui < pid->UiMin)
	{
		pid->Ui = pid->UiMin;
	}

	// calc D
	pid->d_ref = (pid->ref - pid->last_ref) * pid->Fs;
	pid->d_fb = (pid->fb - pid->last_fb) * pid->Fs;
	pid->last_ref = pid->ref;
	pid->last_fb = pid->fb;
	pid->df_ref = pid_simple_filter(pid->d_ref_f_coef, pid->df_ref, pid->d_ref);
	pid->df_fb = pid_simple_filter(pid->d_fb_f_coef, pid->df_fb, pid->d_fb);
	pid->Ud = (pid->df_ref - pid->df_fb) * pid->Kd;

	// calc out
	Usum = pid->Up + pid->Ui + pid->Ud;
	if (Usum > pid->outMax)
	{
		pid->out = pid->outMax;
	}
	else if (Usum < pid->outMin)
	{
		pid->out = pid->outMin;
	}
	else
	{
		pid->out = Usum;
	}

	return pid->out;
}

// using user set derivative
static inline float pid_loop_calc_2(struct _s_pid *pid, float ref, float fb,
		float d_ref, float d_fb)
{
	float Usum, limit;

	pid->ref = ref;
	pid->fb = fb;

	// calc P
	pid->err = ref - fb;
	pid->Up = pid->Kp * pid->err;

	// calc I
	pid->Ui += pid->Ki * pid->err * pid->dT;

#ifdef DYNAMIC_INTEGRATOR_CLAMPING
	limit = pid->outMax - pid->Up;
	if (limit > 0)
	{
		pid->UiMax = limit;
	}
	else
	{
		pid->UiMax = 0;
	}

	limit = pid->outMin - pid->Up;
	if (limit < 0)
	{
		pid->UiMin = limit;
	}
	else
	{
		pid->UiMin = 0;
	}
#endif

	if (pid->Ui > pid->UiMax)
	{
		pid->Ui = pid->UiMax;
	}
	else if (pid->Ui < pid->UiMin)
	{
		pid->Ui = pid->UiMin;
	}

	// calc D
	pid->df_ref = d_ref;
	pid->df_fb = d_fb;
	pid->Ud = (pid->df_ref - pid->df_fb) * pid->Kd;

	// calc out
	Usum = pid->Up + pid->Ui + pid->Ud;
	if (Usum > pid->outMax)
	{
		pid->out = pid->outMax;
	}
	else if (Usum < pid->outMin)
	{
		pid->out = pid->outMin;
	}
	else
	{
		pid->out = Usum;
	}

	return pid->out;
}

#endif

