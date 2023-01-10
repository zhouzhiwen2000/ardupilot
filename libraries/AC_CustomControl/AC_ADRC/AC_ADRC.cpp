#include <AP_Math/AP_Math.h>
#include "AC_ADRC.h"

// #define sin_45 0.70711f
// #define cR     718.078f
// #define cT     1.23884e-5
// #define L      0.255f
// #define d_1      88.448f
// #define Ixx_yy 0.016f
// static float b_const = 8.0f*cR*cT*L*sin_45;

// table of user settable parameters
const AP_Param::GroupInfo AC_ADRC::var_info[] = {

    AP_GROUPINFO("TD_CONTROL_R2",1,AC_ADRC,tdc_r2,25.0f),

    AP_GROUPINFO("TD_CONTROL_H2F",2,AC_ADRC,tdc_h2f,20.0f),

    AP_GROUPINFO("TD_R0",3,AC_ADRC,td_r0,1000.0f),

    AP_GROUPINFO("LESO_W",4,AC_ADRC,leso_w,120.0f),

    AP_GROUPINFO("B0",5,AC_ADRC,b_0,400.0f),

    AP_GROUPINFO("NLSEF_R1",6,AC_ADRC,nlsef_r1,100.0f),

    AP_GROUPINFO("NLSEF_H1F",7,AC_ADRC,nlsef_h1f,50.0f),

    AP_GROUPINFO("NLSEF_C",8,AC_ADRC,nlsef_c,0.01f),

    AP_GROUPINFO("NLSEF_KI",9,AC_ADRC,nlsef_ki,0.05f),

    AP_GROUPINFO("GAMMA",10,AC_ADRC,gamma,0.5f),

    AP_GROUPEND
};

float AC_ADRC::sign(float val)
{
	if(val >= 0.0f)
		return 1.0f;
	else
		return -1.0f;
}

float AC_ADRC::fhan(float v1, float v2, float r0, float h0)
{
	float d = h0 * h0 * r0;
	float a0 = h0 * v2;
	float y = v1 + a0;
	float a1 = sqrtf(d*(d + 8.0f*fabsf(y)));
	float a2 = a0 + sign(y)*(a1-d)*0.5f;
	float sy = (sign(y+d) - sign(y-d))*0.5f;
	float a = (a0 + y - a2)*sy + a2;
	float sa = (sign(a+d) - sign(a-d))*0.5f;
	
	return -r0*(a/d - sign(a))*sa - r0*sign(a);
}

float AC_ADRC::fal(float e, float alpha, float delta)
{
	if(fabsf(e) <= delta){
		return e / (powf(delta, 1.0f-alpha));
	}else{
		return powf(fabsf(e), alpha) * sign(e);
	}
}

AC_ADRC::AC_ADRC(float dt)
{
    AP_Param::setup_object_defaults(this, var_info);
    _dt = dt;
    _td_controller.init(_dt,tdc_r2,tdc_h2f*_dt);
    _td.init(_dt,td_r0,_dt);
    _leso.init(_dt,leso_w,b_0);//_leso.init(_dt,leso_w,400);
    _nlsef.init(_dt,nlsef_r1,nlsef_h1f*_dt,nlsef_c);
    leso_delay.init(3);
    int_i=0;
}

void AC_ADRC::reset()
{
    _td_controller.init(_dt,tdc_r2,tdc_h2f*_dt);
    _td.init(_dt,td_r0,_dt);
    _leso.init(_dt,leso_w,b_0);//_leso.init(_dt,leso_w,400);
    _nlsef.init(_dt,nlsef_r1,nlsef_h1f*_dt,nlsef_c);
    leso_delay.flush();
    int_i=0;
}

float AC_ADRC::update_all(float err, float gyr, float bth)
{
    // don't process inf or NaN
    if (!isfinite(err) || !isfinite(gyr)) {
        return 0.0;
    }
	float sp_rate;
	float rate_err;
	float u0;
    float out;
	//update leso
	adrc_att_observer_update(gyr,bth);

	/* TD control generates target rotational velocity */
	sp_rate = _td_controller.control(err);
	rate_err = sp_rate - gyr;
	
	/* control law */
	// TD extracts derivative of error
	_td.run(rate_err);
	// NLSEF control
	u0 = _nlsef.run(rate_err,_td.v2)/_leso.b0;
    // integral action
	float ki = nlsef_ki;
	if(int_i>=-0.1f&&int_i<=0.1f)
		int_i += rate_err * ki * _nlsef.h;
	u0 += int_i;
	// constrain output
	constrain_float(u0, -0.5f, 0.5f);
	
	/* disturbance rejection */
	out = adrc_att_dis_comp(u0);
    return out;
}

float AC_ADRC::adrc_att_dis_comp(float in)
{
	float out =0;
	out = in - gamma*_leso.z2/_leso.b0;
	
	// constrain output
	constrain_float(out, -0.5f, 0.5f);
	
	// delay control signal
	leso_delay.push(out);
	_leso.u = leso_delay.pop();
    return out;
}

void AC_ADRC::adrc_att_observer_update(const float gyr, float bth)
{
	/* update b0 */
	float bt = bth;
	// do not let base_throttle to be too small or too large
	constrain_float(bt, 0.3f, 0.7f);
	// float b0 = b_const*(cR*bt+d_1)/Ixx_yy;
	_leso.b0 = b_0;
	
	/* observer update */
	_leso.run(gyr);
}

void Delay_Block::init(uint16_t size_p)
{
	data = (float*)malloc(size_p*sizeof(float));
	size = size_p;
	head = 0;
	for(int i = 0 ; i < size ; i++){
		data[i] = 0.0f;
	}
}

void Delay_Block::flush()
{
	head = 0;
	for(int i = 0 ; i < size ; i++){
		data[i] = 0.0f;
	}
}

void Delay_Block::push(float val)
{
	head = (head+1) % size;
	data[head] = val;
}

float Delay_Block::pop()
{
	uint16_t tail = (head+1) % size;
	return data[tail];
}

void TD_Controller::init(float h_p, float r2_p, float h2_p)
{
    v1 = 0.0f;
	v2 = 0.0f;
	r2 = r2_p;
	h2 = h2_p;
	h = h_p;
}

float TD_Controller::control(float err)
{
	float fv = AC_ADRC::fhan(-err, v2, r2, h2);
	v1 += h * v2;
	v2 += h * fv;
	
	return v2;
}

void TD::init(float h_p, float r0_p, float h0_p)
{
	h = h_p;
	r0 = r0_p;
	h0 = h0_p;
	v1 = v2 = 0.0f;
}

void TD::run(float v)
{
	float fv = AC_ADRC::fhan(v1 - v, v2, r0, h0);
	
	v1 += h * v2;
	v2 += h * fv;
}

void LESO::init(float h_p, float w_p, float b0_p)
{
	h = h_p;
	// (s + w)^2 = s^2 + beta_1 * s + beta_2
	beta1 = 2.0f*w_p;
	beta2 = w_p*w_p;
	u = 0.0f;
	b0 = b0_p;
	
	z1 = z2 = 0.0f;
}

void LESO::run(float y)
{
	float e = z1 - y;
	z1 += h*(z2 + b0*u - beta1*e);
    z2 -= h*beta2*e;
}

void NLSEF::init(float h_p, float r1_p, float h1_p, float c_p)
{
    h = h_p;
	h1 = h1_p;
	r1 = r1_p;
	c = c_p;
}

float NLSEF::run(float e1, float e2)
{
	float u0 = -AC_ADRC::fhan(e1, c*e2, r1, h1);

	return u0;
}