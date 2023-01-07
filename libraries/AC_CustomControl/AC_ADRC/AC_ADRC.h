#pragma once


#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>

class Delay_Block
{
public:
    void init(uint16_t size_p);
    void flush();
    void push(float val);
    float pop();

private:
	uint16_t size;
	uint16_t head;
	float *data;
};

class TD_Controller
{
public:
    void init(float h_p, float r2_p, float h2_p);
    float control(float err);

private:
	float v1;
	float v2;
	float r2;
	float h2;
	float h;
};

class TD
{
public:
    void init(float h_p, float r0_p, float h0_p);
    void run(float v);
    float v2;

private:
	float v1;
	float r0;
	float h0;
	float h;
};

class LESO
{
public:
    void init(float h_p, float w_p, float b0_p);
    void run(float y);
	float z2;
	float b0;
	float u;

private:
	float h;
	float beta1;
	float beta2;
	/* LESO */
	float z1;

};

class NLSEF
{
public:
    void init(float h_p, float r1_p, float h1_p, float c_p);
    float run(float e1, float e2);
	float h;

private:
	float h1;
	float r1;
	float c;
};

class AC_ADRC {
public:
    AC_ADRC(float dt);

    CLASS_NO_COPY(AC_ADRC);

    float update_all(float err, float gyr, float bth);
    static float sign(float val);
    static float fhan(float v1, float v2, float r0, float h0);
    static float fal(float e, float alpha, float delta);
    // Reset filter
    void reset();



    static const struct AP_Param::GroupInfo var_info[];
protected:

    // parameters
    AP_Float tdc_r2;
    AP_Float tdc_h2f;
    AP_Float td_r0;
    AP_Float leso_w;
    AP_Float b_0;
    AP_Float nlsef_r1;
    AP_Float nlsef_h1f;
    AP_Float nlsef_c;
    AP_Float nlsef_ki;
    AP_Float gamma;   
    TD_Controller _td_controller;
    TD _td;
    NLSEF _nlsef;
    LESO _leso;
    Delay_Block leso_delay;
    
    float _dt;
    float int_i;
    float adrc_att_dis_comp(float in);
    void adrc_att_observer_update(const float gyr, float bth);
};
