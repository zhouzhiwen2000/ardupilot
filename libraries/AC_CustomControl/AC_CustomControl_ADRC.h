#pragma once

#include "AC_CustomControl_Backend.h"

#ifndef CUSTOMCONTROL_EMPTY_ENABLED
    #define CUSTOMCONTROL_EMPTY_ENABLED AP_CUSTOMCONTROL_ENABLED
#endif

#if CUSTOMCONTROL_EMPTY_ENABLED

#define ESO_AngularRate_his_length 4

class AC_CustomControl_ADRC : public AC_CustomControl_Backend {
public:
    AC_CustomControl_ADRC(AC_CustomControl& frontend, AP_AHRS_View*& ahrs, AC_AttitudeControl_Multi*& att_control, AP_MotorsMulticopter*& motors, float dt);


    Vector3f update(void) override;
    void reset(void) override;

    // user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

protected:
    // declare parameters here
    AP_Float param1;
    AP_Float param2;
    AP_Float param3;
};

class TD4
{
	private:
			
	
	public:
		unsigned char tracking_mode;
		double x1;
		double x2;
		double x3;
		double x4;
	
		double P1;
		double P2;
		double P3;
		double P4;
		
		double r2p , r2n , r3p , r3n , r4p , r4n;
		
		inline unsigned char get_tracking_mode(){ return tracking_mode; }
		inline double get_x1(){ return x1; }
		inline double get_x2(){ return x2; }
		inline double get_x3(){ return x3; }
		inline double get_x4(){ return x4; }
	
		inline void reset()
		{
			this->x1 = this->x2 = this->x3 = this->x4 = 0;
			tracking_mode = 0;
		}
		
		inline TD4()
		{
			this->r2p = this->r2n = this->r3p = this->r3n = this->r4p = this->r4n = 1e12;			
			this->reset();
		}
		inline TD4( double P1, double P2, double P3, double P4 )
		{
			this->P1 = P1;
			this->P2 = P2;
			this->P3 = P3;
			this->P4 = P4;
			this->r2p = this->r2n = this->r3p = this->r3n = this->r4p = this->r4n = 1e12;
			
			this->reset();
		}
		
		inline double track4( double expect , double h )
		{
			this->tracking_mode = 4;
	
			double e1 = expect - this->x1;
			double e1_1 = -this->x2;
			double e1_2 = -this->x3;
			double e1_3 = -this->x4;
			double T2 = this->P1 * e1;
			double P1 = 0;
			if( T2 > this->r2p )
				T2 = this->r2p;
			else if( T2 < -this->r2n )
				T2 = -this->r2n;
			else
				P1 = this->P1;
			double T2_1 = P1 * e1_1;
			double T2_2 = P1 * e1_2;
			double T2_3 = P1 * e1_3;
			
			double e2 = T2 - this->x2;
			double e2_1 = T2_1-this->x3;
			double e2_2 = T2_2-this->x4;
			double T3 = this->P2 * e2;
			double P2 = 0;
			if( T3 > this->r3p )
				T3 = this->r3p;
			else if( T3 < -this->r3n )
				T3 = -this->r3n;
			else
				P2 = this->P2;
			T3 += T2_1;
			double T3_1 = P2 * e2_1 + T2_2;
			double T3_2 = P2 * e2_2 + T2_3;
			
			double e3 = T3 - this->x3;
			double e3_1 = T3_1-this->x4;
			double T4 = this->P3 * e3;
			double P3 = 0;
			if( T4 > this->r4p )
				T4 = this->r4p;
			else if( T4 < -this->r4n )
				T4 = -this->r4n;
			else
				P3 = this->P3;
			T4 += T3_1;
			double T4_1 = P3 * e3_1 + T3_2;
			
			double e4 = T4 - this->x4;
			double T5 = this->P4 * e4 + T4_1;
			
			this->x1 += h*this->x2;
			this->x2 += h*this->x3;
			this->x3 += h*this->x4;
			this->x4 += h*T5;
			
			return this->x1;
		}
		inline double track4( double expect,double expect_1,double expect_2,double expect_3,double expect_4 , double h )
		{
			this->tracking_mode = 4;
	
			double e1 = expect - this->x1;
			double e1_1 = expect_1 - this->x2;
			double e1_2 = expect_2 - this->x3;
			double e1_3 = expect_3 - this->x4;
			double T2 = this->P1 * e1;
			double P1 = 0;
			if( T2 > this->r2p )
				T2 = this->r2p;
			else if( T2 < -this->r2n )
				T2 = -this->r2n;
			else
				P1 = this->P1;
			T2 += expect_1;
			double T2_1 = P1*e1_1 + expect_2;
			double T2_2 = P1*e1_2 + expect_3;
			double T2_3 = P1*e1_3 + expect_4;
			
			double e2 = T2 - this->x2;
			double e2_1 = T2_1-this->x3;
			double e2_2 = T2_2-this->x4;
			double T3 = this->P2 * e2;
			double P2 = 0;
			if( T3 > this->r3p )
				T3 = this->r3p;
			else if( T3 < -this->r3n )
				T3 = -this->r3n;
			else
				P2 = this->P2;
			T3 += T2_1;
			double T3_1 = P2 * e2_1 + T2_2;
			double T3_2 = P2 * e2_2 + T2_3;
			
			double e3 = T3 - this->x3;
			double e3_1 = T3_1-this->x4;
			double T4 = this->P3 * e3;
			double P3 = 0;
			if( T4 > this->r4p )
				T4 = this->r4p;
			else if( T4 < -this->r4n )
				T4 = -this->r4n;
			else
				P3 = this->P3;
			T4 += T3_1;
			double T4_1 = P3 * e3_1 + T3_2;
			
			double e4 = T4 - this->x4;
			double T5 = this->P4 * e4 + T4_1;
			
			this->x1 += h*this->x2;
			this->x2 += h*this->x3;
			this->x3 += h*this->x4;
			this->x4 += h*T5;
			
			return this->x1;
		}
		
		inline double track3( double expect , double h )
		{
			this->tracking_mode = 3;
	
			double e2 = expect - this->x2;
			double e2_1 = -this->x3;
			double e2_2 = -this->x4;
			double T3 = this->P2 * e2;
			double P2 = 0;
			if( T3 > this->r3p )
				T3 = this->r3p;
			else if( T3 < -this->r3n )
				T3 = -this->r3n;
			else
				P2 = this->P2;
			double T3_1 = P2 * e2_1;
			double T3_2 = P2 * e2_2;
			
			double e3 = T3 - this->x3;
			double e3_1 = T3_1-this->x4;
			double T4 = this->P3 * e3;
			double P3 = 0;
			if( T4 > this->r4p )
				T4 = this->r4p;
			else if( T4 < -this->r4n )
				T4 = -this->r4n;
			else
				P3 = this->P3;
			T4 += T3_1;
			double T4_1 = P3 * e3_1 + T3_2;
			
			double e4 = T4 - this->x4;
			double T5 = this->P4 * e4 + T4_1;
			
			this->x1 += h*this->x2;
			this->x2 += h*this->x3;
			this->x3 += h*this->x4;
			this->x4 += h*T5;
			
			return this->x2;
		}
		
};

class ESO_AngularRate_Base
{
	public:
		virtual double get_T() const = 0;
		virtual double get_b() const = 0;
		virtual double get_u() const = 0;
	
		virtual void update_u( double u ) = 0;
		virtual double run( double v ) = 0;
	
		virtual double get_EsAngularRate() const = 0;
		virtual double get_EsDisturbance() const = 0;
		virtual double get_EsAngularAcceleration() const = 0;
		virtual double get_EsMainPower() const = 0;
};

class ESO_AngularRate : public ESO_AngularRate_Base
{
	private:
		double invT;
		double z_inertia;
		double z1;
		double z2;
		double h;
		double his_z1[ ESO_AngularRate_his_length ];
        bool err_sign;
        double err_continues_time;
	
	public:	
		double beta1;
		double beta2;
		double ceta1;
	    double ceta2;

		double T;	double get_T() const{ return this->T; };
		double b;	double get_b() const{ return this->b; };
		double u;	double get_u() const{ return this->u; };
	
		inline void init( double T , double b , double beta1 , double beta2 , double ceta1 , double ceta2 )
		{
            this->beta1 = beta1;
            this->beta2 = beta2;
            
            this->ceta1 = ceta1;
            this->ceta2 = ceta2;
            
            this->z1 = this->z2 = this->z_inertia = 0;
            this->err_continues_time = 0;	
            
            this->T = T;	this->invT = 1.0f / T;
            this->b = b;
            for( unsigned char i = 0 ; i < ESO_AngularRate_his_length ; ++i )
                this->his_z1[i] = 0;
		}
		
		inline void update_u( double u )
		{
            this->u = u;
            this->z_inertia += this->h * this->invT * ( this->b*this->u - this->z_inertia );
            this->z1 += this->h * ( this->z_inertia + this->z2 );
		}
		
		double run( double v )
        {
            float err = v - this->his_z1[0];

            if( (err > 0) ^ this->err_sign )
            {
                this->err_continues_time = 0;
                this->err_sign = err > 0;
            }
            else
                this->err_continues_time += h;
            
            float max_beta1_scale = 0.9f / this->beta1;
            float err_continues_time3 = this->err_continues_time*this->err_continues_time*this->err_continues_time;
            float beta1_scale = 1 + this->ceta1 * err_continues_time3;	
            float beta2_scale = 1 + this->ceta2 * err_continues_time3;		
            if( beta1_scale > 15 )
                beta1_scale = 15;
            if( beta2_scale > 15 )
                beta2_scale = 15;
            if( beta1_scale > max_beta1_scale )
                beta1_scale = max_beta1_scale;
            
            float z1_correction = beta1_scale*this->beta1*err;
            float z2_correction = beta2_scale*this->beta2*err;
            float max_z2_correction = z1_correction / ( ESO_AngularRate_his_length*h );
            if( fabsf(z2_correction) > fabsf(max_z2_correction) )
            {
                z2_correction = max_z2_correction;
                z1_correction = 0;
            }
            else
                z1_correction -= z2_correction * ESO_AngularRate_his_length*h;
            float filter_dt = h;
            for( unsigned char k = 0 ; k < ESO_AngularRate_his_length - 1 ; ++k )
            {
                this->his_z1[ k ] = this->his_z1[ k + 1 ] + z1_correction + filter_dt*z2_correction;
                filter_dt += h;
            }
            this->z2 += z2_correction;
            this->z1 += z1_correction + filter_dt*z2_correction;
            this->his_z1[ ESO_AngularRate_his_length - 1 ] = this->z1;
            
            this->h = h;
            return this->z2;
        }
		
		inline double get_EsAngularRate() const
		{
			return this->z1;
		}
		inline double get_EsDisturbance() const
		{
			return this->z2;
		}
		inline double get_EsAngularAcceleration() const
		{
			return this->z2 + this->z_inertia;
		}
		inline double get_EsMainPower() const
		{
			return this->z_inertia;
		}
};

#endif
