#pragma once

#include "AC_CustomControl_Backend.h"
#include <Filter/LowPassFilter2p.h>
#include <AP_Scheduler/AP_Scheduler.h>
#ifndef CUSTOMCONTROL_EMPTY_ENABLED
    #define CUSTOMCONTROL_EMPTY_ENABLED AP_CUSTOMCONTROL_ENABLED
#endif

#if CUSTOMCONTROL_EMPTY_ENABLED

#define ESO_AngularRate_his_length 4

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
		
		unsigned char get_tracking_mode(){ return tracking_mode; }
		double get_x1(){ return x1; }
		double get_x2(){ return x2; }
		double get_x3(){ return x3; }
		double get_x4(){ return x4; }
	
		void reset()
		{
			this->x1 = this->x2 = this->x3 = this->x4 = 0;
			tracking_mode = 0;
		}
		
		TD4()
		{
			this->r2p = this->r2n = this->r3p = this->r3n = this->r4p = this->r4n = 1e12;			
			this->reset();
		}
		TD4( double P1_p, double P2_p, double P3_p, double P4_p )
		{
			this->P1 = P1_p;
			this->P2 = P2_p;
			this->P3 = P3_p;
			this->P4 = P4_p;
			this->r2p = this->r2n = this->r3p = this->r3n = this->r4p = this->r4n = 1e12;
			
			this->reset();
		}
		void init(double P1_p, double P2_p, double P3_p, double P4_p)
        {
          	this->P1 = P1_p;
			this->P2 = P2_p;
			this->P3 = P3_p;
			this->P4 = P4_p;
            this->r2p = this->r2n = this->r3p = this->r3n = this->r4p = this->r4n = 1e12;
            this->reset();
        }

		double track4( double expect , double h )
		{
			this->tracking_mode = 4;
	
			double e1 = expect - this->x1;
			double e1_1 = -this->x2;
			double e1_2 = -this->x3;
			double e1_3 = -this->x4;
			double T2 = this->P1 * e1;
			double P1_v = 0;
			if( T2 > this->r2p )
				T2 = this->r2p;
			else if( T2 < -this->r2n )
				T2 = -this->r2n;
			else
				P1_v = this->P1;
			double T2_1 = P1_v * e1_1;
			double T2_2 = P1_v * e1_2;
			double T2_3 = P1_v * e1_3;
			
			double e2 = T2 - this->x2;
			double e2_1 = T2_1-this->x3;
			double e2_2 = T2_2-this->x4;
			double T3 = this->P2 * e2;
			double P2_v = 0;
			if( T3 > this->r3p )
				T3 = this->r3p;
			else if( T3 < -this->r3n )
				T3 = -this->r3n;
			else
				P2_v = this->P2;
			T3 += T2_1;
			double T3_1 = P2_v * e2_1 + T2_2;
			double T3_2 = P2_v * e2_2 + T2_3;
			
			double e3 = T3 - this->x3;
			double e3_1 = T3_1-this->x4;
			double T4 = this->P3 * e3;
			double P3_v = 0;
			if( T4 > this->r4p )
				T4 = this->r4p;
			else if( T4 < -this->r4n )
				T4 = -this->r4n;
			else
				P3_v = this->P3;
			T4 += T3_1;
			double T4_1 = P3_v * e3_1 + T3_2;
			
			double e4 = T4 - this->x4;
			double T5 = this->P4 * e4 + T4_1;
			
			this->x1 += h*this->x2;
			this->x2 += h*this->x3;
			this->x3 += h*this->x4;
			this->x4 += h*T5;
			
			return this->x1;
		}
		double track4( double expect,double expect_1,double expect_2,double expect_3,double expect_4 , double h )
		{
			this->tracking_mode = 4;
	
			double e1 = expect - this->x1;
			double e1_1 = expect_1 - this->x2;
			double e1_2 = expect_2 - this->x3;
			double e1_3 = expect_3 - this->x4;
			double T2 = this->P1 * e1;
			double P1_v = 0;
			if( T2 > this->r2p )
				T2 = this->r2p;
			else if( T2 < -this->r2n )
				T2 = -this->r2n;
			else
				P1_v = this->P1;
			T2 += expect_1;
			double T2_1 = P1_v*e1_1 + expect_2;
			double T2_2 = P1_v*e1_2 + expect_3;
			double T2_3 = P1_v*e1_3 + expect_4;
			
			double e2 = T2 - this->x2;
			double e2_1 = T2_1-this->x3;
			double e2_2 = T2_2-this->x4;
			double T3 = this->P2 * e2;
			double P2_v = 0;
			if( T3 > this->r3p )
				T3 = this->r3p;
			else if( T3 < -this->r3n )
				T3 = -this->r3n;
			else
				P2_v = this->P2;
			T3 += T2_1;
			double T3_1 = P2_v * e2_1 + T2_2;
			double T3_2 = P2_v * e2_2 + T2_3;
			
			double e3 = T3 - this->x3;
			double e3_1 = T3_1-this->x4;
			double T4 = this->P3 * e3;
			double P3_v = 0;
			if( T4 > this->r4p )
				T4 = this->r4p;
			else if( T4 < -this->r4n )
				T4 = -this->r4n;
			else
				P3_v = this->P3;
			T4 += T3_1;
			double T4_1 = P3_v * e3_1 + T3_2;
			
			double e4 = T4 - this->x4;
			double T5 = this->P4 * e4 + T4_1;
			
			this->x1 += h*this->x2;
			this->x2 += h*this->x3;
			this->x3 += h*this->x4;
			this->x4 += h*T5;
			
			return this->x1;
		}
		
		double track3( double expect , double h )
		{
			this->tracking_mode = 3;
	
			double e2 = expect - this->x2;
			double e2_1 = -this->x3;
			double e2_2 = -this->x4;
			double T3 = this->P2 * e2;
			double P2_v = 0;
			if( T3 > this->r3p )
				T3 = this->r3p;
			else if( T3 < -this->r3n )
				T3 = -this->r3n;
			else
				P2_v = this->P2;
			double T3_1 = P2_v * e2_1;
			double T3_2 = P2_v * e2_2;
			
			double e3 = T3 - this->x3;
			double e3_1 = T3_1-this->x4;
			double T4 = this->P3 * e3;
			double P3_v = 0;
			if( T4 > this->r4p )
				T4 = this->r4p;
			else if( T4 < -this->r4n )
				T4 = -this->r4n;
			else
				P3_v = this->P3;
			T4 += T3_1;
			double T4_1 = P3_v * e3_1 + T3_2;
			
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
	
		virtual void update_u( double u_p ) = 0;
		virtual double run( double v, double h_p  ) = 0;
	
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

		double T;	double get_T() const override{ return this->T; };
		double b;	double get_b() const override{ return this->b; };
		double u;	double get_u() const override{ return this->u; };
	
		void init( double T_p , double b_p , double beta1_p , double beta2_p , double ceta1_p , double ceta2_p )
		{
            this->beta1 = beta1_p;
            this->beta2 = beta2_p;
            
            this->ceta1 = ceta1_p;
            this->ceta2 = ceta2_p;
            
            this->z1 = this->z2 = this->z_inertia = 0;
            this->err_continues_time = 0;	
            
            this->T = T_p;	this->invT = 1.0f / T_p;
            this->b = b_p;
            for( unsigned char i = 0 ; i < ESO_AngularRate_his_length ; ++i )
                this->his_z1[i] = 0;
		}

		void reset()
		{
            this->z1 = this->z2 = this->z_inertia = 0;
            this->err_continues_time = 0;	
            for( unsigned char i = 0 ; i < ESO_AngularRate_his_length ; ++i )
                this->his_z1[i] = 0;						
		}
		
		void update_u( double u_p ) override
		{
            this->u = u_p;
            this->z_inertia += this->h * this->invT * ( this->b*this->u - this->z_inertia );
            this->z1 += this->h * ( this->z_inertia + this->z2 );
		}
		
		double run( double v , double h_p ) override
        {
            float err = v - this->his_z1[0];

            if( (err > 0) ^ this->err_sign )
            {
                this->err_continues_time = 0;
                this->err_sign = err > 0;
            }
            else
                this->err_continues_time += h_p;
            
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
            float max_z2_correction = z1_correction / ( ESO_AngularRate_his_length*h_p );
            if( fabsf(z2_correction) > fabsf(max_z2_correction) )
            {
                z2_correction = max_z2_correction;
                z1_correction = 0;
            }
            else
                z1_correction -= z2_correction * ESO_AngularRate_his_length*h_p;
            float filter_dt = h_p;
            for( unsigned char k = 0 ; k < ESO_AngularRate_his_length - 1 ; ++k )
            {
                this->his_z1[ k ] = this->his_z1[ k + 1 ] + z1_correction + filter_dt*z2_correction;
                filter_dt += h_p;
            }
            this->z2 += z2_correction;
            this->z1 += z1_correction + filter_dt*z2_correction;
            this->his_z1[ ESO_AngularRate_his_length - 1 ] = this->z1;
            
            this->h = h_p;
            return this->z2;
        }
		
		double get_EsAngularRate() const override
		{
			return this->z1;
		}
		double get_EsDisturbance() const override
		{
			return this->z2;
		}
		double get_EsAngularAcceleration() const override
		{
			return this->z2 + this->z_inertia;
		}
		double get_EsMainPower() const override
		{
			return this->z_inertia;
		}
};

class AC_CustomControl_ADRC : public AC_CustomControl_Backend {
public:
    AC_CustomControl_ADRC(AC_CustomControl& frontend, AP_AHRS_View*& ahrs, AC_AttitudeControl_Multi*& att_control, AP_MotorsMulticopter*& motors, float dt);


    Vector3f update(void) override;
    void reset(void) override;

    // user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

protected:
    // declare parameters here
    AP_Float P_b;
    AP_Float P_P1;
    AP_Float P_P2;
    AP_Float P_P3;
    AP_Float P_P4;
    AP_Float P_TD4P1;
    AP_Float P_TD4P2;
    AP_Float P_TD4P3;
    AP_Float P_TD4P4;
    AP_Float R_b;
    AP_Float R_P1;
    AP_Float R_P2;
    AP_Float R_P3;
    AP_Float R_P4;
    AP_Float R_TD4P1;
    AP_Float R_TD4P2;
    AP_Float R_TD4P3;
    AP_Float R_TD4P4;
    AP_Float Y_b;
    AP_Float Y_P1;
    AP_Float Y_P2;
    AP_Float Y_P3;
    AP_Float Y_P4;
    AP_Float Y_TD4P1;
    AP_Float Y_TD4P2;
    AP_Float Y_TD4P3;
    AP_Float Y_TD4P4;
    AP_Float Motor_T;
    LowPassFilter2pFloat disturbance_filter[3];
    TD4 Target_tracker[3];
    ESO_AngularRate ESO[3];
};


Quaternion Quaternion_get_PRQuat( Quaternion quat );

//  bool BUT_IIR_calc_freq( double k[3] , double sample_freq, const double cutoff_freq , double cp )
// {	
// 	if ( (cutoff_freq <= 0.0001) || (sample_freq <= 1.99 * cutoff_freq) ) {
// 			// no filtering
// 		return false;
// 	}
// 	double cos_PI_cp = cos( Pi * cp );
	
// 	double fr = sample_freq / cutoff_freq;
// 	double ohm = tan(Pi/fr);
// 	double ohm2 = ohm*ohm;
// 	double c = 1.0+2.0*cos_PI_cp*ohm + ohm2;
// 	double inv_c = 1.0 / c;
// 	k[0] = ohm2 * inv_c;
// 	k[1] = 2.0*(ohm2-1.0) * inv_c;
// 	k[2] = (1.0-2.0*cos_PI_cp*ohm+ohm2) * inv_c;
	
// 	return true;
// }

// /*二阶ButterWorth低通*/
// 	class Filter_Butter2_LP
// 	{
// 		private:
// 			bool available;	
		
// 			double k_1[3];
		
// 			double in_1[2];
// 			double out_1[2];
// 		public:
// 			//滤波器可用状态
// 			 bool is_available()
// 			{
// 				return this->available;
// 			}
// 			 void set_inavailable()
// 			{
// 				this->available = false;
// 			}
			
// 			//设置滤波频率
// 			 bool set_cutoff_frequency( double FS , double FC )
// 			{
// 				this->available = BUT_IIR_calc_freq( this->k_1 , FS , FC , 1.0 / 4 );
// 				return this->available;
// 			}
// 			 void set_cutoff_frequency_from( const Filter_Butter2_LP& filter )
// 			{
// 				this->k_1[0] = filter.k_1[0];	this->k_1[1] = filter.k_1[1];	this->k_1[2] = filter.k_1[2];
// 				this->available = filter.available;		
// 			}
			
// 			//设置初始值
// 			 void reset( double initial_value )
// 			{
// 				this->in_1[0] = this->in_1[1] = initial_value;
// 				this->out_1[0] = this->out_1[1] = initial_value;
// 			}
			
// 			//构造函数
// 			Filter_Butter2_LP()
// 			{
// 				this->available = false;
// 				this->reset(0);
// 			}
// 			Filter_Butter2_LP( double FS , double FC )
// 			{
// 				set_cutoff_frequency( FS , FC );
// 				this->reset(0);
// 			}
			
// 			//滤波
// 			 double get_result(){return this->out_1[0];}
// 			 double run( double newdata )
// 			{
// 				if( this->available )
// 				{
// 					double out_1_2 = this->out_1[1];	this->out_1[1] = this->out_1[0];
// 					this->out_1[0] = this->k_1[0]*( newdata + 2*this->in_1[0] + this->in_1[1] ) - this->k_1[1]*this->out_1[1] - this->k_1[2]*out_1_2;			
// 					this->in_1[1] = this->in_1[0];	this->in_1[0] = newdata;
// 				}
// 				else
// 					reset( newdata );
// 				return this->out_1[0];
// 			}
			
// 			//添加偏移
// 			 double add_offset( double offset )
// 			{
// 				this->in_1[0] += offset;	this->in_1[1] += offset;
// 				this->out_1[0] += offset;	this->out_1[1] += offset;				
// 				return this->out_1[0];
// 			}
// 	};
// /*二阶ButterWorth低通*/

#endif
