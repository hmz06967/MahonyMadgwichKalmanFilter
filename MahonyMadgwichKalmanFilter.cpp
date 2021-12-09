/**
 * @author Ozk
 * @email hamza@hamzaozkan.com.tr
 * @create date 2021-12-09 07:31:21
 * @modify date 2021-12-09 07:31:21
 * @desc [description]
 */

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "MahonyMadgwichKalmanFilter.h"

#define true 	1
#define false 	0


#define delayMs(ms)     delay((ms)) 

#define MIN(x,y)                 ((x) < (y) ? (x) : (y))
#define MAX(x,y)                 ((x) > (y) ? (x) : (y))
#define ABS(x)                   ((x) < 0 ? -(x) : (x))
#define CLAMP(x,mn,mx)           {if (x <= (mn)) x = (mn); else if (x >= (mx)) x = (mx);}
#define CORE(x,t)                {if (ABS(x) <= (t)) x = 0;}
#define MCORE(x,t)               {if (x > (t)) x -= (t); else if (x < -(t)) x += (t); else x = 0;}
#define CORRECT(x,mx,mn)  		   (((float)((x)-(mn))/(float)((mx)-(mn))) - 0.5f)
#define INTEGER_ROUNDUP(val)     ((val) >= 0.0f ? (int32_t)((val)+0.5f) : (int32_t)((val)-0.5f))

#define RAD2DEG(r)   ((r)*57.29577951f)
#define DEG2RAD(d)   ((d)*0.017453292f)
#define HPA2CMHG(h)  ((h)*0.075006375f)
#define _180_DIV_PI         57.2957795f
#define PI_DIV_180          0.017453292f
#define _2_PI              6.2831853f

#define CORE_0    0
#define CORE_1    1
// common static configuration options
#define KF_ZMEAS_VARIANCE       	200.0f
#define KF_ACCEL_VARIANCE			90.0f
#define KF_ACCEL_UPDATE_VARIANCE	50.0f
#define KF_ACCELBIAS_VARIANCE   	0.005f

#define IMU_SAMPLE_PERIOD_SECS	0.024390f // IMU sensor samples @ 500Hz
#define KF_SAMPLE_PERIOD_SECS	0.02f // Kalman Filter output @ 50Hz

#define GRAVITY 9.8066

#define RINGBUF_SIZE    20


struct Imu{
    float acc[3],  gyro[3], mag[3];
};

struct EulerAngles {
    float roll_e, pitch_e, yaw_e;
};
// Set structs for converting result from Quaternion to Euler angles
struct Quaternion {
    float w, x, y, z;
};

typedef struct MmkfState_ {
    float z; // altitude
    float v; // climb/sink rate
    float a; // gravity-compensated net earth-z axis acceleration
    float b; // acceleration residual bias (post-calibration)
} MmkfState;

typedef struct KEY_VAL_ {
   char szName[30];
   char szValue[10];
   } KEY_VAL;

/*ringbuf*/
typedef struct RINGBUF_ {
   int head;
   float buffer[RINGBUF_SIZE];
} RINGBUF;

static RINGBUF RingBuf;

static float q0 = 0.0f, q1=0.0f, q2=0.0f, q3=0.0f, q4=0.0f;	// quaternion of sensor frame relative to auxiliary frame
volatile float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};	// quaternion of sensor frame relative to auxiliary frame

static float _baroAltCm = 0.0f, velNorth, velEast, velDown;
static float iirClimbrateCps, kfClimbrateCps,kfAltitudeCm;
static float glideRatio = 1.0f;
static float zAccelAverage;

static volatile float   YawDeg, PitchDeg, RollDeg;
static float data[10][15];  

static float Pzz;
static float Pzv;
static float Pza;
static float Pzb;

static float Pvz; 
static float Pvv;
static float Pva;
static float Pvb;

static float Paz; 
static float Pav; 
static float Paa;
static float Pab;

static float Pbz; 
static float Pbv; 
static float Pba; 
static float Pbb;

static float AccelVariance; // environmental acceleration variance, depends on conditions
static float BiasVariance; // assume a low value for acceleration bias noise variance
static float ZSensorVariance; //  altitude measurement noise variance
static float AUpdateVariance; //  acceleration update noise variance
static bool UseAdaptiveVariance = false;

const float magn_ellipsoid_center[3] = {-1.22362, -3.49591, -28.3068};
const float magn_ellipsoid_transform[3][3] = {{0.936683, -0.0120599, -0.00747369}, {-0.0120599, 0.997691, -5.88781e-05}, {-0.00747369, -5.88781e-05, 0.846255}};

int i=0,j=0;
int kalmanFilterInitialized = 0;
float yawDeg,pitchDeg,rollDeg;

int use_madgwick = true;

MmkfState  State;
Quaternion qua;
Imu imu; 
EulerAngles eul;

void imu_MadgwickQuaternionUpdate(Imu* imu, float deltat);
void compensate_sensor_errors(Imu* imu);
void Matrix_Vector_Multiply(const float a[3][3], const float b[3], Imu* imu);
float imu_gravityCompensatedAccel(Imu* imu, float q0, float q1, float q2, float q3);
void imu_mahonyAHRSupdate9DOF(int bUseAccel, int bUseMag, float dt, Imu* imu);

EulerAngles ToEulerAngles(Quaternion q);

MahonyMadgwichKalmanFilter::MahonyMadgwichKalmanFilter()
{
}

/*	filter 1: madgwick
		   0: mahony
*/

void MahonyMadgwichKalmanFilter::SetAHRSFilter(int filter){
	use_madgwick = filter> 0 ? 1 : 0;
}

int MahonyMadgwichKalmanFilter::SetFilter(float baroAltCm){
    if(kalmanFilterInitialized==0){
        MahonyMadgwichKalmanFilter::kalmanFilter4_configure(KF_ZMEAS_VARIANCE, KF_ACCEL_VARIANCE*1000.0f, true, baroAltCm, 0.0f, 0.0f);
        kalmanFilterInitialized = 1;
		return 1;
    }
}

/*  
*   Baromeyeter mbar->cmHG: data[i][3]
*   
*/
int MahonyMadgwichKalmanFilter::UpdateData(float acc[3],float gyro[3], float mag[3], float baroAltCm){

	_baroAltCm = HPA2CMHG(baroAltCm);
	//baroAltCm = data[i][2];
	
	imu.acc[0] = acc[0]*1000.0;//m to mg
	imu.acc[1] = acc[1]*1000.0;
	imu.acc[2] = acc[2]*1000.0;

	imu.gyro[0] = gyro[0];
	imu.gyro[1] = gyro[1];
	imu.gyro[2] = gyro[2];

	imu.mag[0] = mag[0];
	imu.mag[1] = mag[1];
	imu.mag[2] = mag[2];
	
	float asqd = imu.acc[0]*imu.acc[0] + imu.acc[1]*imu.acc[1] + imu.acc[2]*imu.acc[2];
	// constrain use of accelerometer data to the window [0.75G, 1.25G] for determining
	// the orientation quaternion
	int useAccel = ((asqd > 562500.0f) && (asqd < 1562500.0f)) ? 1 : 0;	
	int useMag = 1;

	MahonyMadgwichKalmanFilter::SetFilter(baroAltCm);

	compensate_sensor_errors(&imu);

	if(use_madgwick){
		imu_MadgwickQuaternionUpdate(&imu, IMU_SAMPLE_PERIOD_SECS);
		qua.w = q[0]; qua.x = q[1]; qua.y = q[2]; qua.z = q[3];
		eul = ToEulerAngles(qua);

	}else{
		imu_mahonyAHRSupdate9DOF(useAccel, useMag, IMU_SAMPLE_PERIOD_SECS, &imu);
		qua.w = q0; qua.x = q1; qua.y = q2; qua.z = q3;
		MahonyMadgwichKalmanFilter::imu_quaternion2YawPitchRoll(qua.w, qua.x, qua.y, qua.z, (float*)&yawDeg, (float*)&pitchDeg, (float*)&rollDeg);
	}

	float gravityCompensatedAccel = imu_gravityCompensatedAccel(&imu, qua.w, qua.x, qua.y, qua.z);
	
	MahonyMadgwichKalmanFilter::ringbuf_addSample(gravityCompensatedAccel); 

	zAccelAverage = ringbuf_averageNewestSamples(10); 
	MahonyMadgwichKalmanFilter::kalmanFilter4_predict(KF_SAMPLE_PERIOD_SECS);
	MahonyMadgwichKalmanFilter::kalmanFilter4_update(_baroAltCm, zAccelAverage, (float*)&kfAltitudeCm, (float*)&kfClimbrateCps);

	return eul.roll_e;
    
}   

/*RINGBUF*/

void MahonyMadgwichKalmanFilter::ringbuf_init() {
   memset(RingBuf.buffer,0,RINGBUF_SIZE);
   RingBuf.head = RINGBUF_SIZE-1;
}

void MahonyMadgwichKalmanFilter::ringbuf_addSample(float sample) {
   RingBuf.head++;
   if (RingBuf.head >= RINGBUF_SIZE) RingBuf.head = 0;
   RingBuf.buffer[RingBuf.head] = sample;
}


float MahonyMadgwichKalmanFilter::ringbuf_averageOldestSamples(int numSamples) {
   int index = RingBuf.head+1; // oldest Sample
   float accum = 0.0f;
   for (int count = 0; count < numSamples; count++) {
      if (index >= RINGBUF_SIZE) index = 0;
      accum += RingBuf.buffer[index];
      index++;
      }
   return accum/numSamples;
}   

float MahonyMadgwichKalmanFilter::ringbuf_averageNewestSamples(int numSamples) {
   int index = RingBuf.head; // newest Sample
   float accum = 0.0f;
   for (int count = 0; count < numSamples; count++) {
      if (index < 0) index = RINGBUF_SIZE - 1;
      accum += RingBuf.buffer[index];
      index--;
      }
   return accum/numSamples;
}   

/*KALMAN FILTER*/

// 4x4 process model state covariance estimate P 
// Note : P is symmetric
// Pzz Pzv Pza Pzb
// Pvz Pvv Pva Pvb
// Paz Pav Paa Pab
// Pbz Pbv Pba Pbb



int MahonyMadgwichKalmanFilter::kalmanFilter4_configure(float zSensorVariance, float aVariance, bool bAdaptUpdateVariance, float zInitial, float vInitial, float aInitial){
	ZSensorVariance = zSensorVariance;
	AUpdateVariance = KF_ACCEL_UPDATE_VARIANCE*1000.0f;
	AccelVariance = aVariance;
    BiasVariance = KF_ACCELBIAS_VARIANCE;
	UseAdaptiveVariance = bAdaptUpdateVariance;

	State.z = zInitial;
	State.v = vInitial;
    State.a = aInitial;
	State.b = 0.0f; // assume residual acceleration bias = 0 initially

	Pzz = 400.0f;
    Pzv = 0.0f;
	Pza = 0.0f;
	Pzb = 0.0f;
	
	Pvz = Pzv; 
	Pvv = 400.0f;
	Pva = 0.0f;
	Pvb = 0.0f;
	
	Paz = Pza;
	Pav = Pva;
	Paa = 50000.0f;
	Pab = 0.0f;

	Pbz = Pzb;
	Pbv = Pvb;
	Pba = Pab;
	Pbb = 400.0f;

	return 1;
}

void MahonyMadgwichKalmanFilter::kalmanFilter4_predict(float dt) {
	// Predicted (a priori) state vector estimate x_k- = F * x_k-1+
	float accel_true = State.a - State.b; // true acceleration = acceleration minus acceleration sensor bias
	State.z = State.z + (State.v * dt) + (accel_true * dt * dt* 0.5f);
	State.v = State.v + (accel_true * dt);

	// Predicted (a priori) state covariance estimate P_k- = (F * P_k-1+ * F_t) + Qk
	float dt2 = dt*dt;  // dt^2
	float dt3 = dt2*dt; // dt^3 
	float dt4 = dt2*dt2; // dt^4;
	float dt2div2 = dt2*0.5f; // dt^2/2
	float dt3div2 = dt3*0.5f; // dt^3/2
	float dt4div2 = dt4*0.5f; // dt^4/2
	float dt4div4 = dt4*0.25f; // dt^4/4
	
	float p00 = Pzz + 2.0f*Pzv*dt + (Pza - Pzb)*dt2  + Pvv*dt2div2 + (Pva - Pvb)*dt3 + (Paa+Pbb)*dt4div4 - Pab*dt4div2;
	float p01 = Pzv + dt*(Pza - Pzb + Pvv) + 3.0f*dt2div2*(Pva - Pvb) - Pab*dt3 + (Paa + Pbb)*dt3div2;
	float p02 = Pza + Pva*dt + (Paa - Pba)*dt2div2;
	float p03 = Pzb + Pvb*dt + (Pab - Pbb)*dt2div2;

	float p11 = Pvv + 2.0f*dt*(Pva - Pvb) + dt2*(Paa - 2.0f*Pab + Pbb);
	float p12 = Pva + dt*(Paa - Pba);
	float p13 = Pvb + dt*(Pab - Pbb);

	float p22 = Paa;
	float p23 = Pab;
	float p33 = Pbb;

	Pzz = p00;
	Pzv = p01;
	Pza = p02;
	Pzb = p03;

	Pvz = Pzv;
	Pvv = p11;
	Pva = p12;
	Pvb = p13;

	Paz = Pza;
	Pav = Pva;
	Paa = p22;
	Pab = p23;

	Pbz = Pzb;
	Pbv = Pvb;
	Pba = Pab;
	Pbb = p33; 

	// Add Q_k
	Paa = Paa + AccelVariance;
	Pbb = Pbb + BiasVariance;
}

void MahonyMadgwichKalmanFilter::kalmanFilter4_update(float zm, float am, float* pz, float* pv) {
	// Innovation Error y_k = measurement minus apriori estimate
	float z_err = zm - State.z;
	float a_err = am - State.a;

	// Innovation covariance S_k
	// S_k = (H * P_k- * H_t) + R_k
	float s00 = Pzz;
	float s01 = Pza;
	float s10 = s01;
	float s11 = Paa;

	// add R_k
	s00 = s00 + ZSensorVariance;
	if (UseAdaptiveVariance) {	
		float accel_ext = (am-State.b)*(am-State.b);
		// allows filter  to respond quickly to moderate/large accelerations while heavily filtering out noise
		// when there is low or no acceleration
		s11 = s11 + accel_ext;
		// allow system to update estimated bias only when there is low acceleration
		BiasVariance = 1.0f/(1.0f + 2.0f*accel_ext);	
		}
	else {
		s11 = s11 + AUpdateVariance;
		}

	// Compute S_k_inv
	float sdetinv = 1.0f/(s00*s11 - s10*s01);
	float sinv00 = sdetinv * s11;
	float sinv01 = -sdetinv * s10;
	float sinv10 = sinv01;
	float sinv11 = sdetinv * s00;

	// Kalman gain K_k [4x2] matrix
	// K_k = P_k- * H_t * S_k_inv
	float k00 = Pzz*sinv00 + Pza*sinv10;  
	float k01 = Pzz*sinv01 + Pza*sinv11;
	float k10 = Pvz*sinv00 + Pva*sinv10;
	float k11 = Pvz*sinv01 + Pva*sinv11;
	float k20 = Paz*sinv00 + Paa*sinv10;
	float k21 = Paz*sinv01 + Paa*sinv11;
	float k30 = Pbz*sinv00 + Pba*sinv10;
	float k31 = Pbz*sinv01 + Pba*sinv11;

	// Updated (a posteriori) state estimate x_k+ 
	// x_k+ = x_k- + K_k * y_k
	State.z = State.z + (k00*z_err + k01*a_err);
	State.v = State.v + (k10*z_err + k11*a_err);
	State.a = State.a + (k20*z_err + k21*a_err);
	State.b = State.b + (k30*z_err + k31*a_err);

	// Updated (a posteriori) state covariance estimate P_k+
	// P_k+ = (I - K_k * H_k)*P_k-
	float tmp = 1.0f - k00;
	float p00 = tmp*Pzz - k01*Paz;
	float p01 = tmp*Pzv - k01*Pav;
	float p02 = tmp*Pza - k01*Paa;
	float p03 = tmp*Pzb - k01*Pab;

	float p11 = -k10*Pzv + Pvv - k11*Pav;
	float p12 = -k10*Pza + Pva - k11*Paa;
	float p13 = -k10*Pzb + Pvb - k11*Pab;

	float p22 = -k20*Pza + (1.0f-k21)*Paa;
	float p23 = -k20*Pzb + (1.0f-k21)*Pab;

	float p33 = -k30*Pzb -k31*Pab + Pbb;

	Pzz = p00;
	Pzv = p01;
	Pza = p02;
	Pzb = p03;

	Pvz = Pzv;
	Pvv = p11;
	Pva = p12;
	Pvb = p13;

	Paz = Pza;
	Pav = Pva;
	Paa = p22;
	Pab = p23;

	Pbz = Pzb;
	Pbv = Pvb;
	Pba = Pab;
	Pbb = p33; 

	// return the state variables of interest (z and v)
	*pz = State.z;
	*pv = State.v;
		
	#if LOG_FILTER
		// WARNING : this should only be enabled for offline analysis of a downloaded data log !
		// e.g. see test code in /offline/kf 
		printf("%.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f\n",zm, State.z, Pzz, State.v, Pvv, State.a - State.b, Paa, State.b, Pbb);
	#endif

}

/*
The KF4d algorithm with dynamic accel/bias uncertainty injection has significantly lower climb/sinkrate uncertainty when the acceleration magnitude is low.

In terms of variometer audio feedback, there is perceptibly less jitter when the vario is stationary or moving with nearly constant velocity.

The trade-off is that for low accelerations/decelerations, there is a slight delay in response compared to the KF4 algorithm. For moderate/large accelerations, the KF4d response is on par with KF4.

KF4d is my preferred algorithm option for variometer response.
*/
 float MahonyMadgwichKalmanFilter::GetAltitudeUncertainty(){return Pzz;}
 float MahonyMadgwichKalmanFilter::GetVelocityUncertainty(){return Pvv;}

 float MahonyMadgwichKalmanFilter::GetAltitudeEstimation(){return State.z;}
 float MahonyMadgwichKalmanFilter::GetVelocityEstimation(){return State.v;}

//#define twoKpDef	(2.0f * 0.5f)	// 2 * proportional gain
#define twoKpDef  (2.0f * 5.0f) // 2 * proportional gain
#define twoKiDef	(2.0f * 0.0f)	// 2 * integral gain


float twoKp = twoKpDef;											// 2 * proportional gain (Kp)
float twoKi = twoKiDef;											// 2 * integral gain (Ki)

float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;	// integral error terms scaled by Ki


// Calibration outcomes
float GYRO_X_OFFSET = 0.0000820f;
float GYRO_Y_OFFSET = -0.0002375f;
float GYRO_Z_OFFSET = -0.0000904f;

float ACCEL_X_OFFSET =  0.1405817f;
float ACCEL_Y_OFFSET = -0.1235667f; 
float ACCEL_Z_OFFSET = -10.2402658f;

//---------------------------------------------------------------------------------------------------
// Variable definitions

//volatile float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};	// quaternion of sensor frame relative to auxiliary frame

//====================================================================================================
// Functions

//---------------------------------------------------------------------------------------------------
// AHRS algorithm update

EulerAngles ToEulerAngles(Quaternion q) {
    EulerAngles angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll_e = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w * q.y - q.z * q.x);
    if (fabs(sinp) >= 1)
        angles.pitch_e = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch_e = asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw_e = atan2(siny_cosp, cosy_cosp);

    return angles;
}

void imu_MadgwickQuaternionUpdate(Imu* imu, float deltat){
	
	float ax = imu->acc[0],  ay=imu->acc[1],  az=imu->acc[2],  
		  gx = imu->gyro[0], gy=imu->gyro[1],  gz=imu->gyro[2],  
		  mx = imu->mag[0],  my=imu->mag[1],  mz=imu->mag[2];

    float beta = 1.5;

    float norm;
    float hx, hy, _2bx, _2bz;
    float s1, s2, s3, s4;
    float qDot1, qDot2, qDot3, qDot4;
	float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
    // Auxiliary variables to avoid repeated arithmetic
    float _2q1mx;
    float _2q1my;
    float _2q1mz;
    float _2q2mx;
    float _4bx;
    float _4bz;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _2q4 = 2.0f * q4;
    float _2q1q3 = 2.0f * q1 * q3;
    float _2q3q4 = 2.0f * q3 * q4;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q1q4 = q1 * q4;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q2q4 = q2 * q4;
    float q3q3 = q3 * q3;
    float q3q4 = q3 * q4;
    float q4q4 = q4 * q4;

    // Normalise accelerometer measurement
    norm = 1.0f / sqrtf(ax * ax + ay * ay + az * az);
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Normalise magnetometer measurement
    norm = 1.0f / sqrtf(mx * mx + my * my + mz * mz);
    mx *= norm;
    my *= norm;
    mz *= norm;

    // Reference direction of Earth's magnetic field
    _2q1mx = 2.0f * q1 * mx;
    _2q1my = 2.0f * q1 * my;
    _2q1mz = 2.0f * q1 * mz;
    _2q2mx = 2.0f * q2 * mx;
    hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
    hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
    _2bx = sqrtf(hx * hx + hy * hy);
    _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    // Gradient decent algorithm corrective step
    s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    norm = sqrtf(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
    norm = 1.0f/norm;
    s1 *= norm;
    s2 *= norm;
    s3 *= norm;
    s4 *= norm;

    // Compute rate of change of quaternion
    qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
    qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
    qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
    qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

    // Integrate to yield quaternion
    q1 += qDot1 * deltat;
    q2 += qDot2 * deltat;
    q3 += qDot3 * deltat;
    q4 += qDot4 * deltat;
    norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
    norm = 1.0f/norm;
    q[0] = q1 * norm;
    q[1] = q2 * norm;
    q[2] = q3 * norm;
    q[3] = q4 * norm;
}

void imu_mahonyAHRSupdate9DOF(int bUseAccel, int bUseMag, float dt, Imu* imu) {
	float ax = imu->acc[0],  ay=imu->acc[1],  az=imu->acc[2],  
		  gx = imu->gyro[0], gy=imu->gyro[1],  gz=imu->gyro[2],  
		  mx = imu->mag[0],  my=imu->mag[1],  mz=imu->mag[2];

	float invNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;  
	float hx, hy, bx, bz;
	float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// Use 6dof  algorithm if magnetometer measurement invalid
	if(!bUseMag) {
		MahonyMadgwichKalmanFilter::imu_mahonyAHRSupdate6DOF(bUseAccel,dt, gx, gy, gz, ax, ay, az);
		return;
	}

	if(bUseAccel) {
		// Normalise accelerometer measurement
		invNorm = 1.0f/sqrt(ax * ax + ay * ay + az * az);
		ax *= invNorm;
		ay *= invNorm;
		az *= invNorm;     

		// Normalise magnetometer measurement
		invNorm = 1.0f/sqrt(mx * mx + my * my + mz * mz);
		mx *= invNorm;
		my *= invNorm;
		mz *= invNorm;   

        // Auxiliary variables to avoid repeated arithmetic
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;   

        // Reference direction of Earth's magnetic field
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrt(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

		// Estimated direction of gravity and magnetic field
		halfvx = q1q3 - q0q2;
		halfvy = q0q1 + q2q3;
		halfvz = q0q0 - 0.5f + q3q3;
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);  
	
		// Error is sum of cross product between estimated direction and measured direction of field vectors
		halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
		halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
		halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * dt;	// integral error scaled by Ki
			integralFBy += twoKi * halfey * dt;
			integralFBz += twoKi * halfez * dt;
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}
	
	// Integrate rate of change of quaternion
	gx *= (0.5f * dt);		// pre-multiply common factors
	gy *= (0.5f * dt);
	gz *= (0.5f * dt);
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx); 
	
	// Normalise quaternion
	invNorm = 1.0f/sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= invNorm;
	q1 *= invNorm;
	q2 *= invNorm;
	q3 *= invNorm;
}


void MahonyMadgwichKalmanFilter::imu_mahonyAHRSupdate6DOF(int bUseAccel, float dt, float gx, float gy, float gz, float ax, float ay, float az) {
	float invNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// Compute feedback only if accelerometer measurement valid )
	if(bUseAccel) {
		// Normalise accelerometer measurement
		invNorm = 1.0f/sqrt(ax * ax + ay * ay + az * az);
		ax *= invNorm;
		ay *= invNorm;
		az *= invNorm;        

		// Estimated direction of gravity and vector perpendicular to magnetic flux
		halfvx = q1 * q3 - q0 * q2;
		halfvy = q0 * q1 + q2 * q3;
		halfvz = q0 * q0 - 0.5f + q3 * q3;
	
		// Error is sum of cross product between estimated and measured direction of gravity
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * dt;	// integral error scaled by Ki
			integralFBy += twoKi * halfey * dt;
			integralFBz += twoKi * halfez * dt;
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}
	
	// Integrate rate of change of quaternion
	gx *= (0.5f * dt);		// pre-multiply common factors
	gy *= (0.5f * dt);
	gz *= (0.5f * dt);
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx); 
	
	// Normalise quaternion
	invNorm = 1.0f/sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= invNorm;
	q1 *= invNorm;
	q2 *= invNorm;
	q3 *= invNorm;
}

// HN
void MahonyMadgwichKalmanFilter::imu_quaternion2YawPitchRoll(float q011, float q111, float q211, float q311, float* pYawDeg, float* pPitchDeg, float* pRollDeg) {
    float invNorm = 1.0f/sqrt(q011*q011 + q111*q111 + q211*q211 + q311*q311);
    q011 *= invNorm;
    q111 *= invNorm;
    q211 *= invNorm;
    q311 *= invNorm;

    *pYawDeg   = _180_DIV_PI * atan2(2.0f * (q111*q211 + q011*q311), q011*q011 + q111*q111 - q211*q211 - q311*q311);
    *pPitchDeg = _180_DIV_PI * -asin(2.0f * (q111*q311 - q011*q211));
    *pRollDeg  = _180_DIV_PI * atan2(2.0f * (q011*q111 + q211*q311), q011*q011 - q111*q111 - q211*q211 + q311*q311);
}


float imu_gravityCompensatedAccel(Imu* imu, float q022, float q122, float q222, float q322){
    float acc = 2.0*(q122*q322 - q022*q222)*imu->acc[0] + 2.0f*(q022*q122 + q222*q322)*imu->acc[1] + (q022*q022 - q122*q122 - q222*q222 + q322*q322)*imu->acc[2] - 1000.0f;
    acc *= 0.9807f; // in cm/s/s, assuming ax, ay, az are in milli-Gs
	return acc;
}

// Multiply 3x3 matrix with vector: out = a * b
// out has to different from b (no in-place)!
void Matrix_Vector_Multiply(const float a[3][3], const float b[3], Imu* imu)
{
  for(int x = 0; x < 3; x++)
  {
    imu->mag[x] = a[x][0] * b[0] + a[x][1] * b[1] + a[x][2] * b[2];
  }
}

// Apply calibration to raw sensor readings
void compensate_sensor_errors(Imu* imu) {
	float magnetom[3]; 
	float magnetom_tmp[3]; 

    // Compensate accelerometer error
    // Compensate accelerometer error
    imu->acc[0] = imu->acc[0] - ACCEL_X_OFFSET;
    imu->acc[1] = imu->acc[1] - ACCEL_Y_OFFSET;
    imu->acc[2] = imu->acc[2] - (ACCEL_Z_OFFSET + GRAVITY);

    // Compensate magnetometer error
    for (int i = 0; i < 3; i++)
      magnetom_tmp[i] = magnetom[i] - magn_ellipsoid_center[i];
	  
    Matrix_Vector_Multiply(magn_ellipsoid_transform, magnetom_tmp, imu);

    // Compensate gyroscope error
    imu->gyro[0] -= GYRO_X_OFFSET;
	imu->gyro[1] -= GYRO_Y_OFFSET;
    imu->gyro[2] -= GYRO_Z_OFFSET;
}
 
//MahonyMadgwichKalmanFilter MahonyMadgwichKalmanFilter = MahonyMadgwichKalmanFilter();