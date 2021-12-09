/**
 * @author Ozk
 * @email hamza@hamzaozkan.com.tr
 * @create date 2021-12-09 07:31:33
 * @modify date 2021-12-09 07:31:33
 * @desc [description]
 */

#ifndef MmKalmanFilter_h_
#define MmKalmanFilter_h_

#define LOG_FILTER   false

class MahonyMadgwichKalmanFilter{

    public:
        MahonyMadgwichKalmanFilter();
        int UpdateData(float acc[3],float gyro[3], float mag[3], float baroAltCm);
        int SetFilter(float baroAltCm);
        float GetAltitudeUncertainty(void);
        float GetVelocityUncertainty(void);
        float GetAltitudeEstimation(void);
        float GetVelocityEstimation(void);
        void SetAHRSFilter(int filter);

        static void imu_mahonyAHRSupdate6DOF(int bUseAccel, float dt, float gx, float gy, float gz, float ax, float ay, float az);
        static void imu_madgwickAHRSupdate6DOF(int bUseAccel, float dt, float gx, float gy, float gz, float ax, float ay, float az);

        // HN
    private:

        static void imu_quaternion2YawPitchRoll(float q0, float q1, float q2, float q3, float* pYawDeg, float* pPitchDeg, float* pRollDeg);
           
        static void ringbuf_init();
        static void ringbuf_addSample(float sample);
        static float ringbuf_averageOldestSamples(int numSamples);
        static float ringbuf_averageNewestSamples(int numSamples);

        /*kalman filter*/
        static int kalmanFilter4_configure(float zSensorVariance, float aVariance, bool bAdaptUpdateVariance, float zInitial, float vInitial, float aInitial);
        static void kalmanFilter4_predict(float dt);
        static void kalmanFilter4_update(float zm, float am, float* pz, float* pv);


};
  

//extern MahonyMadgwichKalmanFilter MahonyMadgwichKalmanFilter; 
 
#endif /* MmKalmanFilter_h_ */
