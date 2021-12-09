#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <math.h>
#include <string.h>
#include "common.h"
#include "config.h"
#include "imu.h"
#include "flashlog.h"
#include "ringbuf.h"
#include "kalmanfilter2.h"
#include "kalmanfilter3.h"
#include "kalmanfilter4.h"

#define KF2    0
#define KF3    1
#define KF4    2
#define KF4D   3

int algo_option = -1;

int select(int argc, char* argv[]){

   if (argc != 3) {
      fprintf(stderr,"Usage : %s -kf2/kf3/kf4/kf4d  ibg_binary_file\n", argv[0]);
      }
   if (!strcmp(argv[1], "-kf2") ) {
      algo_option = KF2;
      }
   else
   if (!strcmp(argv[1], "-kf3") ) {
      algo_option = KF3;
      }
   else      
   if (!strcmp(argv[1], "-kf4") ) {
      algo_option = KF4;
      }
   else   
   if (!strcmp(argv[1], "-kf4d") ) {
      algo_option = KF4D;
      }
   else {
      fprintf(stderr,"Usage : %s -kf2/kf3/kf4/kf4d  ibg_binary_file\n", argv[0]);
   }

}

int main(int argc, char* argv[]) {

    float numberArray[1000];
    int i=0,j=0;

    if(select(argc, argv))return -1;

    FILE* fp =  fopen(argv[2], "r");
	
    if (fp == NULL) {
      fprintf(stderr,"Usage : %s -kf2/kf3/kf4/kf4d  ibg_binary_file\n", argv[0]);
		fprintf(stderr,"Error opening %s", argv[1]);
		return(-1);
	}

    //IBG_HDR hdr;
    //B_RECORD baro;
    //G_RECORD gps;
    //T_RECORD_
    //I_RECORD_

    Quaternion qua;
    Imu imu; 
    EulerAngles eul;

    int kalmanFilterInitialized = 0;
    float gx,gy,gz,ax,ay,az,mx, my, mz; 
    float baroAltCm, velNorth, velEast, velDown;
    float iirClimbrateCps, kfClimbrateCps,kfAltitudeCm;
    float glideRatio = 1.0f;
    float zAccelAverage;

    char * line = NULL;
    size_t len = 0;
    ssize_t read;
    char * xread;

    float data[2000][15];

    while ((read = getline(&line, &len, fp)) != -1) {
        //printf("Retrieved line of length %zu:\n", read);
        xread = strtok(line, " ");

        while (xread != NULL){
            data[i][j] = strtof(xread, NULL);
            //printf ("%f ", data[i][j]);

            xread = strtok (NULL, " ");
            j++;
        }

        baroAltCm = HPA2CMHG(data[i][3]);
        //baroAltCm = data[i][2];
        
        imu.acc[0] = data[i][4]*1000.0;//m to mg
        imu.acc[1] = data[i][5]*1000.0;
        imu.acc[2] = data[i][6]*1000.0;

        imu.gyro[0] = data[i][7];
        imu.gyro[1] = data[i][8];
        imu.gyro[2] = data[i][9];

        imu.mag[0] = data[i][10];
        imu.mag[1] = data[i][11];
        imu.mag[2] = data[i][12];
        
        float asqd = imu.acc[0]*imu.acc[0] + imu.acc[1]*imu.acc[1] + imu.acc[2]*imu.acc[2];
        // constrain use of accelerometer data to the window [0.75G, 1.25G] for determining
        // the orientation quaternion
        int useAccel = ((asqd > 562500.0f) && (asqd < 1562500.0f)) ? 1 : 0;	
        int useMag = 1;

        compensate_sensor_errors(&imu);
        //imu_MadgwickQuaternionUpdate(&imu, IMU_SAMPLE_PERIOD_SECS);
        //qua; qua.w = q[0]; qua.x = q[1]; qua.y = q[2]; qua.z = q[3];
        //eul = ToEulerAngles(qua);

        imu_mahonyAHRSupdate9DOF(useAccel, useMag, IMU_SAMPLE_PERIOD_SECS, &imu);

        float yawDeg,pitchDeg,rollDeg;
        imu_quaternion2YawPitchRoll(q0,q1,q2,q3, (float*)&yawDeg, (float*)&pitchDeg, (float*)&rollDeg);

        float gravityCompensatedAccel = imu_gravityCompensatedAccel(&imu, q0, q1, q2, q3);
        ringbuf_addSample(gravityCompensatedAccel); 


        switch (algo_option) {
            case 3 : // KF4D
            if (kalmanFilterInitialized == 0) {
                kalmanFilter4_configure(KF_ZMEAS_VARIANCE, KF_ACCEL_VARIANCE*1000.0f, true, baroAltCm, 0.0f, 0.0f);
                kalmanFilterInitialized = 1;
            }
            zAccelAverage = ringbuf_averageNewestSamples(10); 
            kalmanFilter4_predict(KF_SAMPLE_PERIOD_SECS);
            kalmanFilter4_update(baroAltCm, zAccelAverage, (float*)&kfAltitudeCm, (float*)&kfClimbrateCps);
            break;
        }

        //printf("\n");
        //printf("%s", line);
        i++;
    }
    fclose(fp);


}   
