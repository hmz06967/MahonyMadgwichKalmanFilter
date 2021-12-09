#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <math.h>
#include <string.h>


#include "../MahonyMadgwichKalmanFilter.h"

MahonyMadgwichKalmanFilter mmkf;

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

    char * line = NULL;
    size_t len = 0;
    ssize_t read;
    char * xread;
    float numberArray[1000];
    int i=0,j=0;

    float data[2000][15];

    if(select(argc, argv))return -1;

    FILE* fp =  fopen(argv[2], "r");
	
    if (fp == NULL) {
      fprintf(stderr,"Usage : %s -kf2/kf3/kf4/kf4d  ibg_binary_file\n", argv[0]);
		fprintf(stderr,"Error opening %s", argv[1]);
		return(-1);
	}


    while ((read = getline(&line, &len, fp)) != -1) {
        //printf("Retrieved line of length %zu:\n", read);
        xread = strtok(line, " ");

        while (xread != NULL){
            data[i][j] = strtof(xread, NULL);
            //printf ("%f ", data[i][j]);

            xread = strtok (NULL, " ");
            j++;
        }
        
        float acc[3], gyro[3], mgno[3], barombar;

        acc[0] =  data[i][4];
        acc[1] =  data[i][5];
        acc[2] =  data[i][6];

        gyro[0] =  data[i][7];
        gyro[1] =  data[i][8];
        gyro[2] =  data[i][9];

        mgno[0] =  data[i][10];
        mgno[1] =  data[i][11];
        mgno[2] =  data[i][12];

        barombar = data[i][3];

        mmkf.UpdateData(acc,gyro,mgno,barombar);
        printf("%f\n",mmkf.GetAltitudeEstimation());

        i++;
    }
    fclose(fp);
}   
