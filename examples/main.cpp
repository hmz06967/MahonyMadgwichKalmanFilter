#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <math.h>
#include <string.h>


#include "../MahonyMadgwichKalmanFilter.h"

MahonyMadgwichKalmanFilter mmkf;

#define KFMH    0
#define KFMG    1

int algo_option = -1;

int select(int argc, char* argv[]){

   if (argc != 3) {
      fprintf(stderr,"Error : %s \n", argv[0]);
      return -1;
   }
   if (!strcmp(argv[1], "-kfmg") ) {
      algo_option = KFMG;
   }
   else if (!strcmp(argv[1], "-kfmh") ) {
      algo_option = KFMH;
   }
   else {
      fprintf(stderr,"Usage : -kfmg, %s  \n",argv[1]);
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
      fprintf(stderr,"Usage : %s \n", argv[0]);
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
      
         //mmkf.SetAHRSFilter(algo_option);
         mmkf.UpdateData(acc,gyro,mgno,barombar);
         printf("%f\n", mmkf.GetAltitudeEstimation());

         i++;
    }
    fclose(fp);
}   
