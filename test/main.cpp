#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <math.h>
#include <string.h>


#include "../MahonyMadgwichKalmanFilter.h"

using namespace std;

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
   
   float acc[3], gyro[3], mgno[3], barombar;
   char * line = NULL;
   char * filename = argv[2];
   size_t len = 0;
   ssize_t read;
   char * xread;
   float numberArray[1000];
   int i=0,j=0;


   float data[15];

   if(select(argc, argv))return -1;
   //algo_option = false;

   mmkf.SetAHRSFilter(algo_option);

   FILE* fp =  fopen(filename, "r");

   if (fp == NULL) {
   fprintf(stderr,"Usage : %s \n", argv[0]);
   fprintf(stderr,"Error opening %s", argv[1]);
   return(-1);
}


    while ((read = getline(&line, &len, fp)) != -1) {
         //printf("Retrieved line of length %zu:\n", read);
         xread = strtok(line, " ");
         j=0;

         while (xread != NULL){
               data[j] = strtof(xread, NULL);
               //printf ("%f ", data[j]);
               xread = strtok (NULL, " ");
               j++;
         }
         
         barombar = data[2];

         acc[0] =  data[4]*100;
         acc[1] =  data[5]*100;
         acc[2] =  data[6]*100;

         gyro[0] =  data[10];
         gyro[1] =  data[11];
         gyro[2] =  data[12];

         mgno[0] =  data[7]*100;
         mgno[1] =  data[8]*100;
         mgno[2] =  data[9]*100;

         //printf("\n");

         //printf("%f %f %f %f %f %f %f %f %f %f\n",barombar, acc[0], acc[1],acc[2],gyro[0],gyro[1],gyro[2],mgno[0],mgno[1],mgno[2]);


         mmkf.UpdateData(acc,gyro,mgno,barombar);

         if(false)
         printf(
            "%f %f %f %f %f\n", 
            (barombar),
            mmkf.GetAltitudeEstimation(),
            mmkf.GetVelocityEstimation(),
            mmkf.GetAltitudeUncertainty(),
            mmkf.GetVelocityUncertainty()
         );

    }
    fclose(fp);
}   
  
