#include "MPU9250.h"
#include <MahonyMadgwichKalmanFilter.h>
#include <Adafruit_BMP085.h>

MPU9250 mpu9250(Wire,0x68);
Adafruit_BMP085 bmp_180;

MahonyMadgwichKalmanFilter  mmkf;

double baseline; // baseline pressure
double Temp,Pascal,Altitude;
double mpu_temp;

// Sensor variables
float accel[3];  // Actually stores the NEGATED acceleration (equals gravity, if board not moving).
float magnetom[3];
float gyro[3];

float zz,zv;

uint32_t times = micros(),
  next_times = micros(),
  half_times = micros();

// Read data from MPU9250
void read_mpu9250() {
  
  mpu9250.readSensor();
  
  accel[0] = mpu9250.getAccelX_mss();
  accel[1] = mpu9250.getAccelY_mss();
  accel[2] = mpu9250.getAccelZ_mss();

  magnetom[0] = mpu9250.getMagX_uT();
  magnetom[1] = mpu9250.getMagY_uT();
  magnetom[2] = mpu9250.getMagZ_uT();

  gyro[0] = mpu9250.getGyroX_rads();
  gyro[1] = mpu9250.getGyroY_rads();
  gyro[2] = mpu9250.getGyroZ_rads();

  mpu_temp = mpu9250.getTemperature_C();
}

void read_bmp180(){
    Temp=bmp_180.readTemperature();
    Altitude= bmp_180.readAltitude();
    Pascal = bmp_180.readPressure();
}
  
void setup() {
  
  Serial.begin(115200);
  Serial.println("Sensor init..");
    
  if (!bmp_180.begin()) {
    Serial.println("bmp180 sensor hatası");
    while (1) {}
  }

  if (!mpu9250.begin()) {
    Serial.println("mpu9250 sensor hatası!");
    while (1) {}
  }
  
  // setting the accelerometer full scale range to +/-8G 
  mpu9250.setAccelRange(MPU9250::ACCEL_RANGE_4G);
  // setting the gyroscope full scale range to +/-500 deg/s
  mpu9250.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
  // setting DLPF bandwidth to 20 Hz
  mpu9250.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_41HZ);
  // setting SRD to 19 fo
  mpu9250.setSrd(9);

  Serial.println("Evertyhink ok..");
}

void loop() {
  
    times = micros();
    read_mpu9250();
    half_times = micros();
    read_bmp180();
    next_times = micros();
    
    /*Serial.print(mpu_temp);
    Serial.print(" ");
    
    Serial.print(Temp);
    
    Serial.print(" ");
    Serial.print(Pascal);
    Serial.print(" ");

    // Calculate altitude assuming 'standard' barometric
    // pressure of 1013.25 millibar = 101325 Pascal
    Serial.print(Altitude);
    Serial.print(" ");

    Serial.print(accel[0]);
    Serial.print(" ");
    Serial.print(accel[1]);
    Serial.print(" ");
    Serial.print(accel[2]);
    Serial.print(" ");
    Serial.print(magnetom[0]);
    Serial.print(" ");
    Serial.print(magnetom[1]);
    Serial.print(" ");
    Serial.print(magnetom[2]);
    Serial.print(" ");
    Serial.print(gyro[0],6);
    Serial.print(" ");
    Serial.print(gyro[1],6);
    Serial.print(" ");
    Serial.print(gyro[2],6);
    Serial.print(" ");

    Serial.print(half_times-times);
    Serial.print(" ");
    Serial.println(next_times-times);*/

    if(!mmkf.UpdateData(accel, gyro, magnetom,  Altitude)){
      zz = mmkf.GetAltitudeEstimation();
      zv = mmkf.GetVelocityEstimation();
      Serial.print(zz);
      Serial.print(" "); 
      Serial.print(zv);
      Serial.println(); 
    }else{
        Serial.println(Altitude);
        delay(10);
    }
   

        
}