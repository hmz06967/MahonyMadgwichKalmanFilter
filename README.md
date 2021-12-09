# MahonyMadgwichKalmanFilter
 Sensor fusion kalman filter with ahrs mahony or madgwich Barometer and IMU 

## Build And Run Code C
  ```c++
 cd examples
 
 g++ -o mmkf main.cpp ../MahonyMadgwichKalmanFilter.cpp
    
 ./mmkf -kf4d bmp180_mpu9250.txt  > test3.txt
