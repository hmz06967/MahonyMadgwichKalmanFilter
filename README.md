# MahonyMadgwichKalmanFilter
 Sensor fusion kalman filter with ahrs mahony or madgwich Barometer and IMU 

## Run Code C
  ###build 
    g++ -o mmkf main.cpp ../MahonyMadgwichKalmanFilter.cpp
    
  ###run
    ./mmkf -kf4d bmp180_mpu9250.txt  > test3.txt
