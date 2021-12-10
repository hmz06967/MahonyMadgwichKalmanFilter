# MahonyMadgwichKalmanFilter
 Sensor fusion kalman filter with ahrs mahony or madgwich Barometer and (9dof or 6dof )IMU 
 Support Arduino Library

## Build And Run Code C
  ```c++
 cd examples
 
 g++ -o mmkf main.cpp ../MahonyMadgwichKalmanFilter.cpp
    
 ./mmkf -kfmg bmp180_mpu9250.txt  > test3.txt
  ```
## Read Doc

`<Kf campute doc>` : [kalman_filter]<https://github.com/hmz06967/MahonyMadgwichKalmanFilter/doc/kalman_filter.pdf>

`<Print graph python response>` : [print_graph]<https://github.com/hmz06967/MahonyMadgwichKalmanFilter/doc/print_graph.ipynb>