# MahonyMadgwichKalmanFilter
 Sensor fusion kalman filter with ahrs mahony or madgwich Barometer and (9dof or 6dof )IMU 
 Support Arduino Library
 https://www.youtube.com/watch?v=FGTOobGvMlY
 
## Build And Run Code C
  ```c++
 cd examples
 
 g++ -o mmkf main.cpp ../MahonyMadgwichKalmanFilter.cpp
    
 ./mmkf -kfmg bmp180_mpu9250.txt  > test3.txt
  ```
## Read Doc

[kalman_filter](doc/)

[print_graph](doc/print_graph.ipynb)

Mahony AHRS estimation algorithm/code - https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/
