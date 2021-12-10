# MahonyMadgwichKalmanFilter DOC
 Sensor fusion kalman filter with ahrs mahony or madgwich Barometer and (9dof or 6dof )IMU 
 Support Arduino Library

## iS Kalman filter

  Hello, I used arduino uno to test this library, then I performed the test using MPU9250 9 Dof Fusion sensor and bmp180 pressure sensor. I did the graphical analysis of the data I read in the python environment.

  Change the value log filter for response  the save is output kalman filter and a print graph width print_graph.ipynb
  

  ```c++
  LOG_FILTER = true //becareful
  ```

![Aplication test](/doc/images/IMG_20211211_012116.jpg)
  
![Aplication test](/doc/images/uart.PNG)