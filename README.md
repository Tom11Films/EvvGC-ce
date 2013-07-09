EvvGC-ce
========

Evvaldis's STM32 Gimbal Controller (community edition)

The EvvGC is a STM32 based gimbal controller originally designed by RCGroups member Evvaldis.  
It uses an IMU (MPU6050) to provide feedback for 3 brushless DC motors which are controlled similarly 
to a microstepping servo. The 3 coils of the 3 motors motors are fed a PWM duty cycle which rapidly 
sets the position of the camera. The STM32 reads the IMU and constantly updates that position to 
stabilize the camera on 3 axis.  There are also external inputs for RC control of pan/tilt.

Currently development of this open-source project is taking place at: 

http://www.rcgroups.com/forums/showthread.php?t=1872199

