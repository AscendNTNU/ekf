# EKF
Extended Kalman Filter for perception data from mission 9


Uses SensorFusion TinyEKF library: Sensor fusion on Arduino using TinyEKF.  
Copyright (C) 2015 Simon D. Levy.
MIT License

# HOW TO USE
Start the simulator (tested with v0.5.1, the topic names may change).
start the node : `rosrun ekf ekf_node`

# Publishing topics:

/ekf/measurement: measurement vector used by the kf
/ekf/module/state:output of the filter, position and velocity of the module
/ekf/state      : state vector of the kf
