# COM offset

The backpack of the robot offsets the starting CoM position (from that of the gazebo simulation) by +4cm in the z-axis and -1cm in the x-axis. The optimal waypoints are scaled accordingly.

# minTau Regularization term

On iCubGenova02 the weight on the tau minimization term needs to be increased from the hard coded values to reduce vibrating in the legs. Default value is 1e-8, good value for the real robot is: 0.00001.


# Frames of reference

dh_frames and neck_1


# Task gains

must be increased a x5-x10 for real robot.
