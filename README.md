# fotonic_wrapper
This ROS package is to publish pointclouds that detected by FOTONIC camera into ROS.

The FOTONIC sensor is inatalled on a segway platform in ROAHM Lab, University of Michigan.

You may run the launch file as:
```
roslaunch fotonic_wrapper fotonic.launch
```

# Additional Filters

I added some filters based on PCL library.

1, Filter out the noisy points with low brightness within 1 meter range, as well as those noise behind camera origin.

2, Clustered the pointcloud data published the clustered pointclouds into ROS.

3, Add a plane segmentation model to extract those pointclouds that are parallel to z-axis.

4, Clustered the planed pointclouds.
