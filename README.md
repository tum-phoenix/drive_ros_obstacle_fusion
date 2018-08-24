# Obstacle Fusion
Fuses [obstacles](https://github.com/tum-phoenix/drive_ros_msgs/blob/master/msg/Obstacle.msg) from multiple sources using a Kalman Filter. 

When receiving new objects they are added to a common object list. Objects are propagated in each timestep using a Kalman Filter. The filter employs a constant velocity model in x- and y-direction separately for each object. Intial and process covariances are defined in the config (.yaml) file. The measurement noise is defined by the centroid covariances included in the obstacles message. Old and new objects are merged when their centroids are below a specific threshold (`dist_threshold`). 

A separate trust value (between 0 and 1) defines when an obstacle will be deleted. With each obstacle arriving, the respective trust from the object is added to the existing (if any) or new object. In each prediction step the trust value is lowered by a specific value (`remove_trust`). If trust is equal or below zero the object is deleted from the object list.


Based on [Ego- and object motion estimation](https://mediatum.ub.tum.de/node?id=1452203) (chapter 4.3.3).
