# One obstacle = one Euclidean cluster of obstacle points (frame given by the parent ObstacleArray header).
uint32 id                              # index within this ObstacleArray (not tracked between frames)
geometry_msgs/Point centroid           # mean of the cluster points [m]
geometry_msgs/Point min                # axis-aligned bounding box, lower corner [m]
geometry_msgs/Point max                # axis-aligned bounding box, upper corner [m]
float32 max_height_above_ground        # tallest point of the cluster measured from the local ground [m]
float32 distance                       # horizontal distance from the frame origin to the closest cluster point [m]
uint32 point_count
