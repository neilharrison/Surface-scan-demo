# Surface-scan-demo

Tool to aid scanning of an object's surface using a robotic manipulator. Specific sensor requirements are defined such as depth of field, standoff distance and field of view.

Given a pointcloud file, this will produce a set of positions and poses that could be used to scan the object entirely.

Currently only visualises the final poses, but these results could be output to a path planning algorithm. 

<b> Dependencies: </b>

Open3D

Octomap

Eigen3

