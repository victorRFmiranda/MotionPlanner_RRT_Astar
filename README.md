# MotionPlanner_RRT_Astar
This is a ROS package that contains path planning algorithms for robots, considering a known 2D environment.
All algorithms use the information from an occupancy map and robot odometry to compute a free-obstacle path and follow it.
To follow the path, a simple controller, proportional to the position error between the robot and the path points, defines the robot's velocities.

## Installation

### Dependencies

Before building the package, install the following dependencies:

```
$ sudo apt install python-rosinstall python-rosinstall-generator build-essential
$ sudo apt-get install python-catkin-tools
$ sudo apt install python-wstool
$ pip install catkin_pkg
```

### Building Package

Install this package on your catking workspace:

```
$ cd ~/catkin_ws/src/
$ git clone https://github.com/victorRFmiranda/MotionPlanner_RRT_Astar
$ cd ..
$ rosdep install --from-paths src --ignore-src --rosdistro kinetic -y
$ catkin build
$ source devel/setup.bash
```


## Tutorial
The listed algorithms are:
* A*
* RRT
* RRT Kinodynamic
* RRT*

For each algorithm, there is a specific launcher as an example of running.
In case of the RRT*:
```
 $ roslaunch MotionPlanner_RRT_Astar rrt_star.launch
```
The command line above will run the ROS Stage simulator in a predefined scenario, and the respective occupancy map will start to publish.
At this moment two windows will open: The simulator and the Rviz, which shows the environment map and the robot's pose.
The planning algorithm will wait for a goal position. The user can select this position by using the **Publish Point** tool in rviz and clicking on the desired point.
Then, the path will be planned and an image appears with a plot of the computed route.
When this image is closed, the robot will start to follow the path until reaching the end of the path.
At this time, the process will repeat, waiting for a new goal position.

