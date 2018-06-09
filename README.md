# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

## Reflections
#### 1. Sensor Fusion Date Processing
I read the vehicle speed and position in Frenet coordinates from the sensor data. I ignore the non ego cars which are far ahead or behind the ego car because those cars make no impact on the ego car's behavior planner. I also calculates the non ego car's lane position based on the d value in the Frenet coordiante. 
The behavior planner does not generate new path until it complete the previous path. Thus, I need to predict the s position of the non ego cars at the future time point when it runs out all the previous path data. I assume that each non ego car has a constant speed and does not change lanes within this short time period. The predicted s position will be used to estimate the relative distance between ego car and non ego cars in order to avoid collision. 
The lane speed is determined by the minimal speed of all the cars in the lane. It ensures a safe transition when the ego car changes lane. If there is no car in the lane, the lane speed is equal to the speed limit.
#### 2. Collision Avoidance
If a non ego car is located within 30m ahead of the ego car, a "too_close" flag will be triggered and the ego car will start to decelerate.
If the ego car detects a non ego cars which are less than 5m ahead or behind in the adjacent right or left lanes, it is not allowed to change to right or left lanes. 
#### 3. Behavior Planner
I use a finite state machine with three states: Keep Lane(KL), Lane Change Left(LCL), and Lane Change Right(LCR). According to the ego car's lane position, I calculate the possible states. For example, if the ego car is in the right most lane, the possible states are KL and LCL. If the ego car is in the left most lane, the possible states are KL and LCR. If the ego car is in the middle lane, the possible states are KL, LCL and LCR.
For each state, I define a cost function which considers the inefficiency cost and target lane cost. The inefficiency cost makes the car drive in the fastest possible lane. The target lane cost makes the car drive near the target lane(middle lane). 
#### 4. Trajectory Generation
I use the spline function to generation trajectory and to minimize jerk. The 
last two points of the previous path and three points at a far distance are used to initialize the spline calculation. The calculation is in local car coordinates. 
The previous path points are copied to the new trajectory. The other points are calculated by the spline function and added to the previous path points. 

