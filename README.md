[//]: # (Image References)
 
[image1]: ./support/Behavior_control_diagram.jpg 
[image2]: ./support/Scheduling_Compute_time.jpg
[image3]: ./support/Finite_state_machine.jpg
[image4]: ./support/Cost_function_classes.jpg
[image5]: ./support/frenet_coordinate_system.jpg 
[image6]: ./support/Prediction_1_example.jpg 
[image7]: ./support/cubic_Spline.jpg
[image8]: ./support/Cost_schematic_chart.jpg
[image9]: ./support/Simulator_lap.jpg


## Eduardo Ribeiro de Campos - September 2019


# **Path Planning Project - Highway driving** 
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

To conclude the project is necessary reach the goals according to the project [rubric](https://review.udacity.com/#!/rubrics/1971/view).

The unput README File from udacity with a lot of instructions about the projecy is [here](./Udacity_README.md)



## 1. Project Overview

In this project the goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

The map of the highway is in [highway_map.txt](./data/map_data.txt)<br/>
Each waypoint in the list contains [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

### 1.1 - Inputs

**Here is the data provided from the Simulator to the C++ Program**

Main car's localization Data (No Noise)
- ["x"] The car's x position in map coordinates.<br/>
- ["y"] The car's y position in map coordinates.<br/>
- ["s"] The car's s position in frenet coordinates.<br/>
- ["d"] The car's d position in frenet coordinates.<br/>
- ["yaw"] The car's yaw angle in the map.<br/>
- ["speed"] The car's speed in MPH.<br/>

**Previous path data given to the Planner**

Note: Return the previous list but with processed points removed, can be a nice tool to show how far along the path has processed since last time.

- ["previous_path_x"] The previous list of x points previously given to the simulator.<br/>
- ["previous_path_y"] The previous list of y points previously given to the simulator.<br/>


**Previous path's end s and d values**

- ["end_path_s"] The previous list's last point's frenet s value.<br/>
- ["end_path_d"] The previous list's last point's frenet d value.<br/>

**Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)**

- ["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates.


### 1.2 - Details.

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.


## 2.Behavior Control diagram.

![alt text][image1]

### 2.1 - Sensor fusion.

Here we have the inputs already explained in item 1.1. in the code file the attributes for others cars are available twice from line 136 to 141 and from line 236 to 241, and the information about our car are available from line 85 to 101. 

### 2.2 - Localization. (Highway Map).

Inside [data/highway_map.csv](./data/highway_map.csv) there is a list of waypoints that go all the way around the track. The track contains a total of 181 waypoints, with the last waypoint mapping back around to the first. The waypoints are in the middle of the double-yellow dividing line in the center of the highway.

The track is 6945.554 meters around (about 4.32 miles). If the car averages near 50 MPH, then it should take a little more than 5 minutes for it to go all the way around the highway.

The highway has 6 lanes total - 3 heading in each direction. Each lane is 4 m wide and the car should only ever be in one of the 3 lanes on the right-hand side. The car should always be inside a lane unless doing a lane change.


### 2.3 - Prediction. (Waypoint Data / Frenet coordinate system).

Each waypoint has an (x,y) global map position, and a Frenet s value and Frenet d unit normal vector (split up into the x component, and the y component).

The s value is the distance along the direction of the road. The first waypoint has an s value of 0 because it is the starting point.

The d vector has a magnitude of 1 and points perpendicular to the road in the direction of the right-hand side of the road. The d vector can be used to calculate lane positions. For example, if you want to be in the left lane at some waypoint just add the waypoint's (x,y) coordinates with the d vector multiplied by 2. Since the lane is 4 m wide, the middle of the left lane (the lane closest to the double-yellow dividing line) is 2 m from the waypoint.

If you would like to be in the middle lane, add the waypoint's coordinates to the d vector multiplied by 6 = (2+4), since the center of the middle lane is 4 m from the center of the left lane, which is itself 2 m from the double-yellow dividing line and the waypoints.

**Converting Frenet Coordinates**

We have included a helper function, `getXY`, which takes in Frenet (s,d) coordinates and transforms them to (x,y) coordinates. more details see the [helpers.h](./src/helpers.h) file.

more informations about frenet coordinate system could be checked [here](http://www.sci.brooklyn.cuny.edu/~mate/misc/frenet_serret.pdf) about math details and [here](https://en.wikipedia.org/wiki/Frenet–Serret_formulas) for a general overview.

Below a schematic piture.

![alt text][image5]

In the [code file](./src/main.cpp) the prediction about the other cars were checked in a `for loop`function interating along the sensor fusion list to predict the lanes status if have a gap to make a lane change considering the our car postion and a red zone distance (line 130).<br/>
Below a schematic picture showing a generic case when it is not possible change to the middle lane. .

![alt text][image6]


### 2.4 - Behavior. 


About speed, the acceleration will be considered while the speed limit (50 km/h) is not reached.If a car ahead is detected, the decelerate process will be started until the distance between the cars was smaller than the red zone security distance equal 30m ( See code file - line 123).

To create the condition for the car behavior it was considered 2 main approaches

- Finite State Machine in a Self driving Car.<br/>
- Cost Function classes.<br/>

#### 2.4.1 - Finite State Machine in a Self driving Car

Below a chart from finite state machine applied in a Self driving Car.

![alt text][image3]

Main points to be considered about statements.



**Keep Lane:**

- The criteria to the car keep the lane is the case when does not have any car ahead.


**Prepare for lane change:**

- The prior condition to the car plan a change lane maneuver is find a other car ahead in a distance less than 30m. if this condition is not true, the car will keep the lane accelerating until the speed limit. 

**Lane change:**

If all the criterias ( cost function)  was satisfied, the car will Change lane.<br/> 
Bellow the main criterias considered on the code.

- If have a car ahead and the both side lanes (left and right) is free, the overtake maneuver will be done to left when any other cars were detected  or the car will check the fastest lane ( see the code from line 275 to 303. 

- 


#### 2.4.2 - Cost Function classes

To guide the behavior criteria, it was assumed to cases to be considered:<br/>
- case 1 - car at front position.
- case 2 - car at rear position.

Below a schematic picture to show the idea.

![alt text][image8]

It will be considered that lane is free to change lane maneuver if the sum of case1 + case2 = 1:

**case 1-** It is equal to to 0.5 if the distance from the car ahead and our car is greather than the red zone distance or the speed of the car at front is greather than our car and the diference of the distance is greather than a safety distance assumed in the line 124.

**case 2-** It is equal to to 0.5 if the distance from the car rear and our car is greather than the red zone distance or the speed of the car at rear is lower than our car and the diference of the distance is greather than a safety distance assumed in the line 124.


### 2.5 - Trajectory.

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.<br/>
the code is available in [spline.h](./src/spline.h)file.

![alt text][image7]


## 3 - The result.

**simulator**
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).


click on the next picture to open the video to show the code results:

[![alt text][image9]](https://youtu.be/XsbE6UfOYy0)
