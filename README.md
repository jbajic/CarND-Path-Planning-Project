# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

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

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## How does it work?

Well it can definitely be improved, lets discuss what it does currently and what it could do?

The Code can be separated into a part that loads and transforms data from ego vehicle and other vehicles. After that a set of waypoints from previous path is selected to determine ego vehicles parameters and determine Frenet coordinate speed and acceleration.

After that has been done we predict and generate trajectories for every other vehicle in order to know which state we can assume. This part can be definitely improved using some sort of FSM instead we just determine to change left,right or keep lane. After that we create trajectories for every of these states, and improve on them using JMT (Jerk Minimizing Trajectory) function.

Then we use a set of cost functions to determine which of these trajectories would be the best to follow. And then we execute the best generated trajectory.

## Components

### Map
This component simply keeps all the data that is related to the map (waypoints of x,y,s,d,dx,dy coordinates)

### Traffic
This component has two main parts Vehicle (which represents ego vehicle) and OtherVehicle which represents all other vehicles that are being represented in sensor_fusion.

`Vehicle` contains logic for parsing data from received JSON and updating the available states.
`OtherVehicle` also contains logic for parsing data from received JSON and predicting the position of the vehicle for a given time.

Here are also helper functions `GetTargetForState` which for a given state (KeepLane, ChangeRight, ChangeLeft) produce target `(s, s_d, s_dd, d, d_d, d_dd)` values which are Frenet coordinates and their respective derivatives. To do that successfully it uses other helper function `GetLeadingVehicleDataForLane` which limits the creating of target coordinates to avoid hitting other vehicles in the lane for which the coordinates are designed for.

### Path Planning

This module serves to generate trajectory for a set of desired target coordinates. GIven start and end coordinates `(s, s_d, s_dd, d, d_d, d_dd)` it produces the trajectory using JMT (Jerk Minimizing Trajectory).

### Cost

Here are cost functions located currently only these are being used:
 - Collision Cost => punishes trajectory with collisions
 - Buffer Cost => prefers trajectory as apart from other vehicles as possible
 - In LaneBuffer Cost = prefers trajectory as apart from other vehicles as possible in the lane
 - EfficiencyCost = prefers bigger speeds
 - Not Middle lane Cost => prefers middle lane

## How it all fits together?

1. Load Map
2. Start communication with simulator
3. Load ego vehicle data
4. Interpolate waypoints on the map for smoother trajectory
5. Define starting point and with it the Frenet coordinates and derivatives of efo vehicle using previous path
6. Load other vehicles and predict their trajectories
7. Update ego vehicle availabe states from the data gained in sensor fusion part
8. Generate trajectories for every available state and pick the best one
