# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

### Simulator. You can download the Term3 Simulator BETA which contains the Path Planning Project from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).

In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

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

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

# Path Planning Project Writeup

## Data Representation

There are several classes/structs for representing complex data in the system. They are contained in ```src/data.h```, as well as ```src/waypoint.cpp and .h```

### data.h

* **InternalState**: Internal cache to store the velocity, requested d value, etc.

* **Path**: Path as received from and communicated to the simulator

* **State**: A collection of all relevant data that defines the world's state: Path received, self and other cars.

* **Car**: Representation of the own car, as it is received from the localization module.

* **OtherCar**: Representation of other cars as perceived from sensor fusion

* **Simulating State, Self and Other Cars**: The ```State``` struct can be simulated by a given timestep.

### waypoint.h and waypoint.cpp

* **Waypoint**: Represents a single waypoint

* **Waypoints**: Contains a collection of waypoints and offers several methods for manipulating them.

## Trajectory Generation

### Quick Overview

Trajectories are generated by fitting a spline to a set of waypoints and then sampling a large number of new waypoints, with a certain lookahead, from that spline. The last bit of the old path is reused for calculating the spline, to avoid oscillations in the path.

The code is located in ```planner.cpp```, mainly in the ```generateTrajectory``` method. This method will use a lot of functionality from ```waypoint.cpp``` for manipulating and performing calculations on sets of waypoints.

### Using the old path

First, the current position and orientation are determined. This is the pose that is used as a starting point for calculating the new path. If an old path is available, the end of that path is taken as initial pose, else it's the vehicles current pose.

The last bit of the old path is concatenated with a set of waypoints that are in front of the car. This set is then used to calculate a spline.

### Calculating a spline

For calculating the spline, the points have to be transformed to "car" frame. It's written in quotes, because while we will end up getting something close to the car frame (coordinate system), what is actually done is the following:

* The first waypoint is taken as positiono.
* The direction between the first and last waypoint is taken as orientation.
* This pose is used as a reference for transforming all points.
* The waypoint class offers helpers for these operations.
* As a reference, have a look at the "Create a Spline" section in ```generateTrajectory```
* As we are now working in a different frame, resulting waypoints will have to be transformed back to world frame later.

This transformation is necessary because the spline calculation library expects the X values to be sorted (in a strict ascending order).

### Sampling from the spline

The resulting spline cannot be used directly as a command to the simulator, so waypoints have to be sampled from it. The procedure for this is the following:

* Start at the X value of the first waypoint
* In very small steps, increase X, calculate Y from the spline and add the distance travelled to S.
* As soon as we have travelled enough, S-wise, create a new waypoint.
* Rinse and repeat until a certain amount of waypoints is available.

This process makes sure th can control the distance between waypoints, no matter if we are in a curve or on a straight section of the road. This is also important when changing lanes.

### Transforming back and setting command

The only things left to do are transforming the waypoints back to world coordinates, concatenating old path and new path, and setting the complete set of waypoints as a command to the simulator.

## Speed limitation

Speed limitation is performed in ```Planner::limitVelocity```. All other cars on the road are checked for their s and d values. If they are closer than a certain threshold and in front of us, we adjust our speed to theirs. If we are too close, we multiply our speed by a certain factor to establish more distance.

## State Machine

A simple state machine is used for deciding on future actions. There is a combination of two nodes (state machine states) and three lanes:

```cpp
enum Lane {
    LEFT, CENTER, RIGHT
};

enum Node {
    KEEP_LANE,
    CHANGE_LANE
};
```

When in ```KEEP_LANE```, the car will monitor whether its speed is limited by traffic in front. If it is, it will try to find a better lane. For calculating traffic situations, the current state is simulated by a certain timestep. This timestep is determined by multiplying the expected distance necessary to change lanes by the car's current velocity. The most promising lane is chosen. If a lane change is necessary, a transition to ```CHANGE_LANE``` is carried out.

When in ```CHANGE_LANE```, the car will monitor whether the requested d value (center of the lane or close to it) is reached. Once this is the case, the state machine steps back to ```KEEP_LANE```.

The state machine is implemented in ```src/state_machine.cpp```, the state transitions happen in ```StateMachine::step```

## TODO

* missing: when the other lane is also limited, prefer it if the lane after that seems ok.

* "timestep is determined by multiplying the expected distance necessary to change lanes by the car's current velocity. " is that really a good idea? lane change time should actually be constant!

* image of plot

## Helpers

The following tools were used to ease development:

### Toolkit ```toolkit.hpp```

A collection of tools that are statically available and not specific to a certain class, including conversions, transformations, plotting and constants.

### Debugging and Testing

The ```CMakeLists.txt``` will build 3 executables, the main class for the project, a test executable, and one for debugging. The sources for the latter two executables are added to the regular sources, and are contained in debug and test folders, respectively. Especially the debugging executable was a very useful resource to avoid having to run the simulator too often.

### Plotting

I chose a header-only SVG library, in combination with a file monitor to get a handy way of plotting data. This was helpful in waypoint manipulation and trajectory generation. The library is contained in ```simple_svg_1.0.0.hpp```

### Configuration

INI Files can be used to configure parameters. By default, the ```configuration/default.ini``` file is loaded.
