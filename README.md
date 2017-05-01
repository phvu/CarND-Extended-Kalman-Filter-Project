# Extended Kalman Filter Project Starter Code
Self-Driving Car Engineer Nanodegree Program

---

## Project Rubic

### Compiling

Project can be compiled with:

    $ mkdir -p build && cd build && cmake .. && make

### Accuracy

MSE on the provided dataset `obj_pose-laser-radar-synthetic-input.txt`:

    $ ./ExtendedKF ../data/obj_pose-laser-radar-synthetic-input.txt ../data/obj_pose-laser-radar-synthetic-output.txt
    RMSE
    0.0980163
    0.0851409
     0.418298
     0.481872
     
This satisfies the requirement RMSE <= [.11, .11, 0.52, 0.52] in the project rubic.

### Follows the Correct Algorithm

The algorithm is implemented in `FusionEKF.cpp` and `kalman_filter.cpp`:

- The first measurement is used to initialize the mean state vector of the Kalman filter. If the first measurement 
is from the lidar, initialization is trivial. If it is from the radar, I convert the polar coordinates in the measurement
to the cartesian coordinates, and use it to initialize `px` and `py` components of the state vector. The velocity
`vx`, `vy` is initialized as zeros for both case.

    The state covariance matrix `P`, transition `F` and process covariance `Q` are all initialized as Identity. 
     I tried different initialization for `P`, but identity matrix tends to give acceptable tracking results.
     Big values on the diagonal of `P` tends to make the estimation varies quickly and might go wild.

    The initialization is done between lines 47-76 of `FusionEKF.cpp`.

- The `predict` step is done between lines 90-113 of `FusionEKF.cpp`. 

    I first update `F` and `Q`, and then call `Predict()` on the Kalman filter, 
    which in turn is implemented in 2 lines of code 22-23 of `kalman_filter.cpp`.
    
    Since we assume linear motion model, the predict step is the same for both lidar and radar data.
    
- The `update` step is done between lines 125-129 of `FusionEKF.cpp`.

    If the measurement is from the radar, I first compute the Jacobian matrix, and use it as the `H` matrix
    in the update equations. Otherwise I use the initialized `H_laser` matrix.
    
    The real update functions are implemented in lines 26-48 of `kalman_filter.cpp`. Depending on the type
    of sensor, we compute the error vector `y` differently, and then call the same private function `Update_()`,
    which works for any given error vector `y` and matrices `H` and `R`.
    
### Code Efficiency

I did some refactoring to improve the readability of the project:

- Removed `H` and `R` from the `KalmanFilter` class, add them as arguments in `Update()` and `UpdateEFK()`.
 This is because `H` and `R` are different for lidar and radar, while they are only needed in the update phase.
 
- Add some utility functions into `Tools` for converting cartesian coordinates into polar coordinates, and making
sure angle measurements are between `[-PI, PI]`.

### Running the visualization tool

Here are the RMSE when running `2D Unity Visualizer` with different sensor inputs:

| Sensor        | RMSE                     |
|---------------|--------------------------|
| Lidar + Radar | [0.16, 0.14, 0.45, 0.46] |
| Lidar         | [0.15, 0.14, 0.48, 0.58] |
| Radar         | [0.21, 0.35, 0.52, 0.82] |

Radar alone is the least accurate, probably because radar is more noisy. Lidar is better, and the combination 
of lidar + radar performs the best.

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

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF path/to/input.txt path/to/output.txt`. You can find
   some sample inputs in 'data/'.
    - eg. `./ExtendedKF ../data/obj_pose-laser-radar-synthetic-input.txt`

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Generating Additional Data

This is optional!

If you'd like to generate your own radar and lidar data, see the
[utilities repo](https://github.com/udacity/CarND-Mercedes-SF-Utilities) for
Matlab scripts that can generate additional data.

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project resources page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/382ebfd6-1d55-4487-84a5-b6a5a4ba1e47)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! We'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Regardless of the IDE used, every submitted project must
still be compilable with cmake and make.
