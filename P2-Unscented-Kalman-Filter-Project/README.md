# Unscented Kalman Filter Project Starter Code
Self-Driving Car Engineer Nanodegree Program
---
NIS Chart drawn on output new data input file (obj_pose-laser-radar-synthetic-input.txt)
![NIS Chart](/P2-Unscented-Kalman-Filter-Project/NIS_Chart.png?raw=true "NIS Chart")

##Results:
For input date file obj_pose-laser-radar-synthetic-input.txt
RMSE
0.0581829
 0.092365
 0.327213
   0.2292

For input date file sample-laser-radar-measurement-data-1.txt
RMSE
0.0768258
 0.081369
 0.588004
 0.573606

For input date file sample-laser-radar-measurement-data-2.txt
RMSE
0.193348
 0.19039
0.303672
0.496079
---

## Dependencies

* cmake >= v3.5
* make >= v4.1
* gcc/g++ >= v5.4

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./UnscentedKF path/to/input.txt path/to/output.txt`. You can find
   some sample inputs in 'data/'.
    - eg. `./UnscentedKF ../data/sample-laser-radar-measurement-data-1.txt output.txt`

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html) as much as possible.

## Generating Additional Data

This is optional!

If you'd like to generate your own radar and lidar data, see the
[utilities repo](https://github.com/udacity/CarND-Mercedes-SF-Utilities) for
Matlab scripts that can generate additional data.

## Project Instructions and Rubric

This information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/c3eb3583-17b2-4d83-abf7-d852ae1b9fff/concepts/4d0420af-0527-4c9f-a5cd-56ee0fe4f09e)
for instructions and the project rubric.
