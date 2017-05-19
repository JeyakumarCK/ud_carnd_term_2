# CarND Term-2, Project-4: PID Controller
---
## Reflections

My Project code is available in this folder of github repository.

[https://github.com/JeyakumarCK/ud_carnd_term_2/tree/master/P4-CarND-PID-Control-Project](https://github.com/JeyakumarCK/ud_carnd_term_2/tree/master/P4-CarND-PID-Control-Project)

Here is the output video of this code [https://youtu.be/Elb7Ao7EODY](https://youtu.be/Elb7Ao7EODY)

<iframe width="560" height="315" src="https://www.youtube.com/embed/Elb7Ao7EODY" frameborder="0" allowfullscreen></iframe>

## The effect of P,I & D components

### P (Proportional) Controller: This controller output is proportional to the error (CTE), but in the opposite direction.  The coefficient Kp determines the magnitude of the effect of this controller.  Having bigger Kp makes it output bigger in each step, making the output value (steer value) changed drastically with in few steps.  In reality this drastic variation will render as the vehicle swerving left and right too fast losing the steer control.  In the below screen shot (that was driven with vehich high Kp value of 1.07), with in 5 steps, the steering value is increasing from -0.8 to -0.4, which is 50% change in the direction.

High_P_2.png

### I (Integral) Controller: This controller is used to correct any systemic bias that cannot be eliminated.  For example, there is a strong wind and because of that the vehicle is getting drifted in one side.  Or the wheel is not aligned properly and dragging the vehicle in a particular direction.  Such bias can be eliminated by using the cumulative error over a period of time, factored by a Ki coefficient.  Most of the time, it will be almost zero.

### D (Derivative) Controller: This controller outputs the correction based on the rate of change of errors (current & previous CTE) with Kd coefficient.  Basically it smoothens the output either by enhancing or dampening the effect from its previous effect.  Higher Kd makes the vehicle very sensitive and lower Kd will be make the vehicle non-reactive and makes it to take long time to reach the desired output value.

## Final hyperparameters

I chose the hyperparameters manually by trying Kp coefficient first while others were zero. Based on the effects of P, adjusted the Kd coefficient based on the P effect.  Conveniently had Ki value to zero all the time.

Tried writing the Twiddle logic to play around with hyper parameters, it helped to learn the effects in various combinations. Finally with manual and twiddle, arrived this coefficients (Kp = 0.7, Ki = 0.001, Kd = 1.7) that is able to drive the track completely. 

Apart from tweaking these coefficients, the throttle value also adjusted and made it slow.  Once I got the working coefficients, the throttle value brought back to 0.3.  I did not tried using another PID controller for throttle value.


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
* [uWebSockets](https://github.com/uWebSockets/uWebSockets) == 0.13, but the master branch will probably work just fine
  * Follow the instructions in the [uWebSockets README](https://github.com/uWebSockets/uWebSockets/blob/master/README.md) to get setup for your platform. You can download the zip of the appropriate version from the [releases page](https://github.com/uWebSockets/uWebSockets/releases). Here's a link to the [v0.13 zip](https://github.com/uWebSockets/uWebSockets/archive/v0.13.0.zip).
  * If you run OSX and have homebrew installed you can just run the ./install-mac.sh script to install this
* Simulator. You can download these from the [project intro page](https://github.com/udacity/CarND-PID-Control-Project/releases) in the classroom.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

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

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
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
