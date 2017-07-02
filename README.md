# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

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
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 


## Reflection

For this project I implemented a PID control in order to allow a car to drive around a race track.

**P** is the proportional term that actuates according to the Cross-Track Error (CTE). A high P-coefficient allows the car to drive the error to zero faster. 
However, it causes the car to overshoot and oscillate. The derivat ive term **D** takes into account the current error change rate, an thus reduces overshooting,
allowing the car to stay closer to the reference line. Finally, to account for errors that may cause drift, an integral term **I** is introduced. 
The integral term accumulates past errors, and drives the car in the opposite direction of the sum of past errors. 

### Parameter Tuning
P, D and I parameters were chosen by manual tuning to control the steering angle based on the CTE. First, with a constant throttle of 0.3, 
the proportional coefficient Kp was increased until I perceived too much oscilation around the reference line. Then, 
the derivative coefficient Kd was increased until the car reduced its oscillations and was able to drive more smoothly reaching the reference line.
The integral term was set to a very small value as the simulator does not seem to have any steering bias. Finally, the paremeters 
where further tuned to reduce the total squared error and the average error.

The chosen parameters are:
* Proportional Kp = 0.18
* Derivative Kd = 2.8
* Integral Ki = 0.0002

For future work, an automatic parameter optimization algorithm such as *twiddle* may be implemented to reduce the error while driving faster.

### Throttle PID
A PID controller was also implemented for generating throttle commands. This PID was manually tuned to drive safely
 trying to reduce the accumulated total squared error and average error. It tries to reach the target speed, but it takes
into account the current error and steering angle to reduce that target speed. Braking is only applied when the CTE or the steering angle is high. 
For low CTE and angles the throttle is set to 0 if the
