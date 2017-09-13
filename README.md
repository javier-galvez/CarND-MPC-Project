# Model Predictive Control (MPC)

Project for the Self-Driving Car Engineer Nanodegree Program

---

## Overview

In this project I implemented Model Predictive Control to drive the car around the track. This time, however you're not given the cross track error like in the [PID project](https://github.com/jjaviergalvez/CarND-PID-Control-Project), I had to calculate that myself! Additionally, there's a 100 millisecond latency between actuations commands on top of the connection latency.


## Dependencies

* cmake >= 3.5
    * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
    * Linux: make is installed by default on most Linux distros
    * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
    * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
    * Linux: gcc / g++ is installed by default on most Linux distros
    * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
    * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
    * Run either `install-mac.sh` or `install-ubuntu.sh`.
    * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Fortran Compiler
    * Mac: `brew install gcc` (might not be required)
    * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
    * Mac: `brew install ipopt`
    * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
    * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
    * Mac: `brew install cppad`
    * Linux `sudo apt-get install cppad` or equivalent.
    * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.
* For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.


## Reflection 

### The Model. 

For the state I consider eight state variables where I include the two actuators:
1. x position of the car
2. y position of the car
3. orientation of the car
4. velocity of the car
5. Cross track error
6. Orientation error
7. Steering angle
8. acceleration of the vehicle

I used the kinematic model equations learned throw the course: 
```
  x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
  y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
  psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
  v_[t+1] = v[t] + a[t] * dt
  cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
  epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
```

I'm not used the dynamic model because the kinematic model was enough to run the car safely at 40 mph (even at 60 mph, the max vel. tried). I believe that if I need to design an F1 car, I need to consider a more complex model.


### Timestep Length and Elapsed Duration (N & dt). 

The reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values. Additionally I provide details the previous values tried.

---

I found out that dt depend heavily on the desired velocity chosen. A dt with that the driving performance was well on low velocity, at high velocity the actuation commands sent when the car was translated more distance. Then, more difficult to achieve good performance. 
Another consideration is when we have a close turn (or far away of the desired path), if a chosen dt*N at some desired velocity lead to a long distance (I called this the "distance of the precision") and as I fit to a 3rd order polynomial, the fitting is not going to do well as all. This has a consequence that the car goes in the opposite direction. 
From intuition, I find that the "distance of the prediction" need to be no more than the half of the arc of the closest curve.

So, for setting these hyperparameters, I first define a desired velocity of 40mph. Then, a choose some dt (I set this parameter to 0.2) and T to have a "distance of the prediction" more or less the arc of the third turn on the track. At this point, I was nearly able to drive a complete lap. So, I reduce dt (and increase T to keep the same "distance of the prediction"), until I got a reasonable performance. That final value was dt=0.05 and T= 15. 

After that, I played with the cost function, specifically penalizing the use of the actuators and the gap between sequential actuation in order to have a smooth driving.


### Polynomial Fitting and MPC Preprocessing.

I find very useful to transform the waypoints from map's coordinate system to vehicle's coordinate system, making alias transformations. First a translation operation and the rotation operation. Then fit the transformed waypoints to a third order polynomial. From that, I consider the crosstrack error cte as the values of the polynomial evaluated at 0 and similar for orientation error psi. As the coefficients are in relation to the car's coordinate system I set the x and y position, as well as the orientation, in the state vector as 0.


### Model Predictive Control with Latency

To consider the latency, I modify the equations that define the actual actuation in MPC model. Instead of taking the actual actuator to compute the kinematic model, I consider the actuator made two times before to compute the model. Why two? because my chosen dt was 0.05 s, so the two dt represent 100 ms. I other words, I compute the kinematic model with throttle and steering command made 100 milliseconds before. 
Although this work in the simulation I believe that in the real world, I need to model the way the actuator delay. I mean, maybe the trajectory followed by the wheel (when you steer it), affect slightly the real trajectory of the vehicle, more at high velocities.
