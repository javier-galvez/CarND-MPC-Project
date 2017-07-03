# Reflection of CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## The Model. 

* Student describes their model in detail. This includes the state, actuators and update equations.

For the state I consider eight state variables where I include the two actuators:
1. x position of the car
2. y position of the car
3. orientation of the car
4. velocity of the car
5. Cross track error
6. Orientation error
7. Steering angle
8. acceleration of the vehicle

I used the kinematic model equations learned to throw the course: 

	x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
    y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
    psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
    v_[t+1] = v[t] + a[t] * dt
    cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
    epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt

I'm not used the dynamic model because the kinematic model was enough to run the car safely at 40 mph (even at 60 mph, the max vel. tried). I believe that if I need to design an F1 car, I need to consider a more complex model.


## Timestep Length and Elapsed Duration (N & dt). 

* Student discusses the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values. Additionally the student details the previous values tried.

I found out that dt depend heavily on the desired velocity chosen. A dt with that the driving performance was well on low velocity, at high velocity the actuation commands sent when the car was translated more distance. Then, more difficult to achieve good performance. 
Another consideration is when we have a close turn (or far away of the desired path), if a chosen dt*N at some desired velocity lead to a long distance (I called this the "distance of the precision") and as I fit to a 3rd order polynomial, the fitting is not going to do well as all. This has a consequence that the car goes in the opposite direction. 
From intuition, I find that the "distance of the prediction" need to be no more than the half of the arc of the closest curve.

So, for setting these hyperparameters, I first define a desired velocity of 40mph. Then, a choose some dt (I set this parameter to 0.2) and T to have a "distance of the prediction" more or less the arc of the third turn on the track. At this point, I was nearly able to drive a complete lap. So, I reduce dt (and increase T to keep the same "distance of the prediction"), until I got a reasonable performance. That final value was dt=0.05 and T= 15. 

After that, I played with the cost function, specifically penalizing the use of the actuators and the gap between sequential actuation in order to have a smooth driving.


## Polynomial Fitting and MPC Preprocessing.

* A polynomial is fitted to waypoints.
* If the student preprocesses waypoints, the vehicle state, and/or actuators prior to the MPC procedure it is described.

I find very useful to transform the waypoints from map's coordinate system to vehicle's coordinate system, making alias transformations. First a translation operation and the rotation operation. Then fit the transformed waypoints to a third order polynomial. From that, I consider the crosstrack error cte as the values of the polynomial evaluated at 0 and similar for orientation error psi. As the coefficients are in relation to the car's coordinate system I set the x and y position, as well as the orientation, in the state vector as 0.


## Model Predictive Control with Latency

* The student implements Model Predictive Control that handles a 100 millisecond latency. Student provides details on how they deal with latency.

To consider the latency, I modify the equations that define the actual actuation in MPC model. Instead of taking the actual actuator to compute the kinematic model, I consider the actuator made two times before to compute the model. Why two? because my chosen dt was 0.05 s, so the two dt represent 100 ms. I other words, I compute the kinematic model with throttle and steering command made 100 milliseconds before. 
Although this work in the simulation I believe that in the real world, I need to model the way the actuator delay. I mean, maybe the trajectory followed by the wheel (when you steer it), affect slightly the real trajectory of the vehicle, more at high velocities.