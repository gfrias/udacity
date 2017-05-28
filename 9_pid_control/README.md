#PID Controller Project

##Reflection

###Describe the effect each of the P, I, D components had in your implementation.

For this project, a PID controller was implemented for controlling the steering wheel angle. As an input, the **Cross-Track-Error (CTE)** is used. The output is the angle returned in the interval [-1, 1]. If the value returned is outside the interval, min/max value is explicitely set. This controller has three coefficients **(P,I,D)** with the following values: **1.65, 0.02, 5.27**.
The P value is the one proportional to the CTE. This one alone is a good start to send the car through the track. A very small correction with the I value is used to take into the account the sum of historic errors. The D value is quite high and is used to compesate errors from sudden changes, tipically used when the car is going straight and has to turn all of the sudden and change its course. It could be said that this is 'almost' a PD controller due to the small value for the Ki coefficient.

###Describe how the final hyperparameters were chosen.

The hyperparameters **Kd**, **Ki**, **Kp** where selected performing the *Twiddle algorithm* described by Sebastian in his lectures (see `twiddle()` function in `main.cpp`). It had to be reworked a bit considering that the C++ code was executed by successive callback invocations instead of a in a loop fashion.
In the code there's a `do_twiddle` parameter that can be set to switch from training to running if it's `true` or `false`.

In order to perform this tunning, the velocity was set to constant 20 by setting the throttle to the following ecuation:

newThrottle = (20.0-currentSpeed)*0.5

For each set of parameters, a constant number of iterations was done `MAX_ALLOWED_COUNT`. It was found that 600 was a good value for it. As a shortcut, if the current error was greater than the best error so far, the testing for that set was interrupted and the next one was tried by sending a `reset` command to the simulator.

Finally, once the best parameters were found, the throttle was fixed to decrease a bit based on the steering angle, and the little brakes were avoided (between -1 and 0) to make the driving a bit smoother.

[Watch video](https://youtu.be/R0rc7-JmXLA)