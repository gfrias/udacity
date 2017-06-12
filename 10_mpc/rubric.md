#Model Predictive Control (MPC)

##Implementation

1) Model

The model used in this project is the kinematic model described in class and used in the quizzes. It consists of 6 equations to update its 6 variables.

	x = x + v*cos(psi)*dt
	y = y + v*sin(psi)*dt
	psi = psi + v*delta*dt/Lf
	v = v + a*dt
	cte = cte + (v * sin(epsi) * dt)
	epsi = epsi + v * delta * dt / Lf
	
Where the car position is `(x,y)`, `psi` is its orientation angle respect to that coordinate system (`x` pointing ahead of the car and `y` to the left). The velocity is `v` and `cte` is the cross track error (distance in `y` from the waypoint polynomial) and `epsi` the difference between the ideal orientation angle and the real one.

The output variables are `a` for acceleration (between -1 and 1), accounting for actuation in the brake/throttle pedals and `delta` for the steering wheel input (from -25 degrees to 25 degrees, in radians). All units are in the `mks` system.

2) Timestep Length and Elapsed Duration (N & dt)

As discussed in class, due to the fact that the model is an approximation to the real world, it is only valid for a few moments. For N=15 and dt=0.1s good results were found (see 5. Finding hyperparameter values). It was found that shorter N made the model lose predictability and a longer one increased the computing times without better results. The dt = 0.1 was a good choice considering that the delay in the actuators was also that value.

3) Polynomial Fitting and MPC Preprocessing

The waypoints returned by the simulated consisted of 6 points so that means the path could be fitted with a polynomia with a degree up to 5. It was found empirically that with a degree greater than 3 the path became unstable and therefore 3 was chosen as good tradeoff between correctly representing the path and stability.

As for preprocessing, the model was fed into car coordinates to simplify the output of path results to the simulator and the car input. That means the ideal path was translated to that coordinate system (which was something needed anyway to display the yellow line in the simulator). This change was done using the equations:

	valx = (x_p - px) * cos(psi) + (y_p - py) * sin(psi)
	valy = (y_p - py) * cos(psi) - (x_p - px) * sin(psi)
	
Where the car has position `(px, py)` and orientation `psi` and each point from the path is `(x_p, y_p)`. See the function `toLocalCoordinates` for more details and where it is used.

4) Model Predictive Control with Latency

Once a viable solution was found for a model without latency, a simple modification was done to adapt the model. It was considered that having actuators latency implied that the car kept moving with the same actuator input for a time = delay, so in `MPC::Solve` the variables `(x, y, psi, v, cte, epsi)` where updated with the current commands `(a, delta)` for a time = delay, before even running the model (see `MPC.cpp`, lines 180-185).
Other approaches were rehearsed like setting constraints for actuator values for as many timesteps as the latency implied and then displaying values shifted in time but the former strategy turned to be more elegant with good results.

5) Cost function

The cost function used accounted for reference values for v, cte, and epsi, each with a tunneable weight, weights for absolute values on the actuators and their change (derivative) from the last timestep.
In the `MPC.h` file a struct name `Config` is defined to account for all the hyperparameters. This allowed tuning from the main file.

6) Finding hyperparameter values

Since the new simulator didn't allow `reset` commands to be sent (as in the PID controller project) each possible vector was tested by 'simulating' a car being driven around the waypoints provided in the given csv file. The sum of the cuadratic CTE was used to compare what were escencially models with different cost functions.

In order to find a good hyperparameter vector, first twiddle was used with poor results. This was due to the fact that parameters the optimization one parameter at a time wasn't good enough. Then a simple random parameters sampling was used (within the 1-100 range for each value) for 10000 iterations. This gave a good result that was later refined using the simulator with manual tuning.

A good set was found for speeds upto 20mph (showed in the first lines of `main.cpp`), it could also be set upto 25 while keeping the car on track but it was found that it caused several oscillations that'd be displaseant to any car passengers.

A video for the final result can be seen [here](https://youtu.be/hSXclKk3f48)