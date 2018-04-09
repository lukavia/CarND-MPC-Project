# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

The goal of this project is to create a Model Predictive Controller that drives a car safely on a simulator. The controller receives the current state of the vehicle: (x, y) position, the angle (psi), velocity, steering_angle, throttle and waypoints (arrays of x, y global coordinates) for the path that the vehicle must follow. The model must find the optimal actuators (steering angle and throttle) and send them to the simulator. It also has to account for 100ms latency for the commands to propagate to the vehicle. 

## Preprocessing

The first objective is to preprocess the received parameters so it can be passed to the optimiser. 
In order to account for the latency a kinematic bicycle model is used to calculate the state of the vehicle as it would be the moment it receives the new commands. The following formulas are used for the goal: 

	double f_px = px + v * cos(psi) * latency;
	double f_py = py + v * sin(psi) * latency;
	double f_psi = psi + v * steering_angle / Lf * latency;
	double f_v = v + throttle * latency;

The next step is to convert the global waypoints coordinates to vehicle local coordinates as this would make calculations much more easier. We do this with the future position and psi found since we want to find a solution for the moment the car responds. 
To convert the positions we use the following code: 

	for (int i = 0; i < pts_size; i++) {
		double relx = ptsx[i] - f_px;
		double rely = ptsy[i] - f_py;
		double ptsx_car = cos(0-f_psi) * relx - sin(0-f_psi) * rely;
		double ptsy_car = cos(0-f_psi) * rely + sin(0-f_psi) * relx;
		...
	}

Next we find coefficients for the polynomial from the waypoints and calculate Cross Track Error and Error for psi that will be used by the optimiser. 

The last step is to run the optimiser and return the found steering angle and throttle. We only use the first step from the optimiser solution and on the next iteration we do all the processing again since the model doesn't account for all possible variables and the real world result will be different from the solution found.

## Optimiser

The main 2 parameters that have to be chosen are Timestep Length (N) and Elapsed Duration (dt). They determine the time horizon that the optimiser will try to find the optimal path (N * dt). The bigger the steps the more computations has to be made, but the path will be more safe. The lower the dt the more accurate the prediction will be. Those have to be chosen such that the duration is long and accurate enough so the car can make safe passage but save the computation power so the model works quick enough. Be trial, error and reading on the forums I have chosen N = 10, dt = 0.2. Lower dt would make the model unsafe and 10 time steps is enough to accomplish the task. Bigger values ware tested but those seams to work best. 

### Tuning optimiser parameters. 

The Ipopt library provides tools for solving optimisation problems by giving the input parameters and their correlations and actuators with their boundaries that the solver must find the optimal solution. 
We can tune the behaviour of the parameters by giving weights to each one. 

I've defined the following variables to tune those weight: 

* w_cte - Cross Track Error
* w_epsi - Psi Error
* w_v - Velocity
* w_delta - Steering
* w_a - Throttle
* w\_delta_gap - Change in steering
* w\_a_gap - Change in throttle

In order to achieve smooth driving without sharp turns and sudden stops and accelerations I have chosen the following weights for those parameters:

	double w_cte = 2000;
	double w_epsi = 2000;
	double w_v = 1;
	double w_delta = 50;
	double w_a = 5;
	double w_delta_gap = 500;
	double w_a_gap = 10;

Those weights drive the car safely accross the track with a relative velocity of 70 mph. 
I've chosen those weight by trying different values in relation to the desired goal.