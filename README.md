# Model_predictive_control

[image1]: ./readme/_1.png "Model"
[image2]: ./readme/_2.png "Cost"

## Implementation

### The Model

We use the Kinematic model. This model neglect parameters like mass, adherence, ...
The model we will use:

![alt text][image1]

This is the state of our model:

- `x, y`: Car coordinates.
- `psi`: Car direction.
- `v`: Velocity.
- `cte`: Cross-track error.
- `epsi`: Orientation error.

Static value:

- `Lf`: distance between the center of gravity and the front wheels.

Actuators:

- `a`: Acceleration.
- `delta`: Steering angle.

This is the cost function we need to minimize in order to find `a` and `delta`:

![alt text][image2]

The following parameters have been manualy tuned to get a good trajectory during the simulation.

The part of the cost based on the reference state:
```cpp
for (int t = 0; t < N; t++) {
    fg[0] += CppAD::pow(vars[cte_start + t], 2);
    // To avoid right left consecutive steering during big turn
    fg[0] += 0.3 * CppAD::pow(vars[epsi_start + t], 2);
    fg[0] += 0.3 * CppAD::pow(vars[v_start + t] - ref_v, 2);
}
```

Minimize the use of actuators:
```cpp
for (int t = 0; t < N - 1; t++) {
    fg[0] += 50 * CppAD::pow(vars[delta_start + t], 2);
    fg[0] += CppAD::pow(vars[a_start + t], 2);
}
```

Minimize the value gap between sequential actuations:
```cpp
for (int t = 0; t < N - 2; t++) {
    fg[0] += CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
    fg[0] += CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
}
```

### Timestep Length and Elapsed Duration (N & dt)

- `N`: Number of points .
It is used to define the number of point to take in account for the polynomial fitting. The more `N` is large, the slower the program run. I tried values between 10 and 15 to get correct performance.
- `dt`: Time interval.
In my implementation, `dt` is fixed to 0.1s whitch give good results.


### Polynomial Fitting and MPC Preprocessing

#### Pipeline

- Transform waypoints into car coordinates.

The simulator send waypoints, those waypoints must be transfomated into car coordinates as follow:
```cpp
const double cos_psi = cos(-psi);
const double sin_psi = sin(-psi);
Eigen::VectorXd x_veh(n_waypts);
Eigen::VectorXd y_veh(n_waypts);
for (int i = 0; i < n_waypts; i++) {
	const double dx = ptsx[i] - px;
	const double dy = ptsy[i] - py;
	x_veh[i] = dx * cos_psi - dy * sin_psi;
	y_veh[i] = dy * cos_psi + dx * sin_psi;
}
```

- Fit the polynomial (3d order).

```cpp
auto coeffs = polyfit(x_veh, y_veh, 3);
```

- Used coefficients predicted before to update actuators.
```cpp
actuators << polyeval(coeffs, 0), -atan(coeffs[1]);
```

### Model Predictive Control with Latency

The actual state of the vehicle take in account this latency by taking 0.1s of "advance". The goal is to simulate a real delay out of the simulator.
In the following code `dt` represent the delay:
```cpp
next_state(0) = v * dt;						// x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
// y = 0, sin(0) = 0
next_state(1) = 0;						// y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
next_state(2) = - v / lf * st_angle * dt;			// psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
next_state(3) = v + epsi * dt;					// v_[t+1] = v[t] + a[t] * dt
next_state(4) = cte + v * sin(epsi) * dt;
next_state(5) = epsi + next_state(2);
```
