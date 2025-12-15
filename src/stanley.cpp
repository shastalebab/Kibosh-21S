#include "main.h"  // IWYU pragma: keep

Stanley stanley({16.5, 0.0, 170.25}, {4.0, 0.15, 29.5, 30.0}, {3, 70}, 10.0);

pair<double, double> Stanley::compute() {
	if(this->stanleyPoints.size() <= 0) return {0, 0};

	// Find closest point in path
	Coordinate cur = {chassis.odom_x_get(), chassis.odom_y_get(), chassis.odom_theta_get()};
	Coordinate min = this->stanleyPoints[0];
	for(Coordinate point : this->stanleyPoints) {
		if(getDistance(cur, point) < getDistance(cur, min)) min = point;
	}

	// Wrap angles to be between 0 and 360
	cur.t = util::wrap_angle(cur.t);
	min.t = util::wrap_angle(min.t);
	if(cur.t < 0) cur.t += 360;
	if(min.t < 0) min.t += 360;

	// Identify crosstrack error, heading error, and velocity
	double crosstrack = getDistance(cur, min);
	double delta_t = cur.t - min.t;
	double velocity = (fabs(chassis.drive_velocity_left()) + fabs(chassis.drive_velocity_right())) / 2;

	// Calculate heading output and set PID targets
	double output_t = delta_t + ((atan2(crosstrack * this->ke, velocity)) * M_PI / 180);

	this->stanleyTurnPID.target_set(output_t);
	this->stanleyDrivePID.target_set(getDistance(cur, this->stanleyPoints.back()));
	this->slewLeft.iterate(chassis.drive_sensor_left());
	this->slewRight.iterate(chassis.drive_sensor_right());

	// Calculate main (left) output voltage
	double l_out =
		util::clamp(this->stanleyTurnPID.compute(chassis.odom_theta_get()), this->stanleyDrivePID.compute(getDistance(this->stanleyPoints.front(), cur)),
					-this->stanleyDrivePID.compute(getDistance(this->stanleyPoints.front(), cur)));

	// Setup ackermann turning radius and wheel velocities to calculate right output
	double radius = 9.5 / tan(output_t * 180 / M_PI);
	double v_left = getVelocity(l_out);
	double v_right = ((2 * radius * v_left) + (ROBOT_WIDTH * v_left)) / ((2 * radius) - ROBOT_WIDTH);

	// Convert right velocity into voltage
	double r_out = (15240 * v_right) / (2 * M_PI * WHEEL_DIAMETER * chassis.drive_rpm_get());

	// Scale voltage to input
	double max_out = fmax(fabs(l_out), fabs(r_out));
	double max_slew = fmax(this->slewLeft.output(), this->slewRight.output());
	if(max_out > max_slew) {
		l_out *= max_slew / max_out;
		r_out *= max_slew / max_out;
	}

	return {l_out, r_out};
}

void Stanley::drive_set_point(Coordinate point, drive_directions dir, int speed, bool slew) {
	// Set up path
	Coordinate cur = {chassis.odom_x_get(), chassis.odom_y_get(), chassis.odom_theta_get()};
	point.t = getTheta(cur, point, dir);
	this->stanleyPoints = injectPoint(cur, point, cw, speed, speed, cur.t, .5);

	// Wrap angles to be between 0 and 360
	cur.t = util::wrap_angle(cur.t);
	point.t = util::wrap_angle(point.t);
	if(cur.t < 0) cur.t += 360;
	if(point.t < 0) point.t += 360;

	// Identify crosstrack error, heading error, and velocity
	double crosstrack = getDistance(cur, point);
	double delta_t = cur.t - point.t;
	double velocity = (fabs(chassis.drive_velocity_left()) + fabs(chassis.drive_velocity_right())) / 2;

	// Calculate heading output and set PID targets
	double output_t = delta_t + ((atan2(crosstrack * this->ke, velocity)) * M_PI / 180);

	this->stanleyTurnPID.target_set(output_t);
	this->stanleyDrivePID.target_set(getDistance(cur, this->stanleyPoints.back()));

	// Restart timers and initialize PID/slew
	active = true;
	chassis.drive_mode_set(DISABLE);
	this->stanleyTurnPID.timers_reset();
	this->stanleyDrivePID.timers_reset();
	if(slew) {
		slewLeft.initialize(true, speed, chassis.drive_sensor_left() + getDistance(cur, point), chassis.drive_sensor_left());
		slewRight.initialize(true, speed, chassis.drive_sensor_right() + getDistance(cur, point), chassis.drive_sensor_right());
	}
}

void Stanley::drive_set(double distance, int speed, bool slew) {
	// Set up path
	Coordinate cur = {chassis.odom_x_get(), chassis.odom_y_get(), chassis.odom_theta_get()};
	Coordinate point = getPoint(cur, distance);
	this->stanleyPoints = injectPoint(cur, point, cw, speed, speed, cur.t, .5);

	// Wrap angles to be between 0 and 360
	cur.t = util::wrap_angle(cur.t);
	point.t = util::wrap_angle(point.t);
	if(cur.t < 0) cur.t += 360;
	if(point.t < 0) point.t += 360;

	// Identify crosstrack error, heading error, and velocity
	double crosstrack = getDistance(cur, point);
	double delta_t = cur.t - point.t;
	double velocity = (fabs(chassis.drive_velocity_left()) + fabs(chassis.drive_velocity_right())) / 2;

	// Calculate heading output and set PID targets
	double output_t = delta_t + ((atan2(crosstrack * this->ke, velocity)) * M_PI / 180);

	this->stanleyTurnPID.target_set(output_t);
	this->stanleyDrivePID.target_set(getDistance(cur, this->stanleyPoints.back()));

	// Restart timers and initialize PID/slew
	active = true;
	chassis.drive_mode_set(DISABLE);
	this->stanleyTurnPID.timers_reset();
	this->stanleyDrivePID.timers_reset();
	if(slew) {
		slewLeft.initialize(true, speed, chassis.drive_sensor_left() + distance, chassis.drive_sensor_left());
		slewRight.initialize(true, speed, chassis.drive_sensor_right() + distance, chassis.drive_sensor_right());
	}
}

void Stanley::drive_wait_until(double distance) {
	if(active == false) {
		chassis.pid_wait();
		return;
	}

	// Setup variables for wait until
	Coordinate initial = {chassis.odom_x_get(), chassis.odom_y_get(), chassis.odom_theta_get()};
	Coordinate cur = {chassis.odom_x_get(), chassis.odom_y_get()};
	double sign = util::sgn(fabs(distance) - getDistance(initial, cur));

	// Setup variables for normal exit, in edge cases
	exit_output drive_exit = RUNNING;
	exit_output heading_exit = RUNNING;

	while(sign > 0) {
		// Update position and sign
		cur = {chassis.odom_x_get(), chassis.odom_y_get()};
		sign = util::sgn(fabs(distance) - getDistance(initial, cur));

		// Update exit condition states and return if both PID loops exit
		stanleyDrivePID.velocity_sensor_secondary_set(chassis.drive_imu_accel_get());
		stanleyTurnPID.velocity_sensor_secondary_set(chassis.drive_imu_accel_get());
		if(drive_exit == RUNNING) drive_exit = stanleyDrivePID.exit_condition();
		if(heading_exit == RUNNING) heading_exit = stanleyTurnPID.exit_condition();
		if(drive_exit != RUNNING && heading_exit != RUNNING) return;

		pros::delay(10);
	}

	stanleyDrivePID.timers_reset();
	stanleyTurnPID.timers_reset();
}

void Stanley::drive_wait() {
	if(active == false) {
		chassis.pid_wait();
		return;
	}

	exit_output drive_exit = RUNNING;
	exit_output heading_exit = RUNNING;

	while(drive_exit == RUNNING || heading_exit == RUNNING) {
		//stanleyDrivePID.velocity_sensor_secondary_set(chassis.drive_imu_accel_get());
		//stanleyTurnPID.velocity_sensor_secondary_set(chassis.drive_imu_accel_get());
		//if(drive_exit == RUNNING) drive_exit = stanleyDrivePID.exit_condition();
		//if(heading_exit == RUNNING) heading_exit = stanleyTurnPID.exit_condition();
		pros::delay(10);
	}
	descore.set(true);
	active = false;
}

void stanleyTask() {
	stanley.stanleyDrivePID.exit_condition_set(70, 1.7, 180, 4, 200, 200);
	stanley.stanleyTurnPID.exit_condition_set(50, 3, 170, 9, 150, 150);

	while(true) {
		if(autonMode == STANLEY && !pros::competition::is_disabled() && stanley.active) {
			auto speeds = stanley.compute();
			chassis.drive_set(speeds.first, speeds.second);
		}
		pros::delay(10);
	}
}