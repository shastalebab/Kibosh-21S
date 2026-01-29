#include "main.h"  // IWYU pragma: keep

Stanley stanley({16.5, 0.0, 170.25}, {4.0, 0.15, 29.5, 30.0}, {3, 60}, 15.0);

double Stanley::crosstrack(Coordinate cur, Coordinate min) {
	// Check for edge cases
	if(fmod(min.t, 180) == 0) {
		// Case where cot(a) is undefined
		return getDistance({cur.x, min.y}, cur);
	} else if(fmod(min.t, 90) == 0) {
		// Case where tan(a) is undefined
		return getDistance({min.x, cur.y}, cur);
	}

	// Calculate intersection between tangent line of min passing through min and normal line of min passing through cur
	double a = min.t * M_PI / 180;
	double det = -(tan(a)) - (cos(a) / sin(a));
	double new_x = -(-(min.y - (tan(a) * min.x)) + (cur.y + ((cos(a) / sin(a)) * cur.x))) / det;
	double new_y = -((tan(a) * (cur.y + ((cos(a) / sin(a)) * cur.x))) + ((cos(a) / sin(a)) * (min.y - (tan(a) * min.x)))) / det;
	Coordinate intersection = {new_x, new_y};

	// Return the length of the line segment between the robot pose and its intersection of the tangent line
	return getDistance(intersection, cur);
}

double Stanley::xy_error(Coordinate cur, Coordinate target) {
	// Translated current x, y translated around origin
	double fakek_y = (cur.y - target.y);
	double fakek_x = (cur.x - target.x);

	// Rotate around origin
	double fake_angle = target.t * M_PI / 180;
	double fake_x = (fakek_x * cos(fake_angle)) - (fakek_y * sin(fake_angle));
	double fake_y = (fakek_y * cos(fake_angle)) + (fakek_x * sin(fake_angle));

	return fake_y;
}

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
	double ct_error = crosstrack(cur, min);
	double delta_t = cur.t - min.t;
	double velocity = (fabs(chassis.drive_velocity_left()) + fabs(chassis.drive_velocity_right())) / 2;

	// Calculate heading output and set PID targets
	double output_t = delta_t + ((atan2(ct_error * this->ke, velocity)) * (180 / M_PI));

	// Decide if we've past the target or not
	double temp_target = xy_error(cur, this->stanleyPoints.back());		   // Use this instead of distance formula to fix impossible movements
	int dir = (slewLeft.speed_max_get() < 0 ? -1 : 1);					   // If we're going backwards, add a -1
	int flipped = util::sgn(temp_target) != util::sgn(sgn_init) ? -1 : 1;  // Check if we've flipped directions to what we started

	// Compute xy PID
	xy_current_fake = fabs(xy_error(cur, min));
	if(!was_stanley_just_set)
		xy_delta_fake = fabs(xy_current_fake - xy_last_fake);
	else
		was_stanley_just_set = false;
	xy_last_fake = xy_current_fake;

	new_current_fake += xy_delta_fake * ((dir * flipped));	// Create a "current sensor value" for the PID to calculate off of
	
	this->stanleyDrivePID.compute_error(fabs(temp_target) * dir * flipped, new_current_fake);
	this->stanleyTurnPID.target_set(output_t);
	this->slewLeft.iterate(chassis.drive_sensor_left());
	this->slewRight.iterate(chassis.drive_sensor_right());

	// Calculate main (left) output voltage
	double l_out = this->stanleyTurnPID.compute(chassis.odom_theta_get()) + this->stanleyDrivePID.output;

	// Setup ackermann turning radius and wheel velocities to calculate right output
	double radius = 4.75 / tan(output_t * (M_PI / 180));
	double v_left = getVelocity(l_out);
	double v_right = ((2 * radius * v_left) + (ROBOT_WIDTH * v_left)) / ((2 * radius) - ROBOT_WIDTH);
	if(tan(output_t * (M_PI / 180)) == 0) v_right = v_left;

	// Convert right velocity into voltage
	double r_out = (15240 * v_right) / (2 * M_PI * WHEEL_DIAMETER * chassis.drive_rpm_get());

	// Scale voltage to input
	double max_out = fmax(fabs(l_out), fabs(r_out));
	double max_slew = fmax(this->slewLeft.output(), this->slewRight.output());
	if(max_out > max_slew) {
		cout << slewLeft.output() << ", " << slewRight.output() << "\n";
		l_out *= max_slew / max_out;
		r_out *= max_slew / max_out;
	}

	return {l_out, r_out};
}

void Stanley::drive_set_path(vector<Coordinate> points, drive_directions dir, int speed, bool slew) {
	// Set up path
	Coordinate cur = {chassis.odom_x_get(), chassis.odom_y_get(), chassis.odom_theta_get()};
	this->stanleyPoints = points;

	// Wrap angles to be between 0 and 360
	cur.t = util::wrap_angle(cur.t);
	points[0].t = util::wrap_angle(points[0].t);
	if(cur.t < 0) cur.t += 360;
	if(points[0].t < 0) points[0].t += 360;

	// Identify crosstrack error, heading error, and velocity
	double ct_error = crosstrack(cur, points[0]);
	double delta_t = cur.t - points[0].t;
	double velocity = (fabs(chassis.drive_velocity_left()) + fabs(chassis.drive_velocity_right())) / 2;

	// Calculate heading output and set PID targets
	double output_t = delta_t + ((atan2(ct_error * this->ke, velocity)) * M_PI / 180);

	this->stanleyTurnPID.target_set(output_t);
	this->stanleyDrivePID.target_set(getDistance(cur, this->stanleyPoints.back()));

	// Restart timers and initialize PID/slew
	active = true;
	chassis.drive_mode_set(DISABLE);
	sgn_init = util::sgn(xy_error(points[points.size() - 2], points.back()));
	this->stanleyTurnPID.timers_reset();
	this->stanleyDrivePID.timers_reset();
	this->slewLeft.initialize(slew, speed, chassis.drive_sensor_left() + getDistance(cur, points.back()), chassis.drive_sensor_left());
	this->slewRight.initialize(slew, speed, chassis.drive_sensor_right() + getDistance(cur, points.back()), chassis.drive_sensor_right());
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
	double ct_error = crosstrack(cur, point);
	double delta_t = cur.t - point.t;
	double velocity = (fabs(chassis.drive_velocity_left()) + fabs(chassis.drive_velocity_right())) / 2;

	// Calculate heading output and set PID targets
	double output_t = delta_t + ((atan2(ct_error * this->ke, velocity)) * M_PI / 180);

	this->stanleyTurnPID.target_set(output_t);
	this->stanleyDrivePID.target_set(getDistance(cur, this->stanleyPoints.back()));

	// Restart timers and initialize PID/slew
	active = true;
	chassis.drive_mode_set(DISABLE);
	sgn_init = util::sgn(xy_error(cur, point));
	this->stanleyTurnPID.timers_reset();
	this->stanleyDrivePID.timers_reset();
	this->slewLeft.initialize(slew, speed, chassis.drive_sensor_left() + getDistance(cur, point), chassis.drive_sensor_left());
	this->slewRight.initialize(slew, speed, chassis.drive_sensor_right() + getDistance(cur, point), chassis.drive_sensor_right());
}

void Stanley::drive_set(double distance, int speed, bool slew) {
	// Set up path
	Coordinate cur = {chassis.odom_x_get(), chassis.odom_y_get(), chassis.odom_theta_get()};
	Coordinate point = getPoint(cur, distance);
	this->stanleyPoints = injectPoint(cur, point, cw, speed, speed, cur.t, .5);
	for(auto point : stanleyPoints) {
		cout << "(" << point.x << ", " << point.y << ", " << point.t << ")\n";
	}
	cout << "=================\n===================\n\n";

	// Wrap angles to be between 0 and 360
	cur.t = util::wrap_angle(cur.t);
	point.t = util::wrap_angle(point.t);
	if(cur.t < 0) cur.t += 360;
	if(point.t < 0) point.t += 360;

	// Identify crosstrack error, heading error, and velocity
	double ct_error = crosstrack(cur, point);
	double delta_t = cur.t - point.t;
	double velocity = (fabs(chassis.drive_velocity_left()) + fabs(chassis.drive_velocity_right())) / 2;

	// Calculate heading output and set PID targets
	double output_t = delta_t + ((atan2(ct_error * this->ke, velocity)) * M_PI / 180);

	this->stanleyTurnPID.target_set(output_t);
	this->stanleyDrivePID.target_set(getDistance(cur, this->stanleyPoints.back()));

	// Restart timers and initialize PID/slew
	active = true;
	chassis.drive_mode_set(DISABLE);
	sgn_init = util::sgn(xy_error(cur, point));
	this->stanleyTurnPID.timers_reset();
	this->stanleyDrivePID.timers_reset();
	this->slewLeft.initialize(slew, speed, chassis.drive_sensor_left() + distance, chassis.drive_sensor_left());
	this->slewRight.initialize(slew, speed, chassis.drive_sensor_right() + distance, chassis.drive_sensor_right());
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
		stanleyDrivePID.velocity_sensor_secondary_set(chassis.drive_imu_accel_get());
		stanleyTurnPID.velocity_sensor_secondary_set(chassis.drive_imu_accel_get());
		if(drive_exit == RUNNING) drive_exit = stanleyDrivePID.exit_condition();
		if(heading_exit == RUNNING) heading_exit = stanleyTurnPID.exit_condition();
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