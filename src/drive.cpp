#include "main.h"  // IWYU pragma: keep

// Global variable definitions
AutonMode autonMode = BRAIN;
Coordinate currentPoint = {0, 0, 0};
vector<Coordinate> autonPath = {};
const double default_angles[7]{6.0, 11.25, 22.5, 45.0, 90.0, 135.0, 180.0};

//
// Utility functions
//

double getDistance(Coordinate point1, Coordinate point2) {
	double errorX = point2.x - point1.x;
	double errorY = point2.y - point1.y;
	return sqrt((errorX * errorX) + (errorY * errorY));
}

double getTheta(Coordinate point1, Coordinate point2, drive_directions direction) {
	auto new_direction = direction == rev ? 180 : 0;
	double errorX = point2.x - point1.x;
	double errorY = point2.y - point1.y;
	double theta = (atan2(errorX, errorY) * 180 / M_PI) + new_direction;
	theta = fmod(theta, 360);
	if(theta < 0) theta += 360;
	return theta;
}

double getVelocity(double voltage) { return (2 * M_PI * (voltage / 127 * chassis.drive_rpm_get()) * WHEEL_DIAMETER) / 120; }

double getTimeToPoint(double distance, double velocity) { return distance / velocity; }

Coordinate getPoint(Coordinate startPoint, double distance) {
	// Get the x and y error between the new point and the current point
	double errorX = distance * (sin(startPoint.t * M_PI / 180));
	double errorY = distance * (cos(startPoint.t * M_PI / 180));

	// Add the error to the start point to create the end point
	Coordinate endPoint = startPoint;
	endPoint.x += errorX;
	endPoint.y += errorY;

	return endPoint;
}

Coordinate getPoint(Coordinate startPoint, double v_left, double v_right, double time) {
	// Get the coordinate within the reference frame of the robot of the end point
	double radius = (v_right + v_left) / (v_right - v_left) * (ROBOT_WIDTH / 2);
	double theta = ((v_right - v_left) / ROBOT_WIDTH * time) + (startPoint.t * M_PI / 180);

	double relative_x = -((-radius * cos(theta) + radius) - (-radius * cos(startPoint.t * M_PI / 180) + radius));
	double relative_y = -((radius * sin(theta)) - (radius * sin(startPoint.t * M_PI / 180)));

	theta *= 180 / M_PI;
	theta = fmod(theta, 360);
	if(theta < 0) theta += 360;

	Coordinate point_relative = {relative_x, relative_y, theta};

	// Translate the point's x and y values by the start point's x and y values
	point_relative.x += startPoint.x;
	point_relative.y += startPoint.y;

	return point_relative;
}

std::vector<Coordinate> injectPoint(Coordinate startPoint, Coordinate endPoint, e_angle_behavior behavior, double left, double right, double theta,
									double lookAhead) {
	// Make sure theta is positive
	if(startPoint.t < 0) startPoint.t += 360;

	// Get wheel velocities and proper time
	double v_left = getVelocity(left);
	double v_right = getVelocity(right);
	double v_all = (v_left + v_right) / 2;
	if(v_all == 0) v_all = v_left;

	double time = abs(getTimeToPoint(lookAhead, v_all));

	std::vector<Coordinate> pointsBar;
	Coordinate newPoint = startPoint;
	double iter = 0;

	theta = fmod(theta, 360);
	if(theta < 0) theta += 360;

	if(left != KEY) {
		if(left != right) {
			// Make sure the robot travels in the correct direction
			if(left == -right)
				time *= -1;
			else if(((left > right && behavior == cw) || (right > left && behavior == ccw)))
				time *= -1;
			// Inject points along curve
			while(!(newPoint.t > theta - abs((v_right - v_left) / ROBOT_WIDTH * time * 180 / M_PI) &&
					newPoint.t < theta + abs((v_right - v_left) / ROBOT_WIDTH * time * 180 / M_PI))) {
				newPoint = getPoint(startPoint, v_left, v_right, iter);
				newPoint.left = left;
				newPoint.right = right;
				iter += time;
				pointsBar.push_back(newPoint);
			}
		} else {
			// Set direction
			if(left < 0) lookAhead *= -1;

			// Inject points along straight line
			while(getDistance(startPoint, newPoint) < getDistance(startPoint, endPoint)) {
				newPoint = getPoint(startPoint, iter);
				newPoint.t = getTheta(startPoint, endPoint, left < 0 ? rev : fwd);
				newPoint.left = left;
				newPoint.right = right;
				iter += lookAhead;
				pointsBar.push_back(newPoint);
			}
		}
	} else
		pointsBar.push_back(endPoint);

	return pointsBar;
}

std::vector<Coordinate> injectPath(std::vector<Coordinate> coordList, double lookAhead) {
	if(coordList.size() > 1) {
		std::vector<Coordinate> injectedList = {};
		for(int i = 0; i < coordList.size() - 1; i++) {
			std::vector<Coordinate> segList = injectPoint(coordList[i], coordList[i + 1], coordList[i + 1].behavior, coordList[i + 1].left,
														  coordList[i + 1].right, coordList[i + 1].t, lookAhead);
			injectedList.insert(injectedList.end(), segList.begin(), segList.end());
		}
		injectedList.push_back(coordList.back());
		return injectedList;
	}
	return coordList;
}

//
// Set position wrappers
//

void setPosition(double x, double y) { setPosition(x, y, autonMode != BRAIN ? chassis.odom_theta_get() : currentPoint.t); }

void setPosition(double x, double y, double t) {
	if(autonMode != BRAIN) {
		chassis.odom_xyt_set(x, y, t);
		pros::delay(10);
	}
	currentPoint.x = x;
	currentPoint.y = y;
	currentPoint.t = t;
	autonPath.push_back(currentPoint);
}

double getDistanceActualBack() { return (distanceSensBack.get() / 25.4) + 7.5; }

double getDistanceActualSide() { return (distanceSensSide.get() / 25.4) + 5.657; }

double getDistanceActual(double hyp, double theta) { return hyp * cos(theta * M_PI / 180); }

//
// Wait wrappers
//

void pidWait(Wait type) {
	switch(autonMode) {
		case PLAIN:
		case ODOM:
			switch(type) {
				case QUICK:
					chassis.pid_wait_quick();
					break;
				case CHAIN:
					chassis.pid_wait_quick_chain();
					break;
				default:
					chassis.pid_wait();
					break;
			}
			break;
		case STANLEY:
			stanley.drive_wait();
			break;
		default:
			break;
	}
}

void pidWaitUntil(okapi::QLength distance) {
	switch(autonMode) {
		case PLAIN:
		case ODOM:
			chassis.pid_wait_until(distance);
			break;
		case STANLEY:
			stanley.drive_wait_until(distance.convert(okapi::inch));
			break;
		default:
			break;
	}
}

void pidWaitUntil(okapi::QAngle theta) {
	switch(autonMode) {
		case PLAIN:
		case ODOM:
		case STANLEY:
			chassis.pid_wait_until(theta);
			break;
		default:
			break;
	}
}

void pidWaitUntil(Coordinate coordinate) {
	switch(autonMode) {
		case PLAIN:
		case ODOM:
			chassis.pid_wait_until({coordinate.x * okapi::inch, coordinate.y * okapi::inch});
			break;
		case STANLEY:
			stanley.drive_wait_until(getDistance({chassis.odom_x_get(), chassis.odom_y_get()}, coordinate));
			break;
		default:
			break;
	}
}

void delayMillis(int millis, bool ignore) {
	switch(autonMode) {
		case PLAIN:
		case ODOM:
		case STANLEY:
			pros::delay(millis);
			break;
		default:
			break;
	}
	if(!ignore) {
		currentPoint.left = KEY;
		currentPoint.right = millis;
		autonPath.push_back(currentPoint);
	}
}

void delayMillis(int millis) { delayMillis(millis, false); }

//
// Move to point wrappers
//

void moveThroughPoints(vector<Coordinate> points, drive_directions direction, int speed, bool slew) {
	vector<odom> points_ez = {};
	switch(autonMode) {
		case PLAIN:
			for(auto point : points) {
				turnSet(getTheta({chassis.odom_x_get(), chassis.odom_y_get(), chassis.odom_theta_get()}, point, direction), speed);
				&point != &points.back() ? pidWait(CHAIN) : pidWait(WAIT);
				driveSet((getDistance({chassis.odom_x_get(), chassis.odom_y_get(), chassis.odom_theta_get()}, point) * (direction == rev ? -1 : 1)), speed, slew);
				if(&point != &points.back()) pidWait(CHAIN);
			}
			break;
		case ODOM:
			currentPoint.t = getTheta({currentPoint.x, currentPoint.y}, points[0], direction);
			currentPoint.x = points[0].x;
			currentPoint.y = points[0].y;
			currentPoint.left = speed * (direction == fwd ? 1 : -1);
			currentPoint.right = speed * (direction == fwd ? 1 : -1);
			autonPath.push_back(currentPoint);
			for(auto point : points) {
				odom point_ez = {{point.x, point.y}, direction, speed};
				points_ez.push_back(point_ez);
			}
			chassis.pid_odom_set(points_ez, slew);
			break;
		case STANLEY:
			currentPoint.t = getTheta({currentPoint.x, currentPoint.y}, points[0], direction);
			currentPoint.x = points[0].x;
			currentPoint.y = points[0].y;
			currentPoint.left = speed * (direction == fwd ? 1 : -1);
			currentPoint.right = speed * (direction == fwd ? 1 : -1);
			autonPath.push_back(currentPoint);
			stanley.drive_set_path(injectPath(points, .5), direction, speed, slew);
			break;
		default:
			for(auto point : points) {
				turnSet(getTheta(currentPoint, point, direction), speed);
				pidWait(WAIT);
				driveSet((getDistance(currentPoint, point) * (direction == rev ? -1 : 1)), speed, slew);
			}
			break;
	}
}

void moveThroughPoints(vector<Coordinate> points, drive_directions direction, int speed) {
	bool slew_state = false;
	if(getDistance(currentPoint, points[0]) > 48) slew_state = true;
	moveThroughPoints(points, direction, speed, slew_state);
}

void moveToPoint(Coordinate newpoint, drive_directions direction, int speed, bool slew) {
	switch(autonMode) {
		case PLAIN:
			turnSet(getTheta({chassis.odom_x_get(), chassis.odom_y_get(), chassis.odom_theta_get()}, newpoint, direction), speed);
			pidWait(WAIT);
			driveSet((getDistance({chassis.odom_x_get(), chassis.odom_y_get(), chassis.odom_theta_get()}, newpoint) * (direction == rev ? -1 : 1)), speed, slew);
			break;
		case ODOM:
			currentPoint.t = getTheta({currentPoint.x, currentPoint.y}, newpoint, direction);
			currentPoint.x = newpoint.x;
			currentPoint.y = newpoint.y;
			currentPoint.left = speed * (direction == fwd ? 1 : -1);
			currentPoint.right = speed * (direction == fwd ? 1 : -1);
			autonPath.push_back(currentPoint);
			chassis.pid_odom_set({{newpoint.x, newpoint.y}, direction, speed}, slew);
			break;
		case STANLEY:
			currentPoint.t = getTheta({currentPoint.x, currentPoint.y}, newpoint, direction);
			currentPoint.x = newpoint.x;
			currentPoint.y = newpoint.y;
			currentPoint.left = speed * (direction == fwd ? 1 : -1);
			currentPoint.right = speed * (direction == fwd ? 1 : -1);
			autonPath.push_back(currentPoint);
			stanley.drive_set_point(newpoint, direction, speed, slew);
			break;
		default:
			turnSet(getTheta(currentPoint, newpoint, direction), speed);
			pidWait(WAIT);
			driveSet((getDistance(currentPoint, newpoint) * (direction == rev ? -1 : 1)), speed, slew);
			break;
	}
}

void moveToPoint(Coordinate newpoint, drive_directions direction, int speed) {
	bool slew_state = false;
	if(getDistance(currentPoint, newpoint) > 48) slew_state = true;
	moveToPoint(newpoint, direction, speed, slew_state);
}

//
// Drive set wrappers
//

void driveSet(double distance, int speed, bool slew) {
	drive_directions direction = distance < 0 ? rev : fwd;
	switch(autonMode) {
		case PLAIN:
			chassis.pid_drive_set(distance * okapi::inch, speed, false);
			currentPoint.x = chassis.odom_x_get();
			currentPoint.y = chassis.odom_y_get();
			break;
		case ODOM:
			chassis.pid_odom_set(distance * okapi::inch, speed, false);
			currentPoint.x = chassis.odom_x_get();
			currentPoint.y = chassis.odom_y_get();
			break;
		case STANLEY:
			stanley.drive_set(distance, speed, slew);
			currentPoint.x = chassis.odom_x_get();
			currentPoint.y = chassis.odom_y_get();
			break;
		default:
			break;
	}
	currentPoint = getPoint(currentPoint, distance);
	currentPoint.left = speed * (direction == fwd ? 1 : -1);
	currentPoint.right = speed * (direction == fwd ? 1 : -1);
	autonPath.push_back(currentPoint);
}

void driveSet(double distance, int speed) {
	bool slew = abs(distance) > 48 ? true : false;
	driveSet(distance, speed, slew);
}

void driveSmartSet(double distance, int speed) {
	Coordinate endpoint = getPoint(currentPoint, distance);
	double newdist = getDistance({chassis.odom_x_get(), chassis.odom_y_get()}, endpoint);
	if(autonMode == BRAIN) newdist = distance;
	driveSet(newdist, speed);
}

//
// Turn set wrappers
//

void turnConstantsSet(double theta, double current) {
	double target = util::turn_shortest(theta, current);
	double error = fabs(current - target);

	int iter = 0;
	for(auto angle : default_angles) {
		if(angle > error) break;
		iter++;
	}

	switch(iter) {
		case 0:
			chassis.pid_turn_constants_set(4.0, 0.15, 29.5, 30.0);
			break;
		case 1:
			chassis.pid_turn_constants_set(4.0, 0.15, 29.5, 30.0);
			break;
		case 2:
			chassis.pid_turn_constants_set(4.0, 0.15, 29.5, 30.0);
			break;
		case 3:
			chassis.pid_turn_constants_set(4.0, 0.15, 29.5, 30.0);
			break;
		case 4:
			chassis.pid_turn_constants_set(4.0, 0.15, 29.5, 30.0);
			break;
		case 5:
			chassis.pid_turn_constants_set(4.0, 0.15, 29.5, 30.0);
			break;
		default:
			chassis.pid_turn_constants_set(4.0, 0.15, 29.5, 30.0);
			break;
	}
}

void turnSet(double theta, int speed, e_angle_behavior behavior) {
	switch(autonMode) {
		case PLAIN:
		case ODOM:
		case STANLEY:
			turnConstantsSet(theta, chassis.drive_imu_get());
			chassis.pid_turn_set(theta * okapi::degree, speed, behavior);
			break;
		default:
			break;
	}

	if(behavior == shortest) behavior = (util::turn_shortest(theta, currentPoint.t) < currentPoint.t) ? ccw : cw;

	if(behavior == ccw) speed *= -1;

	currentPoint.t = theta;
	currentPoint.left = speed;
	currentPoint.right = -speed;
	currentPoint.behavior = behavior;
	autonPath.push_back(currentPoint);
}

void turnSet(double theta, int speed) {
	e_angle_behavior behavior = (util::turn_shortest(theta, currentPoint.t) < currentPoint.t) ? ccw : cw;
	switch(autonMode) {
		case PLAIN:
		case ODOM:
		case STANLEY:
			behavior = (util::turn_shortest(theta, chassis.odom_theta_get()) < chassis.odom_theta_get()) ? ccw : cw;
			break;
		default:
			break;
	}
	turnSet(theta, speed, behavior);
}

void turnSet(Coordinate point, drive_directions direction, int speed, e_angle_behavior behavior) {
	double theta = getTheta(currentPoint, point, direction);
	switch(autonMode) {
		case PLAIN:
		case ODOM:
		case STANLEY:
			theta = getTheta({chassis.odom_x_get(), chassis.odom_y_get(), chassis.odom_theta_get()}, point, direction);
			break;
		default:
			break;
	}
	turnSet(theta, speed, behavior);
}

void turnSet(Coordinate point, drive_directions direction, int speed) {
	double theta = getTheta(currentPoint, point, direction);
	e_angle_behavior behavior = (util::turn_shortest(theta, currentPoint.t) < currentPoint.t) ? ccw : cw;
	switch(autonMode) {
		case PLAIN:
		case ODOM:
		case STANLEY:
			behavior = (util::turn_shortest(theta, chassis.odom_theta_get()) < chassis.odom_theta_get()) ? ccw : cw;
			theta = getTheta({chassis.odom_x_get(), chassis.odom_y_get(), chassis.odom_theta_get()}, point, direction);
			break;
		default:
			break;
	}
	turnSet(theta, speed, behavior);
}

void turnSetRelative(double theta, int speed, e_angle_behavior behavior) {
	switch(autonMode) {
		case PLAIN:
		case ODOM:
		case STANLEY:
			theta += chassis.odom_theta_get();
			break;
		default:
			theta += currentPoint.t;
			break;
	}
	fmod(theta, 360);
	if(theta < 0) theta += 360;
	turnSet(theta, speed, behavior);
}

void turnSetRelative(double theta, int speed) {
	e_angle_behavior behavior = (util::turn_shortest(theta, currentPoint.t) < 0) ? ccw : cw;
	switch(autonMode) {
		case PLAIN:
		case ODOM:
		case STANLEY:
			behavior = (util::turn_shortest(theta, chassis.odom_theta_get()) < 0) ? ccw : cw;
			theta += chassis.odom_theta_get();
			break;
		default:
			theta += currentPoint.t;
			break;
	}
	fmod(theta, 360);
	if(theta < 0) theta += 360;
	turnSet(theta, speed, behavior);
}

//
// Swing set wrappers
//

void swingSet(e_swing side, double theta, double main, double opp, e_angle_behavior behavior) {
	switch(autonMode) {
		case PLAIN:
		case ODOM:
		case STANLEY:
			chassis.pid_swing_set(side, theta * okapi::degree, main, opp, behavior);
			break;
		default:
			break;
	}

	// Convert main/opposite voltages to left/right voltages
	double right = side == RIGHT_SWING ? main : opp;
	double left = side == LEFT_SWING ? main : opp;
	right = util::clamp(right, 117);
	left = util::clamp(left, 117);

	// Convert voltage to velocity
	double v_left = getVelocity(left);
	double v_right = getVelocity(right);
	double v_all = (v_left + v_right) / 2;

	// Get radius and arc length
	double new_t = theta - currentPoint.t;
	fmod(new_t, 360);
	if(new_t < 0) new_t += 360;
	double radius = (v_right + v_left) / (v_right - v_left) * (ROBOT_WIDTH / 2);
	double arcLength = radius * new_t * M_PI / 180;

	currentPoint = getPoint(currentPoint, v_left, v_right, getTimeToPoint(arcLength, v_all));

	currentPoint.left = left;
	currentPoint.right = right;
	currentPoint.behavior = behavior;
	autonPath.push_back(currentPoint);
}

void swingSet(ez::e_swing side, double theta, double main, ez::e_angle_behavior behavior) { swingSet(side, theta, main, 0, behavior); }

void swingSet(ez::e_swing side, double theta, double main, double opp) {
	e_angle_behavior behavior = (util::turn_shortest(theta, currentPoint.t) < 0) ? ccw : cw;
	switch(autonMode) {
		case PLAIN:
		case ODOM:
		case STANLEY:
			behavior = (util::turn_shortest(theta, chassis.odom_theta_get()) < 0) ? ccw : cw;
			break;
		default:
			break;
	}
	swingSet(side, theta, main, opp, behavior);
}

void swingSet(ez::e_swing side, double theta, double main) {
	e_angle_behavior behavior = (util::turn_shortest(theta, currentPoint.t) < 0) ? ccw : cw;
	switch(autonMode) {
		case PLAIN:
		case ODOM:
		case STANLEY:
			behavior = (util::turn_shortest(theta, chassis.odom_theta_get()) < 0) ? ccw : cw;
			break;
		default:
			break;
	}
	swingSet(side, theta, main, 0, behavior);
}

//
// Print path data
//

void getPath() {
	cout << "===========================================" << endl;
	for(auto point : autonPath) {
		cout << "(" << point.x << ", " << point.y << ")" << endl;
	}
	cout << "===========================================" << endl;
}

void getPathInjected() {
	auto injected = injectPath(autonPath, 2);
	cout << "===========================================" << endl;
	for(auto point : injected) {
		cout << "(" << point.x << ", " << point.y << ")" << endl;
	}
	cout << "===========================================" << endl;
}