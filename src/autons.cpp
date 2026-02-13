#include "main.h"  // IWYU pragma: keep

// Commonly used speed constants
const int DRIVE_SPEED = 127;
const int TURN_SPEED = 110;
const int SWING_SPEED = 127;

///
// Constants
///
void default_constants() {
	// P, I, D, and Start I
	chassis.pid_drive_constants_set(16.5, 0.4, 175.25);	 // Straight driving constants, used for odom and non odom motions
	chassis.pid_heading_constants_set(4.75, 1.55,
									  54.0);  // Holds the robot straight while going forward without odom
	chassis.pid_turn_constants_set(4.0, 0.15, 29.5,
								   30.0);					   // Turn in place constants
	chassis.pid_swing_constants_set(6.75, 0.0, 57.75);		   // Swing constants
	chassis.pid_odom_angular_constants_set(6.25, 0.1, 78.5);   // Angular control for odom motions
	chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);  // Angular control for boomerang motions
	chassis.pid_drive_constants_get();

	// Exit conditions
	chassis.pid_turn_exit_condition_set(40_ms, 3_deg, 120_ms, 9_deg, 2000_ms, 2000_ms, false);
	chassis.pid_swing_exit_condition_set(40_ms, 3_deg, 110_ms, 7_deg, 2000_ms, 2000_ms, false);
	chassis.pid_drive_exit_condition_set(50_ms, 1.7_in, 150_ms, 4_in, 200_ms, 200_ms, false);
	chassis.pid_odom_turn_exit_condition_set(60_ms, 2_deg, 120_ms, 6_deg, 2000_ms, 2000_ms, false);
	chassis.pid_odom_drive_exit_condition_set(50_ms, 1.7_in, 130_ms, 3.4_in, 2000_ms, 2000_ms, false);
	chassis.pid_turn_chain_constant_set(4_deg);
	chassis.pid_swing_chain_constant_set(5_deg);
	chassis.pid_drive_chain_constant_set(4_in);
	chassis.drive_imu_scaler_set(1.00005);

	// Slew constants
	chassis.slew_turn_constants_set(3_deg, 70);
	chassis.slew_drive_constants_set(3_in, 70);
	chassis.slew_swing_constants_set(3_in, 80);

	// The amount that turns are prioritized over driving in odom motions
	// - if you have tracking wheels, you can run this higher.  1.0 is the max
	chassis.odom_turn_bias_set(1.0);

	chassis.odom_look_ahead_set(16_in);			 // This is how far ahead in the path the robot looks at
	chassis.odom_boomerang_distance_set(16_in);	 // This sets the maximum distance away from target that the carrot
												 // point can be
	chassis.odom_boomerang_dlead_set(0.625);	 // This handles how aggressive the end of boomerang motions are

	chassis.odom_path_smooth_constants_set(0.9, 0.02, 0.0001);

	chassis.pid_angle_behavior_set(shortest);  // Changes the default behavior for turning, this defaults it to
											   // the shortest path there
}

//
// PID TUNING/TESTING ROUTINES
//

void drive_test(int inches) {
	chassis.pid_drive_set(inches, DRIVE_SPEED);
	chassis.pid_wait();
}

void turn_test(int degrees) {
	chassis.pid_turn_set(degrees, TURN_SPEED, raw);
	chassis.pid_wait();
}

void swing_test(int degrees) {
	chassis.pid_swing_set(LEFT_SWING, degrees, SWING_SPEED, raw);
	chassis.pid_wait();
}

void heading_test(int degrees) {
	chassis.pid_drive_set(12, DRIVE_SPEED);
	chassis.headingPID.target_set(chassis.drive_imu_get() + degrees);
}

void odom_test(int degrees) {
	chassis.odom_xyt_set(0, 0, degrees);
	chassis.pid_odom_set({{0_in, 24_in}, fwd, DRIVE_SPEED});
}

void constants_test() {
	// setPosition(59.64, 20.33, -25);
	moveToPoint({24, 24}, fwd, 127);
	// setPosition(7.21875, 9.25, 180);
}

//
// MODULES (used within autons)
//

void wall_reset(double in, int speed, bool slew) {
	if(autonMode == BRAIN || speed == 0) return;

	if(slew) {
		chassis.pid_drive_set(util::sgn(speed) * 100, abs(speed), true);
		pros::delay(2000);
	}

	chassis.drive_set(speed, speed);

	int it = 0;
	while(it < 10) {
		if(getDistanceActualBack() <= in)
			it++;
		else
			it = 0;
		pros::delay(10);
	}
	chassis.drive_set(0, 0);
	pros::delay(200);
}

void barrier_reset(int speed) {
	if(autonMode == BRAIN || speed == 0) return;

	chassis.pid_drive_set(util::sgn(speed) * 200, abs(speed));

	int it = 0;
	while(it < 10) {
		if(chassis.interfered == true)
			it++;
		else
			it = 0;
		pros::delay(10);
	}
	chassis.drive_set(0, 0);
	pros::delay(200);
}

void matchload() {
	driveSet(-35, 90);
	if(autonMode == BRAIN) return;
	setScraper(true);
	setRedirect(false);
	setAligner(false);
	setSortPrime(PRIMED);
	int it = 0;
	while(true) {
		if(it > 250 || getSortPrime() == UNPRIMED) break;
		it++;
		pros::delay(10);
	}
}

//
// RIGHT AUTONS
//

void vexu_scrim() {
	setPosition(-17.38, 20.92, 328);
	// Collect two blocks under goal
	setRedirect(true);
	setIntake(127, true);
	driveSet(38, DRIVE_SPEED);
	pidWait(CHAIN);
	driveSet(16, 70, false);
	pidWait(WAIT);
	// Collect cluster of three blocks with back intake and score on long goal
	moveThroughPoints({{-16, 44}, {-46.75, 36}}, rev, DRIVE_SPEED);
	pidWait(CHAIN);
	setRedirect(false);
	setAligner(true);
	turnSet(0, TURN_SPEED);
	pidWait(CHAIN);
	moveToPoint({-48.5, 42}, fwd, DRIVE_SPEED);
	delayMillis(100, true);
	setIntake(127, false);
	delayMillis(550, true);
	chassis.drive_set(0, 0);
	delayMillis(600);
	// Matchload
	setAligner(false);
	setIntake(127, true);
	moveToPoint({-48.5, 10}, rev, 70);
	delayMillis(300, true);
	setScraper(true);
	if(autonMode != BRAIN) {
		delayMillis(900);
		chassis.drive_set(0, 0);
		delayMillis(300);
		chassis.drive_set(-20, -20);
		setPosition(getDistanceActual(getDistanceActualSide(), chassis.odom_theta_get()) - 72, 13.5);
	} else
		delayMillis(900);
	// Score long goal again
	moveToPoint({-48, 42}, fwd, DRIVE_SPEED);
	delayMillis(200);
	setScraper(false);
	setAligner(true);
	if(autonMode != BRAIN) {
		delayMillis(300);
		setIntake(127, false);
		delayMillis(450);
	} else
		pidWait(WAIT);
	chassis.drive_set(0, 0);
	delayMillis(900);
	// Matchload again
	setAligner(false);
	moveToPoint({-48, 10}, rev, 70);
	delayMillis(300, true);
	setScraper(true);
	if(autonMode != BRAIN) {
		delayMillis(900);
		chassis.drive_set(0, 0);
		delayMillis(300);
		chassis.drive_set(-20, -20);
		delayMillis(300);
		chassis.drive_set(0, 0);
		delayMillis(300);
		chassis.drive_set(-20, -20);
		delayMillis(300);
		setIntake(127, true);
		chassis.drive_set(0, 0);
		delayMillis(300);
		chassis.drive_set(-20, -20);
		setPosition(getDistanceActual(24, chassis.odom_theta_get()) - 72, 13.5);
	} else
		delayMillis(900);
	// Score middle goal
	moveThroughPoints({{autonMode == BRAIN ? -48 : chassis.odom_x_get(), 28}, {-28, 44}, {-15.5, 58.5}}, fwd, DRIVE_SPEED);
	delayMillis(400);
	setScraper(false);
	setAligner(true);
	if(autonMode == ODOM)
		chassis.pid_wait_until_index(1);
	else
		delayMillis(1300, true);
	setRedirect(true);
	setIntake(90, 127, false);
	pidWait(WAIT);
	delayMillis(1200);
	driveSet(-2, DRIVE_SPEED);
	pidWait(CHAIN);
	driveSet(4, DRIVE_SPEED);
	pidWait(CHAIN);
	// Wing blocks in long goal to center
	moveToPoint({-32, 38}, rev, DRIVE_SPEED);
	setIntake(127, true);
	setRedirect(false);
	setAligner(false);
	pidWait(CHAIN);
	turnSet(180, TURN_SPEED);
	pidWait(CHAIN);
	setWing(true);
	setDescore(false);
	driveSet(-27, 85, false);
	pidWait(WAIT);
	// Travel to other matchloader and matchload
	driveSet(27, DRIVE_SPEED);
	pidWait(CHAIN);
	setWing(false);
	setDescore(true);
	moveToPoint({43.5, 24}, rev, DRIVE_SPEED);
	pidWait(CHAIN);
	turnSet(0, TURN_SPEED);
	setRedirect(false);
	setScraper(true);
	pidWait(CHAIN);
	moveToPoint({50, 10}, rev, 70);
	if(autonMode != BRAIN) {
		delayMillis(900);
		chassis.drive_set(0, 0);
		delayMillis(300);
		chassis.drive_set(-20, -20);
		setPosition(72 - getDistanceActual(24, chassis.odom_theta_get()), 13.5);
	} else
		delayMillis(900);
	// Score in long goal
	moveToPoint({50, 48.5}, fwd, DRIVE_SPEED);
	delayMillis(200);
	setScraper(false);
	setAligner(true);
	if(autonMode != BRAIN) {
		delayMillis(300);
		setIntake(127, false);
		delayMillis(450);
	} else
		pidWait(WAIT);
	chassis.drive_set(0, 0);
	delayMillis(900);
	// Matchload
	setAligner(false);
	moveToPoint({51, 10}, rev, 70);
	delayMillis(300, true);
	setScraper(true);
	if(autonMode != BRAIN) {
		delayMillis(900);
		chassis.drive_set(0, 0);
		delayMillis(300);
		chassis.drive_set(-20, -20);
		delayMillis(300);
		chassis.drive_set(0, 0);
		delayMillis(300);
		chassis.drive_set(-20, -20);
		delayMillis(300);
		setIntake(127, true);
		chassis.drive_set(0, 0);
		delayMillis(300);
		chassis.drive_set(-20, -20);
		setPosition(72 - getDistanceActual(24, chassis.odom_theta_get()), 13.5);
	} else
		delayMillis(900);
	// Score on long goal
	moveToPoint({50, 48.5}, fwd, DRIVE_SPEED);
	delayMillis(200);
	setScraper(false);
	setAligner(true);
	if(autonMode != BRAIN) {
		delayMillis(300);
		setIntake(127, false);
		delayMillis(450);
	} else
		pidWait(WAIT);
	chassis.drive_set(0, 0);
	delayMillis(1200);
	// Push blocks into center with wing
	setIntake(127, true);
	swingSet(LEFT_SWING, 175, DRIVE_SPEED, 3, ccw);
	pidWait(CHAIN);
	setWing(true);
	setDescore(false);
	driveSet(-24, 75, false);
	pidWait(CHAIN);
	turnSet(-135, 50);
}

void right_elims() {}

void right_split() {
	setPosition(17.38, 20.92, -328);
	// Collect two blocks under goal
	setRedirect(true);
	setIntake(127, true);
	driveSet(38, DRIVE_SPEED);
	pidWait(CHAIN);
	driveSet(16, 70, false);
	pidWait(WAIT);
	// Collect cluster of three blocks with back intake and score on long goal
	moveThroughPoints({{16, 44}, {46.25, 36}}, rev, DRIVE_SPEED);
	delayMillis(650, true);
	setScraper(true);
	delayMillis(300, true);
	setScraper(false);
	pidWait(CHAIN);
	setRedirect(false);
	setAligner(true);
	turnSet(0, TURN_SPEED);
	pidWait(CHAIN);
	moveToPoint({49.75, 42}, fwd, DRIVE_SPEED);
	delayMillis(100, true);
	setIntake(127, false);
	delayMillis(550, true);
	chassis.drive_set(0, 0);
	delayMillis(1500);
	// Matchload
	setAligner(false);
	setIntake(127, true);
	moveToPoint({49.75, 10}, rev, 70);
	delayMillis(300, true);
	setScraper(true);
	if(autonMode != BRAIN) {
		delayMillis(900);
		chassis.drive_set(0, 0);
		delayMillis(300);
		chassis.drive_set(-20, -20);
		setPosition(72 - getDistanceActual(24, chassis.odom_theta_get()), 13.5);
	} else
		delayMillis(900);
	// Score middle goal
	moveThroughPoints({{autonMode == BRAIN ? 48 : chassis.odom_x_get(), 23}, {28, 44}, {11.75, 60.25}}, fwd, DRIVE_SPEED);
	delayMillis(200, true);
	setScraper(false);
	if(autonMode == ODOM)
		chassis.pid_wait_until_index(1);
	else
		delayMillis(1300, true);
	setAligner(true);
	pidWait(WAIT);
	setIntake(-127, false);
	delayMillis(1200);
	setAligner(false);
	// Wing blocks in long goal to center
	moveToPoint({34, 38}, rev, DRIVE_SPEED);
	setIntake(127, true);
	setRedirect(true);
	pidWait(CHAIN);
	turnSet(0, TURN_SPEED);
	pidWait(CHAIN);
	setWing(true);
	setDescore(false);
	driveSet(27, 85, false);
	pidWait(CHAIN);
	turnSet(-45, 50);
}

void right_greed() {}

void right_rush() {}

void right_superrush() {}

void right_awp() {
	setPosition(72, 24.75, -90);
	// Push other bot and grab preload
	driveSet(4, DRIVE_SPEED);
	setIntake(127, true);
	pidWait(WAIT);
	// Matchload and score
	moveToPoint({116, 24}, rev, DRIVE_SPEED);
	pidWait(CHAIN);
	turnSet(0, TURN_SPEED);
	pidWait(CHAIN);
	setScraper(true);
	// Intake blocks from loader
	driveSet(-13, 90);
	if(autonMode != BRAIN) {
		delayMillis(600);
		chassis.drive_set(0, 0);
		delayMillis(250);
		setPosition(120, 13.25);
	}
	// Score on long goal
	moveToPoint({120, 40}, fwd, DRIVE_SPEED);
	delayMillis(200);
	setScraper(false);
	setAligner(true);
	if(autonMode != BRAIN) {
		delayMillis(50);
		setIntake(127, false);
		delayMillis(800);
	} else
		pidWait(WAIT);
	// Cross field and score on middle goal
	delayMillis(450);
	setIntake(127, true);
	swingSet(ez::RIGHT_SWING, 135, SWING_SPEED);
	delayMillis(300, true);
	moveThroughPoints({{96, 51}, {53, 51}, {27.25, 36}}, rev, DRIVE_SPEED);
	pidWait(CHAIN);
	turnSet(0, TURN_SPEED);
	pidWait(CHAIN);
	moveToPoint({23.5, 42}, fwd, DRIVE_SPEED);
	delayMillis(100, true);
	setIntake(127, false);
	delayMillis(550, true);
	driveSet(-0.75, DRIVE_SPEED);
	delayMillis(1500);
	// Matchload
	setAligner(false);
	setIntake(127, true);
	moveToPoint({23.5, 10}, rev, 70);
	delayMillis(300, true);
	setScraper(true);
	if(autonMode != BRAIN) {
		delayMillis(900);
		chassis.drive_set(0, 0);
		delayMillis(300);
		chassis.drive_set(-20, -20);
		setPosition(getDistanceActual(getDistanceActualSide(), chassis.odom_theta_get()), 13.5);
	} else
		delayMillis(900);
	// Score middle goal
	setScraper(false);
	moveThroughPoints({{autonMode == BRAIN ? 24 : chassis.odom_x_get(), 17}, {44, 44}, {57.5, 57.5}}, fwd, DRIVE_SPEED);
	delayMillis(400);
	setAligner(true);
	setRedirect(true);
	if(autonMode == ODOM)
		chassis.pid_wait_until_index(1);
	else
		delayMillis(1300, true);
	setIntake(90, false);
	setAligner(false);
	pidWait(WAIT);
	/*
	delayMillis(500);
	driveSet(-2, DRIVE_SPEED);
	pidWait(CHAIN);
	driveSet(4, DRIVE_SPEED);
	pidWait(CHAIN);
	// Wing blocks in long goal to center
	moveToPoint({42, 38}, rev, DRIVE_SPEED);
	setIntake(127, true);
	pidWait(CHAIN);
	turnSet(180, TURN_SPEED);
	pidWait(CHAIN);
	setWing(true);
	setDescore(false);
	driveSet(-27, 85, false);
	pidWait(CHAIN);
	turnSet(-135, 50);
	*/
}

//
// LEFT AUTONS
//

void left_elims() {}

void left_split() {
	setPosition(-17.38, 20.92, 328);
	// Collect two blocks under goal
	setRedirect(true);
	setIntake(127, true);
	driveSet(38, DRIVE_SPEED);
	pidWait(CHAIN);
	driveSet(16, 70, false);
	pidWait(WAIT);
	// Collect cluster of three blocks with back intake and score on long goal
	moveThroughPoints({{-16, 44}, {-45.75, 36}}, rev, DRIVE_SPEED);
	delayMillis(650, true);
	setScraper(true);
	delayMillis(300, true);
	setScraper(false);
	pidWait(CHAIN);
	setRedirect(false);
	setAligner(true);
	turnSet(0, TURN_SPEED);
	pidWait(CHAIN);
	moveToPoint({-48.5, 42}, fwd, DRIVE_SPEED);
	delayMillis(100, true);
	setIntake(127, false);
	delayMillis(550, true);
	chassis.drive_set(0, 0);
	delayMillis(1500);
	// Matchload
	setAligner(false);
	setIntake(127, true);
	moveToPoint({-48.5, 10}, rev, 70);
	delayMillis(300, true);
	setScraper(true);
	if(autonMode != BRAIN) {
		delayMillis(900);
		chassis.drive_set(0, 0);
		delayMillis(300);
		chassis.drive_set(-20, -20);
		setPosition(getDistanceActual(getDistanceActualSide(), chassis.odom_theta_get()) - 72, 13.5);
	} else
		delayMillis(900);
	// Score middle goal
	moveThroughPoints({{autonMode == BRAIN ? -48 : chassis.odom_x_get(), 23}, {-28, 44}, {-15, 57.25}}, fwd, DRIVE_SPEED);
	delayMillis(200, true);
	setScraper(false);
	delayMillis(200, true);
	setRedirect(true);
	if(autonMode == ODOM)
		chassis.pid_wait_until_index(1);
	else
		delayMillis(1300, true);
	setIntake(90, 127, false);
	pidWait(WAIT);
	delayMillis(700);
	driveSet(-2, DRIVE_SPEED);
	pidWait(CHAIN);
	driveSet(4, DRIVE_SPEED);
	pidWait(CHAIN);
	// Wing blocks in long goal to center
	moveToPoint({-32, 38}, rev, DRIVE_SPEED);
	setIntake(127, true);
	pidWait(CHAIN);
	turnSet(180, TURN_SPEED);
	pidWait(CHAIN);
	setWing(true);
	setDescore(false);
	driveSet(-27, 85, false);
	pidWait(CHAIN);
	turnSet(-135, 50);
}

void left_greed() {}

void left_rush() {}

void left_superrush() {}

void left_awp() {}

//
// SKILLS
//

void skills() {
	setPosition(-17.38, -51.08, 328);
	// Collect two blocks under goal
	setRedirect(true);
	setIntake(127, true);
	driveSet(38, DRIVE_SPEED);
	pidWait(CHAIN);
	driveSet(16, 70, false);
	pidWait(WAIT);
	// Collect cluster of three blocks with back intake and score on long goal
	moveThroughPoints({{-16, -28}, {-45.75, -36}}, rev, DRIVE_SPEED);
	delayMillis(650, true);
	setScraper(true);
	delayMillis(300, true);
	setScraper(false);
	pidWait(CHAIN);
	setRedirect(false);
	setAligner(true);
	turnSet(0, TURN_SPEED);
	pidWait(CHAIN);
	moveToPoint({-48.5, -30}, fwd, DRIVE_SPEED);
	delayMillis(100, true);
	setIntake(127, false);
	delayMillis(550, true);
	chassis.drive_set(0, 0);
	delayMillis(2000);
	// Matchload
	setAligner(false);
	setIntake(127, true);
	moveToPoint({-48.5, -62}, rev, 70);
	delayMillis(300, true);
	setScraper(true);
	if(autonMode != BRAIN) {
		delayMillis(900);
		chassis.drive_set(0, 0);
		delayMillis(300);
		chassis.drive_set(-20, -20);
		delayMillis(300);
		chassis.drive_set(0, 0);
		delayMillis(300);
		chassis.drive_set(-20, -20);
		delayMillis(300);
		chassis.drive_set(0, 0);
		delayMillis(300);
		chassis.drive_set(-20, -20);
		delayMillis(300);
		setPosition(getDistanceActual(getDistanceActualSide(), chassis.odom_theta_get()) - 72, -58.75);
	} else
		delayMillis(2000);
	// Score on opposing end of long goal
	driveSet(10, DRIVE_SPEED);
	pidWait(CHAIN);
	moveToPoint({-24, -24}, fwd, DRIVE_SPEED, false);
	setScraper(false);
	pidWait(CHAIN);
	setIntake(0);
	moveThroughPoints({{-24, 28}, {-46, 48}}, fwd, DRIVE_SPEED);
	pidWait(CHAIN);
	turnSet(180, TURN_SPEED);
	pidWait(CHAIN);
	moveToPoint({-48, 30}, fwd, DRIVE_SPEED);
	setAligner(true);
	delayMillis(200);
	setIntake(127, false);
	delayMillis(550);
	chassis.drive_set(0, 0);
	delayMillis(3000);
	// Matchload and score
	setAligner(false);
	setIntake(127, true);
	moveToPoint({-48, 62}, rev, 70);
	delayMillis(300, true);
	setScraper(true);
	if(autonMode != BRAIN) {
		delayMillis(900);
		chassis.drive_set(0, 0);
		delayMillis(300);
		chassis.drive_set(-20, -20);
		delayMillis(300);
		chassis.drive_set(0, 0);
		delayMillis(300);
		chassis.drive_set(-20, -20);
		delayMillis(300);
		chassis.drive_set(0, 0);
		delayMillis(300);
		chassis.drive_set(-20, -20);
		delayMillis(300);
		setPosition(getDistanceActual(24, chassis.odom_theta_get()) - 72, 58.75);
	} else
		delayMillis(2000);
	delayMillis(200);
	moveToPoint({-48, 30}, fwd, DRIVE_SPEED);
	setAligner(true);
	delayMillis(200);
	setIntake(127, false);
	delayMillis(550);
	chassis.drive_set(0, 0);
	delayMillis(3000);
	// Score cluster in other long goal
	setIntake(127, true);
	driveSet(-12, DRIVE_SPEED);
	pidWait(WAIT);
	moveToPoint({24, 24}, rev, DRIVE_SPEED);
	pidWait(CHAIN);
	moveToPoint({46, 48}, rev, DRIVE_SPEED);
	pidWait(CHAIN);
	turnSet(180, TURN_SPEED);
	pidWait(CHAIN);
	moveToPoint({48, 30}, fwd, DRIVE_SPEED);
	setAligner(true);
	delayMillis(200);
	setIntake(127, false);
	delayMillis(550);
	chassis.drive_set(0, 0);
	delayMillis(3000);
	// Matchload
	setAligner(false);
	setIntake(127, true);
	moveToPoint({48, 62}, rev, 70);
	delayMillis(300, true);
	setScraper(true);
	if(autonMode != BRAIN) {
		delayMillis(900);
		chassis.drive_set(0, 0);
		delayMillis(300);
		chassis.drive_set(-20, -20);
		delayMillis(300);
		chassis.drive_set(0, 0);
		delayMillis(300);
		chassis.drive_set(-20, -20);
		delayMillis(300);
		chassis.drive_set(0, 0);
		delayMillis(300);
		chassis.drive_set(-20, -20);
		delayMillis(300);
		setPosition(72 - getDistanceActual(getDistanceActualSide(), chassis.odom_theta_get()), 58.75);
	} else
		delayMillis(2000);
	// Score on opposing end of long goal
	driveSet(10, DRIVE_SPEED);
	pidWait(CHAIN);
	moveToPoint({24, 24}, fwd, DRIVE_SPEED, false);
	setScraper(false);
	pidWait(CHAIN);
	moveThroughPoints({{24, -28}, {46, -44}}, fwd, DRIVE_SPEED);
	pidWait(CHAIN);
	turnSet(0, TURN_SPEED);
	pidWait(CHAIN);
	moveToPoint({48, -30}, fwd, DRIVE_SPEED);
	setAligner(true);
	delayMillis(200);
	setIntake(127, false);
	delayMillis(550);
	chassis.drive_set(0, 0);
	delayMillis(3000);
	// Matchload and score
	setAligner(false);
	setIntake(127, true);
	moveToPoint({48.5, -62}, rev, 70);
	delayMillis(300, true);
	setScraper(true);
	if(autonMode != BRAIN) {
		delayMillis(900);
		chassis.drive_set(0, 0);
		delayMillis(300);
		chassis.drive_set(-20, -20);
		delayMillis(300);
		chassis.drive_set(0, 0);
		delayMillis(300);
		chassis.drive_set(-20, -20);
		delayMillis(300);
		chassis.drive_set(0, 0);
		delayMillis(300);
		chassis.drive_set(-20, -20);
		delayMillis(300);
		setPosition(72 - getDistanceActual(24, chassis.odom_theta_get()), -58.75);
	} else
		delayMillis(2000);
	delayMillis(200);
	moveToPoint({48, -30}, fwd, DRIVE_SPEED);
	setAligner(true);
	delayMillis(200);
	setIntake(127, false);
	delayMillis(550);
	chassis.drive_set(0, 0);
	delayMillis(3000);
	// Park
	driveSet(-22, DRIVE_SPEED);
	pidWait(WAIT);
	setIntake(127, false);
	setPosition(48, -54);
	moveToPoint({-8, -72}, rev, 110);

	/*
	// Cross parking barrier and reset position
	moveToPoint({42, 129}, rev, DRIVE_SPEED);
	pidWait(CHAIN);
	swingSet(RIGHT_SWING, -90, SWING_SPEED, cw);
	pidWait(WAIT);
	wall_reset(56, -DRIVE_SPEED, true);
	barrier_reset(30);
	setPosition(88.436, 135, -90);
	// Score parking barrier blocks
	swingSet(RIGHT_SWING, -45, SWING_SPEED, cw);
	pidWait(CHAIN);
	moveToPoint({120, 120}, rev, DRIVE_SPEED, false);
	pidWait(WAIT);
	turnSet(180, TURN_SPEED);
	setAligner(true);
	pidWait(WAIT);
	driveSet(24, DRIVE_SPEED);
	delayMillis(200);
	setIntake(127, false);
	delayMillis(300);
	driveSet(-2, DRIVE_SPEED);
	setAligner(false);
	delayMillis(2000);
	// Matchload and score
	setIntake(127, true);
	setScraper(true);
	driveSet(-35, 80);
	delayMillis(3000);
	chassis.drive_set(0, 0);
	setPosition(120, 130.75);
	delayMillis(200);
	driveSet(35, DRIVE_SPEED);
	setScraper(false);
	setAligner(true);
	delayMillis(200);
	setIntake(127, false);
	delayMillis(300);
	driveSet(-2, DRIVE_SPEED);
	setAligner(false);
	delayMillis(2000);
	// Cross to other matchloader and score (only red)
	setIntake(127, true);
	swingSet(LEFT_SWING, 0, DRIVE_SPEED, 5, ccw);
	pidWait(WAIT);
	setIntake(0);
	moveToPoint({96, 60}, rev, DRIVE_SPEED);
	pidWait(CHAIN);
	moveToPoint({120, 48}, rev, DRIVE_SPEED, false);
	pidWait(WAIT);
	turnSet(0, DRIVE_SPEED);
	setIntake(127, true);
	setScraper(true);
	driveSet(-35, 80);
	setSortPrime(DELAYED);
	delayMillis(3000);
	chassis.drive_set(0, 0);
	setPosition(120, 13.25);
	delayMillis(200);
	driveSet(35, DRIVE_SPEED);
	setScraper(false);
	setAligner(true);
	delayMillis(300);
	setIntake(127, false);
	setIntake(127, 0);
	delayMillis(500);
	driveSet(-2, DRIVE_SPEED);
	setAligner(false);
	delayMillis(2000);
	// Cross parking barrier and reset position
	setIntake(127, true);
	moveToPoint({102, 12}, rev, DRIVE_SPEED);
	pidWait(CHAIN);
	swingSet(RIGHT_SWING, 90, SWING_SPEED, cw);
	pidWait(WAIT);
	wall_reset(56, -DRIVE_SPEED, true);
	barrier_reset(30);
	setPosition(55.564, 135, -90);
	// Control middle
	swingSet(RIGHT_SWING, -45, SWING_SPEED, cw);
	pidWait(CHAIN);
	moveToPoint({48, 48}, rev, DRIVE_SPEED, false);
	pidWait(WAIT);
	turnSet(45, TURN_SPEED);
	setAligner(true);
	pidWait(WAIT);
	setRedirect(true);
	driveSet(9.5, DRIVE_SPEED);
	setIntake(67, false);
	delayMillis(7000);
	// Park
	moveToPoint({72, 48}, rev, DRIVE_SPEED);
	pidWait(CHAIN);
	moveToPoint({72, 0}, rev, DRIVE_SPEED, false);
	*/
}

void skills_awp() {
	setPosition(79.5, 24.75, -90);
	// Matchload and score
	moveToPoint({115.75, 24}, rev, DRIVE_SPEED);
	pidWait(CHAIN);
	turnSet(0, TURN_SPEED);
	pidWait(CHAIN);
	setScraper(true);
	setIntake(127, true);
	// Intake blocks from loader
	driveSet(-13, 90);
	if(autonMode != BRAIN) {
		delayMillis(1200);
		setPosition(124, chassis.odom_y_get());
		chassis.drive_set(0, 0);
		delayMillis(2000);
	}
	// Score on long goal
	driveSet(31, DRIVE_SPEED);
	delayMillis(200);
	setScraper(false);
	setAligner(true);
	if(autonMode != BRAIN) {
		delayMillis(500);
		setIntake(127, false);
		delayMillis(550);
	} else
		pidWait(WAIT);
	driveSet(-2, DRIVE_SPEED);
	// Cross field and score on middle goal
	delayMillis(5000);
	setIntake(127, true);
	delayMillis(300);
	swingSet(ez::RIGHT_SWING, 90, SWING_SPEED, 10);
	pidWait(CHAIN);
	moveToPoint({96, 48}, rev, DRIVE_SPEED, false);
	pidWait(CHAIN);
	moveToPoint({53.5, 48}, rev, 110, false);
	pidWait(CHAIN);
	turnSet(45, TURN_SPEED);
	pidWait(WAIT);
	driveSet(12.5, DRIVE_SPEED);
	delayMillis(200);
	setRedirect(true);
	setAligner(false);
	setIntake(100, false);
	delayMillis(5400);
	pidWait(WAIT);
	// Align to loader/long goal
	moveToPoint({30, 27}, rev, DRIVE_SPEED);
	setIntake(127, false);
	setRedirect(false);
	setScraper(true);
	pidWait(CHAIN);
	turnSet(0, TURN_SPEED);
	pidWait(CHAIN);
	// Intake blocks from loader
	setIntake(127, true);
	driveSet(-15, 90);
	if(autonMode != BRAIN) {
		delayMillis(1200);
		setPosition(23, chassis.odom_y_get());
		chassis.drive_set(0, 0);
		delayMillis(2000);
	}
	// Score on long goal
	driveSet(31, DRIVE_SPEED);
	delayMillis(200);
	setScraper(false);
	setAligner(true);
	if(autonMode != BRAIN) {
		delayMillis(500);
		setIntake(127, false);
		delayMillis(550);
	} else
		pidWait(WAIT);
	driveSet(-2, DRIVE_SPEED);
	delayMillis(5000);
	// Park
	driveSet(-24, DRIVE_SPEED);
	pidWait(WAIT);
	setIntake(127, true);
	setPosition(24, 11);
	moveToPoint({82, 0}, rev, 110);
	delayMillis(500);
	if(autonMode != BRAIN) chassis.drive_set(-127, -127);
	delayMillis(2200);
	chassis.drive_set(0, 0);
}