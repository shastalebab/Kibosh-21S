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

void park() {
	int it = 0;
	int jit = 0;
	while(it < 10 && jit < 160) {
		if(getDistanceActual(BACK, false, 72) <= 72)
			it++;
		else
			it = 0;
		jit++;
		pros::delay(10);
	}
	chassis.drive_set(0, 0);
}

//
// MODULES (used within autons)
//

void wall_reset(double in, int speed, bool slew) {
	if(autonMode == BRAIN || speed == 0) return;

	chassis.pid_drive_set(util::sgn(speed) * 1000, abs(speed), slew);

	int it = 0;
	while(it < 40) {
		if(getDistanceActual(BACK, false, 48) <= in + 12)
			it++;
		else
			it--;
		if(it < 0) it = 0;
		pros::delay(10);
	}
	chassis.pid_drive_set(util::sgn(speed) * 3, abs(speed));
	chassis.pid_wait();
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

void constants_test() {
	setPosition(48, 32, 180);
	// Cross park zone
	driveSet(-18, DRIVE_SPEED);
	pidWait(CHAIN);
	setIntake(127, true);
	setAligner(false);
	if(autonMode == ODOM) {
		chassis.pid_odom_set({{{28, 62}, rev, DRIVE_SPEED}, {{22, 64}, rev, 100}, {{0, 64}, rev, DRIVE_SPEED}, {{-90, 64}, rev, DRIVE_SPEED}});
		delayMillis(900);
	} else {
		moveThroughPoints({{28, 62}, {22, 64}, {0, 64}, {-90, 64}}, rev, 100);
		delayMillis(750, true);
	}
	setScraper(true);
	delayMillis(200, true);
	setScraper(false);
	delayMillis(1300, true);
	wall_reset(48, -DRIVE_SPEED, true);
	delayMillis(400, true);
	// Reset position
	angle_offset = 90;
	setPosition(getDistanceActual(BACK, true, -30) - 72, 72 - getDistanceActual(LEFT, true, 13));
	angle_offset = 0;
	// Make sure park zone blocks are properly collected
	setScraper(true);
	setIntake(-80, false);
	delayMillis(1200);
	setIntake(127, true);
	setScraper(false);
	delayMillis(200);
	// Collect one red block from cluster and score low goal
	swingSet(LEFT_SWING, 180, SWING_SPEED, cw);
	pidWait(CHAIN);
	moveThroughPoints({{-8, 32}, {-20, 20}}, fwd, DRIVE_SPEED);
	pidWait(WAIT);
	turnSet(135, TURN_SPEED);
	pidWait(WAIT);
	setAligner(true);
	moveToPoint({-8, 10}, fwd, DRIVE_SPEED);
	pidWait(WAIT);
	turnSet(135, TURN_SPEED);
	setIntake(-120, false);
	delayMillis(400);
	setIntake(-50, false);
	delayMillis(3000);
	setAligner(false);
	// Ensure last block enters goal
	driveSet(-4, 80);
	pidWait(CHAIN);
	driveSet(2, 40);
	pidWait(WAIT);
	// Score extra blocks in long goal (POTENTIALLY REPLACE WITH MATCHLOAD)
	moveToPoint({-46, 48}, rev, DRIVE_SPEED);
	pidWait(CHAIN);
	setIntake(127, true);
	turnSet(180, TURN_SPEED);
	pidWait(CHAIN);
	moveToPoint({-48, 30}, fwd, DRIVE_SPEED);
	setAligner(true);
	delayMillis(200);
	setIntake(127, false);
	delayMillis(550);
	driveSet(-0.25, DRIVE_SPEED);
}

//
// RIGHT AUTONS
//

void right_split() {
	setPosition(17.38, 20.92, -328);
	// Collect two blocks under goal
	setRedirect(true);
	setIntake(127, true);
	driveSet(54, DRIVE_SPEED);
	pidWait(WAIT);
	// Collect cluster of three blocks with back intake and score on long goal
	moveThroughPoints({{16, 44}, {46.25, 36}}, rev, DRIVE_SPEED);
	delayMillis(300, true);
	setRedirect(false);
	delayMillis(350, true);
	setScraper(true);
	delayMillis(300, true);
	setScraper(false);
	pidWait(CHAIN);
	setAligner(true);
	turnSet(0, TURN_SPEED);
	pidWait(CHAIN);
	moveToPoint({49.75, 42}, fwd, DRIVE_SPEED);
	delayMillis(200, true);
	setIntake(127, false);
	delayMillis(450, true);
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
		setPosition(72 - getDistanceActual(RIGHT, true, 24), getDistanceActual(BACK, true, 13.5));
	} else
		delayMillis(900);
	// Score middle goal
	moveThroughPoints({{autonMode == BRAIN ? 48 : chassis.odom_x_get(), 23}, {28, 44}, {8.75, 63.25}}, fwd, DRIVE_SPEED);
	delayMillis(200, true);
	setScraper(false);
	if(autonMode == ODOM)
		chassis.pid_wait_until_index(1);
	else
		delayMillis(1300, true);
	setAligner(true);
	pidWait(WAIT);
	setIntake(-100, false);
	delayMillis(1200);
	setAligner(false);
	driveSet(-4, DRIVE_SPEED);
	pidWait(CHAIN);
	driveSet(4, DRIVE_SPEED);
	pidWait(CHAIN);
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

void right_spread() {
	setPosition(17.38, 20.92, -328);
	// Collect two blocks under goal
	setRedirect(true);
	setIntake(127, true);
	driveSet(54, DRIVE_SPEED);
	pidWait(WAIT);
	// Collect cluster of three blocks with back intake
	moveToPoint({16, 44}, rev, DRIVE_SPEED);
	delayMillis(300, true);
	setRedirect(false);
	delayMillis(350, true);
	setScraper(true);
	delayMillis(300, true);
	setScraper(false);
	// Score block in upper middle goal
	setAligner(true);
	moveThroughPoints({{6, 55}, {1.25, 59.5}}, fwd, DRIVE_SPEED);
	pidWait(WAIT);
	setIntake(90, false);
	delayMillis(500);
	setIntake(127, true);
	// Score rest of blocks in lower middle goal
	moveToPoint({19, 56.25}, rev, DRIVE_SPEED);
	pidWait(CHAIN);
	moveToPoint({11.5, 63.75}, fwd, DRIVE_SPEED);
	pidWait(WAIT);
	setIntake(-127, false);
	delayMillis(1000);
	setAligner(false);
	// Matchload
	moveToPoint({46.25, 36}, rev, DRIVE_SPEED);
	delayMillis(300, true);
	setIntake(127, true);
	pidWait(CHAIN);
	turnSet(0, TURN_SPEED);
	pidWait(CHAIN);
	moveToPoint({49.75, 10}, rev, 70);
	setScraper(true);
	if(autonMode != BRAIN) {
		delayMillis(900);
		chassis.drive_set(0, 0);
		delayMillis(300);
		chassis.drive_set(-20, -20);
		setPosition(72 - getDistanceActual(RIGHT, true, 24), getDistanceActual(BACK, true, 13.5));
	} else
		delayMillis(900);
	// Score long goal
	moveToPoint({48, 42}, fwd, DRIVE_SPEED);
	delayMillis(200);
	setScraper(false);
	setAligner(true);
	delayMillis(300, true);
	setIntake(127, false);
	delayMillis(550, true);
	driveSet(-.75, DRIVE_SPEED);
	delayMillis(800);
	// Wing blocks to control
	swingSet(LEFT_SWING, 175, DRIVE_SPEED, 3, ccw);
	pidWait(CHAIN);
	setWing(true);
	setDescore(false);
	driveSet(-24, 75, false);
	pidWait(CHAIN);
	turnSet(-135, 50);
}

void right_co_awp() {
	setPosition(14.935, 24.75, -90);
	// Matchload and score
	moveToPoint({43, 24}, rev, DRIVE_SPEED);
	pidWait(CHAIN);
	turnSet(0, TURN_SPEED);
	setScraper(true);
	pidWait(CHAIN);
	setIntake(127, true);
	// Intake blocks from loader
	driveSet(-13, 90);
	if(autonMode != BRAIN) {
		delayMillis(700);
		chassis.drive_set(0, 0);
		delayMillis(400);
		setPosition(72 - getDistanceActual(RIGHT, true, 24), getDistanceActual(BACK, true, 13.5));
	}
	// Score on long goal
	moveToPoint({48.75, 42}, fwd, DRIVE_SPEED);
	delayMillis(200);
	setScraper(false);
	setAligner(true);
	delayMillis(300, true);
	setIntake(127, false);
	delayMillis(550, true);
	chassis.drive_set(0, 0);
	delayMillis(100);
	driveSet(-.75, DRIVE_SPEED);
	delayMillis(700);
	// Grab cluster of three blocks
	setIntake(127, true);
	driveSet(-10, DRIVE_SPEED);
	delayMillis(300, true);
	setAligner(false);
	moveThroughPoints({{24, 51}, {10, 62.75}}, fwd, DRIVE_SPEED);
	pidWait(CHAIN);
	// Score low middle goal
	pidWait(WAIT);
	turnSet(-45, TURN_SPEED);
	setAligner(true);
	setIntake(-100, false);
	delayMillis(800);
	setIntake(127, true);
	// Grab blocks under goal
	driveSet(-4, DRIVE_SPEED);
	pidWait(CHAIN);
	moveToPoint({43, 65.5}, rev, DRIVE_SPEED);
	pidWait(WAIT);
	// Potentially grab other cluster of three blocks and score middle goal
	moveThroughPoints({{22, 53}, {-22, 51}}, fwd, DRIVE_SPEED);
	setAligner(false);
	pidWait(CHAIN);
	setRedirect(true);
	moveToPoint({-14, 60}, fwd, DRIVE_SPEED);
	delayMillis(300, true);
	setIntake(110, false);
	delayMillis(1000);
	driveSet(-3, DRIVE_SPEED);
	pidWait(CHAIN);
	// Wing blocks in long goal to center
	moveThroughPoints({{-16, 54}, {32, 38}}, rev, DRIVE_SPEED);
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

void right_awp() {
	setPosition(0, 24.75, -90);
	// Push other bot and grab preload
	driveSet(4, DRIVE_SPEED);
	setIntake(127, true);
	pidWait(WAIT);
	// Matchload and score
	moveToPoint({43, 24}, rev, DRIVE_SPEED);
	pidWait(CHAIN);
	turnSet(0, TURN_SPEED);
	setScraper(true);
	pidWait(CHAIN);
	// Intake blocks from loader
	driveSet(-13, 70);
	if(autonMode != BRAIN) {
		delayMillis(700);
		chassis.drive_set(0, 0);
		delayMillis(400);
		setPosition(72 - getDistanceActual(RIGHT, true, 24), getDistanceActual(BACK, true, 13.5));
	}
	// Score on long goal
	moveToPoint({48.75, 41}, fwd, DRIVE_SPEED);
	delayMillis(200);
	setScraper(false);
	setAligner(true);
	delayMillis(300, true);
	setIntake(127, false);
	delayMillis(550, true);
	chassis.drive_set(0, 0);
	delayMillis(100);
	driveSet(-.75, DRIVE_SPEED);
	delayMillis(800);
	// Cross field and score on long goal
	setIntake(127, true);
	swingSet(ez::RIGHT_SWING, 135, SWING_SPEED, 2);
	delayMillis(300, true);
	moveThroughPoints({{24, 51}, {-19, 51}, {-45, 30}}, rev, DRIVE_SPEED);
	pidWait(CHAIN);
	turnSet(0, TURN_SPEED);
	pidWait(CHAIN);
	moveToPoint({-48, 41}, fwd, DRIVE_SPEED);
	delayMillis(100, true);
	setIntake(127, false);
	delayMillis(450, true);
	chassis.drive_set(0, 0);
	delayMillis(100);
	driveSet(-0.75, DRIVE_SPEED);
	delayMillis(800);
	// Matchload
	setAligner(false);
	setIntake(127, true);
	moveToPoint({-48.5, 9}, rev, 70);
	delayMillis(300, true);
	setScraper(true);
	if(autonMode != BRAIN) {
		delayMillis(1000);
		chassis.drive_set(0, 0);
		delayMillis(300);
		chassis.drive_set(-20, -20);
		setPosition(getDistanceActual(LEFT, true, 24) - 72, getDistanceActual(BACK, true, 13.5));
	} else
		delayMillis(900);
	// Score middle goal
	driveSet(5, DRIVE_SPEED);
	pidWait(CHAIN);
	moveThroughPoints({{autonMode == BRAIN ? -48 : chassis.odom_x_get(), 24}, {-28, 44}, {-14, 58}}, fwd, DRIVE_SPEED);
	delayMillis(200);
	setScraper(false);
	delayMillis(200);
	setRedirect(true);
	if(autonMode == ODOM)
		chassis.pid_wait_until_index(1);
	else
		delayMillis(1300, true);
	setIntake(110, false);
	pidWait(WAIT);
	delayMillis(800);
	setAligner(false);
	// Push blocks in middle goal
	driveSet(-3, DRIVE_SPEED);
	pidWait(CHAIN);
	driveSet(3, DRIVE_SPEED);
	pidWait(CHAIN);
	driveSet(-.25, DRIVE_SPEED);
}

//
// LEFT AUTONS
//

void left_split() {
	setPosition(-17.38, 20.92, 328);
	// Collect two blocks under goal
	setRedirect(true);
	setIntake(127, true);
	driveSet(54, DRIVE_SPEED);
	pidWait(WAIT);
	// Collect cluster of three blocks with back intake and score on long goal
	moveThroughPoints({{-16, 44}, {-45.75, 36}}, rev, DRIVE_SPEED);
	delayMillis(300, true);
	setRedirect(false);
	delayMillis(350, true);
	setScraper(true);
	delayMillis(300, true);
	setScraper(false);
	pidWait(CHAIN);
	setAligner(true);
	turnSet(0, TURN_SPEED);
	pidWait(CHAIN);
	moveToPoint({-48, 42}, fwd, DRIVE_SPEED);
	delayMillis(100, true);
	setIntake(127, false);
	delayMillis(550, true);
	chassis.drive_set(0, 0);
	delayMillis(100);
	driveSet(-0.75, DRIVE_SPEED);
	delayMillis(1000);
	// Matchload
	setAligner(false);
	setIntake(127, true);
	moveToPoint({-48, 10}, rev, 70);
	delayMillis(300, true);
	setScraper(true);
	if(autonMode != BRAIN) {
		delayMillis(900);
		chassis.drive_set(0, 0);
		delayMillis(300);
		chassis.drive_set(-20, -20);
		setPosition(getDistanceActual(LEFT, true, 24) - 72, getDistanceActual(BACK, true, 13.5));
	} else
		delayMillis(900);
	// Score middle goal
	moveThroughPoints({{autonMode == BRAIN ? -48 : chassis.odom_x_get(), 23}, {-28, 44}, {-14, 58}}, fwd, DRIVE_SPEED);
	delayMillis(200, true);
	setScraper(false);
	delayMillis(200, true);
	setRedirect(true);
	if(autonMode == ODOM)
		chassis.pid_wait_until_index(1);
	else
		delayMillis(1300, true);
	setIntake(110, false);
	pidWait(WAIT);
	delayMillis(700);
	driveSet(-2, DRIVE_SPEED);
	pidWait(CHAIN);
	driveSet(7, DRIVE_SPEED);
	pidWait(CHAIN);
	// Wing blocks in long goal to center
	moveToPoint({-31.75, 38}, rev, DRIVE_SPEED);
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

void left_rush() {
	setPosition(-17.38, 20.92, 328);
	// Collect two blocks under goal
	setRedirect(true);
	setIntake(127, true);
	driveSet(54, DRIVE_SPEED);
	pidWait(WAIT);
	// Collect cluster of three blocks with back intake and score on long goal
	moveThroughPoints({{-16, 44}, {-45.75, 36}}, rev, DRIVE_SPEED);
	delayMillis(300, true);
	setRedirect(false);
	delayMillis(350, true);
	setScraper(true);
	delayMillis(300, true);
	setScraper(false);
	pidWait(CHAIN);
	setAligner(true);
	turnSet(0, TURN_SPEED);
	pidWait(CHAIN);
	moveToPoint({-48, 42}, fwd, DRIVE_SPEED);
	delayMillis(200, true);
	setIntake(127, false);
	delayMillis(450, true);
	chassis.drive_set(0, 0);
	delayMillis(1100);
	// Matchload
	setAligner(false);
	setIntake(127, true);
	moveToPoint({-48, 10}, rev, 70);
	delayMillis(300, true);
	setScraper(true);
	if(autonMode != BRAIN) {
		delayMillis(900);
		chassis.drive_set(0, 0);
		delayMillis(300);
		chassis.drive_set(-20, -20);
		setPosition(getDistanceActual(LEFT, true, 24) - 72, getDistanceActual(BACK, true, 13.5));
	} else
		delayMillis(900);
	// Wing blocks in long goal to center
	moveThroughPoints({{autonMode != BRAIN ? chassis.odom_x_get() : -48, 21}, {-58, 38}, {-58, 42}}, fwd, DRIVE_SPEED);
	if(autonMode == ODOM)
		chassis.pid_wait_until_index(0);
	else
		delayMillis(600);
	setScraper(false);
	setWing(true);
	setDescore(false);
	pidWait(CHAIN);
	setRedirect(true);
	driveSet(22, 85, false);
	pidWait(CHAIN);
	setIntake(0);
	turnSet(-45, 50);
}

//
// SKILLS
//

void skills() {
	setPosition(7.25, -47.64, 180);
	// Collect red block and score in low goal
	setIntake(127, true);
	moveThroughPoints({{12, -18}, {21, -21.64}}, rev, DRIVE_SPEED);
	pidWait(WAIT);
	setAligner(true);
	turnSet(-45, TURN_SPEED);
	pidWait(CHAIN);
	moveToPoint({13, -12}, fwd, DRIVE_SPEED);
	pidWait(WAIT);
	setIntake(-127, false);
	delayMillis(800);
	// Matchload
	driveSet(-4, DRIVE_SPEED);
	setAligner(false);
	pidWait(CHAIN);
	driveSet(2, 30);
	pidWait(CHAIN);
	moveToPoint({45.75, -36}, rev, DRIVE_SPEED);
	delayMillis(400, true);
	setIntake(127, true);
	pidWait(CHAIN);
	turnSet(0, TURN_SPEED);
	pidWait(WAIT);
	moveToPoint({50, -64}, rev, 70);
	delayMillis(100, true);
	setScraper(true);
	if(autonMode != BRAIN) {
		delayMillis(1000);
		chassis.drive_set(0, 0);
		delayMillis(400);
		chassis.drive_set(-30, -30);
		delayMillis(200);
		chassis.drive_set(0, 0);
		delayMillis(400);
		chassis.drive_set(-30, -30);
		delayMillis(200);
		chassis.drive_set(0, 0);
		delayMillis(400);
		chassis.drive_set(-30, -30);
		delayMillis(200);
		chassis.drive_set(0, 0);
		delayMillis(400);
		chassis.drive_set(-30, -30);
		setPosition(72 - getDistanceActual(RIGHT, true, 24), getDistanceActual(BACK, true, 13.5) - 72);
	} else
		delayMillis(2000);
	// Score on opposing end of long goal
	driveSet(10, DRIVE_SPEED);
	pidWait(CHAIN);
	moveToPoint({24, -24}, fwd, DRIVE_SPEED, false);
	setScraper(false);
	pidWait(CHAIN);
	setAligner(true);
	setIntake(0, true);
	moveThroughPoints({{24, 28}, {45, 42}}, fwd, DRIVE_SPEED);
	pidWait(CHAIN);
	turnSet(180, TURN_SPEED);
	pidWait(CHAIN);
	moveToPoint({48, 30}, fwd, DRIVE_SPEED);
	delayMillis(200);
	setIntake(127, false);
	delayMillis(550);
	chassis.drive_set(0, 0);
	delayMillis(100);
	driveSet(-0.25, DRIVE_SPEED);
	delayMillis(1600);
	// Matchload and score
	setAligner(false);
	driveSet(0.5, DRIVE_SPEED);
	pidWait(CHAIN);
	setIntake(127, true);
	moveToPoint({48, 64}, rev, 70);
	delayMillis(300, true);
	setScraper(true);
	if(autonMode != BRAIN) {
		delayMillis(1000);
		chassis.drive_set(0, 0);
		delayMillis(400);
		chassis.drive_set(-30, -30);
		delayMillis(200);
		chassis.drive_set(0, 0);
		delayMillis(400);
		chassis.drive_set(-30, -30);
		delayMillis(200);
		chassis.drive_set(0, 0);
		delayMillis(400);
		chassis.drive_set(-30, -30);
		setPosition(72 - getDistanceActual(LEFT, true, 24), 72 - getDistanceActual(BACK, true, 13.5));
	} else
		delayMillis(2000);
	delayMillis(200);
	moveToPoint({48, 30}, fwd, DRIVE_SPEED);
	setScraper(false);
	setAligner(true);
	delayMillis(200);
	setIntake(127, false);
	delayMillis(550);
	chassis.drive_set(0, 0);
	delayMillis(1700);
	setAligner(false);
	driveSet(0.5, DRIVE_SPEED);
	// Cross park zone
	driveSet(-18, DRIVE_SPEED);
	pidWait(CHAIN);
	setAligner(false);
	if(autonMode == ODOM) {
		chassis.pid_odom_set({{{28, 62}, rev, DRIVE_SPEED}, {{22, 64}, rev, 100}, {{0, 64}, rev, DRIVE_SPEED}, {{-90, 64}, rev, DRIVE_SPEED}});
		delayMillis(900);
	} else {
		moveThroughPoints({{28, 62}, {22, 64}, {0, 64}, {-90, 64}}, rev, 100);
		delayMillis(750, true);
	}
	setScraper(true);
	delayMillis(200, true);
	setScraper(false);
	delayMillis(900, true);
	setIntake(127, true);
	delayMillis(400, true);
	wall_reset(48, -DRIVE_SPEED, true);
	setScraper(true);
	delayMillis(400, true);
	// Reset position
	angle_offset = 90;
	setPosition(getDistanceActual(BACK, true, -30) - 72, 72 - getDistanceActual(LEFT, true, 13));
	angle_offset = 0;
	// Make sure park zone blocks are properly collected
	setIntake(-80, false);
	delayMillis(900);
	setIntake(127, true);
	setScraper(false);
	delayMillis(200);
	// Collect one red block from cluster and score low goal
	swingSet(LEFT_SWING, 180, SWING_SPEED, cw);
	pidWait(CHAIN);
	moveThroughPoints({{-8, 32}, {-20, 20}}, fwd, DRIVE_SPEED);
	pidWait(WAIT);
	turnSet(135, TURN_SPEED);
	pidWait(WAIT);
	setAligner(true);
	moveToPoint({-10, 10}, fwd, DRIVE_SPEED);
	pidWait(WAIT);
	turnSet(135, TURN_SPEED);
	setIntake(-127, false);
	delayMillis(400);
	setIntake(-50);
	delayMillis(800);
	setIntake(127);
	delayMillis(300);
	setIntake(-50);
	delayMillis(2400);
	setAligner(false);
	// Ensure last block enters goal
	driveSet(-5, 80);
	pidWait(CHAIN);
	driveSet(3, 30);
	pidWait(WAIT);
	// Score extra blocks in long goal (POTENTIALLY REPLACE WITH MATCHLOAD)
	moveToPoint({-46, 48}, rev, DRIVE_SPEED);
	pidWait(CHAIN);
	setIntake(127, true);
	turnSet(180, TURN_SPEED);
	pidWait(CHAIN);
	moveToPoint({-48, 30}, fwd, DRIVE_SPEED);
	setAligner(true);
	delayMillis(200);
	setIntake(127, false);
	delayMillis(550);
	chassis.drive_set(0, 0);
	delayMillis(700);
	// Matchload
	setAligner(false);
	driveSet(0.5, DRIVE_SPEED);
	pidWait(CHAIN);
	setIntake(127, true);
	moveToPoint({-48, 64}, rev, 70);
	delayMillis(300, true);
	setScraper(true);
	if(autonMode != BRAIN) {
		delayMillis(1000);
		chassis.drive_set(0, 0);
		delayMillis(400);
		chassis.drive_set(-30, -30);
		delayMillis(200);
		chassis.drive_set(0, 0);
		delayMillis(400);
		chassis.drive_set(-30, -30);
		delayMillis(200);
		chassis.drive_set(0, 0);
		delayMillis(400);
		chassis.drive_set(-30, -30);
		setPosition(getDistanceActual(RIGHT, true, 24) - 72, 72 - getDistanceActual(BACK, true, 13.5));
	} else
		delayMillis(2000);
	// Score on opposing end of long goal
	driveSet(10, DRIVE_SPEED);
	pidWait(CHAIN);
	moveToPoint({-24, 24}, fwd, DRIVE_SPEED, false);
	setScraper(false);
	pidWait(CHAIN);
	setAligner(true);
	setIntake(0, true); 
	moveThroughPoints({{-24, -28}, {-45, -42}}, fwd, DRIVE_SPEED);
	pidWait(CHAIN);
	turnSet(0, TURN_SPEED);
	pidWait(CHAIN);
	moveToPoint({-48, -30}, fwd, DRIVE_SPEED);
	delayMillis(200);
	setIntake(127, false);
	delayMillis(550);
	chassis.drive_set(0, 0);
	delayMillis(1700);
	// Matchload and score
	setAligner(false);
	driveSet(0.5, DRIVE_SPEED);
	pidWait(CHAIN);
	setIntake(127, true);
	moveToPoint({-48, -64}, rev, 70);
	delayMillis(300, true);
	setScraper(true);
	if(autonMode != BRAIN) {
		delayMillis(1000);
		chassis.drive_set(0, 0);
		delayMillis(400);
		chassis.drive_set(-30, -30);
		delayMillis(200);
		chassis.drive_set(0, 0);
		delayMillis(400);
		chassis.drive_set(-30, -30);
		delayMillis(200);
		chassis.drive_set(0, 0);
		delayMillis(400);
		chassis.drive_set(-30, -30);
		setPosition(getDistanceActual(LEFT, true, 24) - 72, getDistanceActual(BACK, true, 13.5) - 72);
	} else
		delayMillis(2000);
	moveToPoint({-48, -30}, fwd, DRIVE_SPEED);
	delayMillis(200);
	setScraper(false);
	delayMillis(200);
	setAligner(true);
	delayMillis(200);
	setIntake(127, false);
	delayMillis(550);
	chassis.drive_set(0, 0);
	delayMillis(1700);
	// Park
	setAligner(false);
	driveSet(0.5, DRIVE_SPEED);
	pidWait(CHAIN);
	driveSet(-18, DRIVE_SPEED);
	pidWait(CHAIN);
	moveThroughPoints({{-28, -62}, {-22, -64}, {0, -64}, {90, -64}}, rev, DRIVE_SPEED);
	delayMillis(900);
	setIntake(127, true);
	setScraper(true);
	delayMillis(800);
	if(autonMode != BRAIN) park();
	setScraper(false);
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

//
// EXTRAS
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
		setPosition(getDistanceActual(LEFT, true, 24) - 72, getDistanceActual(BACK, true, 13.5));
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
		setPosition(getDistanceActual(LEFT, true, 24) - 72, getDistanceActual(BACK, true, 13.5));
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
	setIntake(90, false);
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
		setPosition(72 - getDistanceActual(RIGHT, true, 24), getDistanceActual(BACK, true, 13.5));
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
		setPosition(72 - getDistanceActual(RIGHT, true, 24), getDistanceActual(BACK, true, 13.5));
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
