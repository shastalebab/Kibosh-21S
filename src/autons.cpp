#include "drive.hpp"
#include "main.h"  // IWYU pragma: keep
#include "subsystems.hpp"

// Commonly used speed constants
const int DRIVE_SPEED = 127;
const int TURN_SPEED = 110;
const int SWING_SPEED = 110;

///
// Constants
///
void default_constants() {
	// P, I, D, and Start I
	chassis.pid_drive_constants_set(16.5, 0.0, 170.25);	 // Straight driving constants, used for odom and non odom motions
	chassis.pid_heading_constants_set(9.25, 0.1,
									  31.25);  // Holds the robot straight while going forward without odom
	chassis.pid_turn_constants_set(4.0, 0.15, 29.5,
								   30.0);					   // Turn in place constants
	chassis.pid_swing_constants_set(6.75, 0.0, 57.75);		   // Swing constants
	chassis.pid_odom_angular_constants_set(6.5, 0.0, 52.5);	   // Angular control for odom motions
	chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);  // Angular control for boomerang motions
	chassis.pid_drive_constants_get();

	// Exit conditions
	chassis.pid_turn_exit_condition_set(50_ms, 3_deg, 170_ms, 9_deg, 150_ms, 150_ms, false);
	chassis.pid_swing_exit_condition_set(70_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms, false);
	chassis.pid_drive_exit_condition_set(70_ms, 1.7_in, 180_ms, 4_in, 200_ms, 200_ms, false);
	chassis.pid_odom_turn_exit_condition_set(40_ms, 3_deg, 170_ms, 6_deg, 500_ms, 750_ms, false);
	chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms, false);
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
	chassis.odom_turn_bias_set(0.5);

	chassis.odom_look_ahead_set(7_in);			 // This is how far ahead in the path the robot looks at
	chassis.odom_boomerang_distance_set(16_in);	 // This sets the maximum distance away from target that the carrot
												 // point can be
	chassis.odom_boomerang_dlead_set(0.625);	 // This handles how aggressive the end of boomerang motions are

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
	chassis.headingPID.target_set(chassis.drive_imu_get() + degrees);
}

void constants_test() {
	setPosition(85.44, 20.86, 45);
	driveSet(12, 127);
	pidWait(WAIT);
	turnSet(90, 127);
	pidWait(WAIT);
	turnSet(0, 127);
	pidWait(WAIT);
	turnSet(45, 127);
	pidWait(WAIT);
}

//
// RIGHT AUTONS
//

void right_split() {
	
}

void right_greed() {
	setPosition(91.5, 9, 0);
	// Collect middle three blocks
	moveToPoint({97, 57}, fwd, DRIVE_SPEED);
	setIntake(127, true);
	pidWaitUntil(16_in);
	chassis.pid_speed_max_set(60);
	pidWait(WAIT);
	// Grab blocks under long goal
	moveToPoint({118, 66}, rev, DRIVE_SPEED);
	pidWait(WAIT);
	setScraper(true);
	setRedirect(false);
	// Align to loader/long goal
	driveSet(20, DRIVE_SPEED);
	delayMillis(200);
	pidWait(WAIT);
	moveToPoint({121, 22}, rev, DRIVE_SPEED);
	pidWait(WAIT);
	turnSet(0, TURN_SPEED);
	pidWait(WAIT);
	// Grab blocks from loader and score on long goal
	driveSet(-17, 70);
	delayMillis(600);
	chassis.drive_set(0, 0);
	delayMillis(400);
	turnSet(1, TURN_SPEED);
	pidWait(CHAIN);
	driveSet(27, 127);
	delayMillis(300);
	setScraper(false);
	delayMillis(200);
	setAligner(true);
	pidWait(WAIT);
	chassis.drive_set(0, 0);
	setIntake(127, false);
	// Push blocks into center with wing
	delayMillis(2800);
	setIntake(127, true);
	swingSet(RIGHT_SWING, 90, SWING_SPEED, 40);
	pidWait(WAIT);
	turnSet(0, TURN_SPEED);
	pidWait(WAIT);
	setWing(true);
	setDescore(false);
	driveSet(40, 75);
	pidWait(WAIT);
	chassis.drive_set(0, 0);
}

void right_awp() {
	
}

//
// LEFT AUTONS
//

void left_split() {
	setPosition(50, 9, 0);
	// Collect and score middle three blocks in mid goal
	moveToPoint({47, 47}, fwd, DRIVE_SPEED);
	setIntake(127, true);
	pidWaitUntil(16_in);
	chassis.pid_speed_max_set(35);
	pidWait(WAIT);
	turnSet(45, TURN_SPEED);
	setRedirect(true);
	pidWait(WAIT);
	driveSet(13, DRIVE_SPEED);
	pidWait(QUICK);
	// Grab blocks under long goal
	setIntake(127, false);
	delayMillis(300);
	setIntake(110, false);
	delayMillis(900);
	setIntake(127, true);
	delayMillis(100);
	moveToPoint({26, 66}, rev, DRIVE_SPEED);
	pidWait(WAIT);
	setScraper(true);
	setRedirect(false);
	// Align to loader/long goal
	driveSet(20, DRIVE_SPEED);
	delayMillis(200);
	pidWait(WAIT);
	moveToPoint({24, 22}, rev, DRIVE_SPEED);
	pidWait(WAIT);
	turnSet(0, TURN_SPEED);
	pidWait(WAIT);
	// Grab blocks from loader and score on long goal
	driveSet(-17, 70);
	delayMillis(600);
	chassis.drive_set(0, 0);
	delayMillis(800);
	turnSet(-1 , TURN_SPEED);
	pidWait(CHAIN);
	driveSet(27, 127);
	delayMillis(300);
	setScraper(false);
	delayMillis(200);
	setAligner(true);
	pidWait(WAIT);
	chassis.drive_set(0, 0);
	setIntake(127, false);
}

void left_greed() {
	setPosition(50, 9, 0);
	// Collect and score middle three blocks in mid goal
	moveToPoint({47, 57}, fwd, DRIVE_SPEED);
	setIntake(127, true);
	pidWaitUntil(16_in);
	chassis.pid_speed_max_set(80);
	pidWait(WAIT);
	// Grab blocks under long goal
	moveToPoint({26, 67}, rev, DRIVE_SPEED);
	pidWait(WAIT);
	setScraper(true);
	setRedirect(false);
	// Align to loader/long goal
	driveSet(20, DRIVE_SPEED);
	delayMillis(200);
	pidWait(WAIT);
	moveToPoint({23, 22}, rev, DRIVE_SPEED);
	pidWait(WAIT);
	turnSet(0, TURN_SPEED);
	pidWait(WAIT);
	// Grab blocks from loader and score on long goal
	driveSet(-17, 70);
	delayMillis(600);
	chassis.drive_set(0, 0);
	delayMillis(400);
	turnSet(0, TURN_SPEED);
	pidWait(CHAIN);
	driveSet(27, 127);
	delayMillis(300);
	setScraper(false);
	delayMillis(200);
	setAligner(true);
	pidWait(WAIT);
	chassis.drive_set(0, 0);
	setIntake(127, false);
	// Push blocks into center with wing
	delayMillis(2800);
	setIntake(127, true);
	swingSet(RIGHT_SWING, 90, SWING_SPEED, 40);
	pidWait(WAIT);
	turnSet(0, TURN_SPEED);
	pidWait(WAIT);
	setWing(true);
	setDescore(false);
	driveSet(40, 75);
	pidWait(WAIT);
	chassis.drive_set(0, 0);
}

void left_awp() {
	setPosition(64.5, 24.75, 270);
	// Clear matchloader & score on long goal
	moveToPoint({23, 24}, fwd, DRIVE_SPEED);
	pidWaitUntil(20_in);
	setScraper(true);
	pidWait(WAIT);
	turnSet(0, TURN_SPEED);
	pidWait(WAIT);
	setIntake(127, true);
	// Grab blocks from loader and score on long goal
	driveSet(-17, 110);
	delayMillis(600);
	chassis.drive_set(0, 0);
	delayMillis(400);
	turnSet(1, 60);
	pidWait(CHAIN);
	setScraper(false);
	driveSet(26.25, DRIVE_SPEED);
	delayMillis(300);
	setAligner(true);
	pidWait(WAIT);
	chassis.drive_set(0, 0);
	setIntake(127, false);
	delayMillis(1500);
	setIntake(127, true);
	driveSet(-17, DRIVE_SPEED);
	setAligner(false);
	pidWait(WAIT);
	// Score blocks on middle goal
	setRedirect(true);
	turnSet(45, TURN_SPEED);
	pidWait(WAIT);
	driveSet(47.5, 90);
	pidWaitUntil(40_in);
	setIntake(127, false);
	pidWait(WAIT);
	delayMillis(1000);
	// Go to other loader
	setIntake(127, true);
	moveToPoint({124.1, 32}, rev, DRIVE_SPEED);
	pidWait(WAIT);
	setRedirect(false);
	turnSet(0, TURN_SPEED);
	pidWait(WAIT);
	setScraper(true);
	// Grab blocks from loader and score on long goal
	driveSet(-27, 90);
	delayMillis(600);
	chassis.drive_set(0, 0);
	delayMillis(400);
	turnSet(-1, 60);
	pidWait(CHAIN);
	setScraper(false);
	driveSet(26.25, DRIVE_SPEED);
	delayMillis(300);
	setAligner(true);
	pidWait(WAIT);
	chassis.drive_set(0, 0);
	setIntake(127, false);
}

//
// SKILLS
//

void skills() {
	Colors userColor = allianceColor;
	setPosition(89.5, 9, 90);
	setAlliance(NEUTRAL);
	// Clear parking barrier
	//driveSet(16, DRIVE_SPEED);
	pidWait(WAIT);
	if(autonMode != BRAIN) {
		driveSet(-90, DRIVE_SPEED, true);
		setIntake(127, true);
		delayMillis(500);
		chassis.drive_set(-127, -127);
		delayMillis(1500);
		chassis.drive_set(0, 0);
		setPosition(getDistanceActual(), 9);
		delayMillis(250);
	} else {
		driveSet(56, 70, true);
		pidWait(WAIT);
	}
	// Score parking barrier blocks
	swingSet(RIGHT_SWING, 135, SWING_SPEED);
	moveToPoint({26, 24}, fwd, DRIVE_SPEED);
	pidWait(WAIT);
	setAligner(true);
	moveToPoint({26, 42}, fwd, DRIVE_SPEED);
	pidWait(WAIT);
	setIntake(127, false);
	delayMillis(1200);
	setIntake(127, true);
	setAligner(false);
	// Intake and score matchloader blocks
	driveSet(-27, 90);
	delayMillis(200);
	setScraper(true);
	delayMillis(3000);
	driveSet(26, 60);
	delayMillis(400);
	setScraper(false);
	delayMillis(200);
	setAligner(true);
	pidWait(WAIT);
	setIntake(127, false);
	// Go to other side of field and repeat matchload scoring
	swingSet(RIGHT_SWING, 90, SWING_SPEED, 35, cw);
	pidWait(WAIT);
	moveToPoint({12, 104}, fwd, DRIVE_SPEED);
	pidWait(WAIT);
	moveToPoint({24, 120}, fwd, TURN_SPEED);
	setAligner(false);
	pidWait(WAIT);
	turnSet(180, TURN_SPEED);
	setScraper(true);
	setIntake(127, true);
	pidWait(WAIT);
	driveSet(-17, 60);
	delayMillis(1200);
	driveSet(26.5, 60);
	delayMillis(400);
	setScraper(false);
	delayMillis(200);
	setAligner(true);
	pidWait(WAIT);
	setIntake(127, false);
	delayMillis(1200);
	setIntake(127, true);
	// Collect other parking barrier blocks
	driveSet(-12, DRIVE_SPEED);
	pidWait(WAIT);
	moveToPoint({40, 130}, fwd, DRIVE_SPEED);
	pidWait(WAIT);
	setAlliance(NEUTRAL);
	turnSet(45, TURN_SPEED);
	pidWait(WAIT);
	swingSet(LEFT_SWING, 90, SWING_SPEED, 30, cw);
	if(autonMode != BRAIN) {
		driveSet(60, DRIVE_SPEED, true);
		setIntake(127, true);
		delayMillis(500);
		chassis.drive_set(127, 127);
		delayMillis(1500);
		chassis.drive_set(0, 0);
		setPosition(getDistanceActual(), 9);
		delayMillis(250);
		swingSet(LEFT_SWING, 90, SWING_SPEED, cw);
		pidWait(WAIT);
		setPosition(144 - getDistanceActual(), 122.5, 90);
	} else {
		driveSet(48, 70, true);
		pidWait(WAIT);
	}
	pidWait(WAIT);
	moveToPoint({118, 120}, fwd, DRIVE_SPEED);
	pidWait(WAIT);
	moveToPoint({118, 102}, fwd, DRIVE_SPEED);
	pidWait(WAIT);
	setIntake(127, false);
	delayMillis(1200);
	setIntake(127, true);
	// Intake and score matchloader blocks
	driveSet(-27, DRIVE_SPEED);
	setAligner(false);
	delayMillis(200);
	setScraper(true);
	delayMillis(3000);
	driveSet(26, DRIVE_SPEED);
	delayMillis(400);
	setScraper(false);
	delayMillis(200);
	setAligner(true);
	pidWait(WAIT);
	setIntake(127, false);
	delayMillis(1200);
	setIntake(127, true);
	// Go to other side of field and repeat matchload scoring again, but only on low goals
	swingSet(RIGHT_SWING, 270, SWING_SPEED, 35, cw);
	pidWait(WAIT);
	moveToPoint({132, 40}, fwd, DRIVE_SPEED);
	pidWait(WAIT);
	moveToPoint({120, 24}, fwd, TURN_SPEED);
	setAligner(false);
	pidWait(WAIT);
	turnSet(0, TURN_SPEED);
	setScraper(true);
	pidWait(WAIT);
	driveSet(-17, 60);
	delayMillis(1200);
	driveSet(26.5, 60);
	delayMillis(400);
	setScraper(false);
	delayMillis(200);
	setAligner(true);
	pidWait(WAIT);
	setIntake(127, false);
	delayMillis(1200);
	setIntake(127, true);
	driveSet(-18, DRIVE_SPEED);
	pidWait(WAIT);
	moveToPoint({72, 0}, fwd, DRIVE_SPEED);

	/*
	pidWait(WAIT);
	setAlliance(RED);
	//setIntake(127, -127, 127, false);
	delayMillis(2000);
	// Score blue on mid goal
	setAlliance(BLUE);
	//setIntake(127);
	moveToPoint({96, 48}, fwd, 70);
	pidWait(WAIT);
	moveToPoint({48, 48}, fwd, 70);
	pidWait(WAIT);
	moveToPoint({58, 58}, fwd, 70);
	pidWait(WAIT);
	//setIntake(80, -90, -30, false);
	delayMillis(2500);
	//setIntake(127);
	// Score red on low goal
	setAlliance(RED);
	moveToPoint({48, 48}, rev, DRIVE_SPEED);
	pidWait(WAIT);
	moveToPoint({96, 48}, rev, DRIVE_SPEED);
	pidWait(WAIT);
	moveToPoint({92, 18}, fwd, TURN_SPEED);
	pidWait(WAIT);
	moveToPoint({96, 96}, fwd, 90);
	pidWait(WAIT);
	moveToPoint({48, 96}, fwd, 70);
	pidWait(WAIT);
	moveToPoint({59, 85}, fwd, 70);
	pidWait(WAIT);
	//setIntake(-70, -100, 127, false);
	delayMillis(3000);
	setAlliance(NEUTRAL);
	//setIntake(127);
	moveToPoint({48, 96}, rev, DRIVE_SPEED);
	pidWait(WAIT);
	moveToPoint({72, 112}, fwd, DRIVE_SPEED);
	pidWait(WAIT);
	turnSet(0, TURN_SPEED);
	pidWait(WAIT);
	driveSet(24, DRIVE_SPEED);
	pidWait(CHAIN);
	if(autonMode != BRAIN) {
		chassis.drive_set(DRIVE_SPEED, DRIVE_SPEED);
		delayMillis(1000);
		driveSet(5, DRIVE_SPEED);
	}
	setAlliance(userColor);
	*/
}

void skills_awp() {
	setPosition(64.5, 24.75, 270);
	// Clear matchloader & score on long goal
	moveToPoint({23, 24}, fwd, DRIVE_SPEED);
	pidWaitUntil(20_in);
	setScraper(true);
	pidWait(WAIT);
	turnSet(0, TURN_SPEED);
	pidWait(WAIT);
	setIntake(127, true);
	// Grab blocks from loader and score on long goal
	driveSet(-17, 70);
	delayMillis(1000);
	chassis.drive_set(0, 0);
	delayMillis(2000);
	turnSet(1, 60);
	pidWait(CHAIN);
	setScraper(false);
	driveSet(26.25, DRIVE_SPEED);
	delayMillis(300);
	setAligner(true);
	pidWait(WAIT);
	chassis.drive_set(0, 0);
	setIntake(127, false);
	delayMillis(5000);
	setIntake(127, true);
	driveSet(-17, DRIVE_SPEED);
	setAligner(false);
	pidWait(WAIT);
	// Score blocks on middle goal
	setRedirect(true);
	turnSet(45, TURN_SPEED);
	pidWait(WAIT);
	driveSet(48, 70);
	pidWaitUntil(44_in);
	setIntake(127, false);
	pidWait(WAIT);
	setIntake(90);
	delayMillis(3000);
	// Go to other loader
	setIntake(127, true);
	moveToPoint({124.1, 32}, rev, DRIVE_SPEED);
	pidWait(WAIT);
	setRedirect(false);
	turnSet(0, TURN_SPEED);
	pidWait(WAIT);
	setScraper(true);
	// Grab blocks from loader and score on long goal
	driveSet(-27, 70);
	delayMillis(1000);
	chassis.drive_set(0, 0);
	delayMillis(2000);
	turnSet(-1, 60);
	pidWait(CHAIN);
	setScraper(false);
	driveSet(26.25, DRIVE_SPEED);
	delayMillis(300);
	setAligner(true);
	pidWait(WAIT);
	chassis.drive_set(0, 0);
	setIntake(127, false);
	delayMillis(5000);
	// Park
	driveSet(-24, DRIVE_SPEED);
	pidWait(WAIT);
	setIntake(127, true);
	setPosition(120, 11);
	moveToPoint({62, 0}, rev, 110);
	delayMillis(500);
	if(autonMode != BRAIN) chassis.drive_set(-127, -127);
	delayMillis(2000);
	chassis.drive_set(0, 0);
}