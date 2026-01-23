#include "main.h"

bool drifting = false;

// Chassis constructor
ez::Drive chassis(
	// These are your drive motors, the first motor is used for sensing!
	{-12, 16, -17},	 // Left Chassis Ports (negative port will reverse it!)
	{11, -20, 19},	 // Right Chassis Ports (negative port will reverse it!)

	21,				 // IMU Port
	WHEEL_DIAMETER,	 // Wheel Diameter
	400);			 // Wheel RPM

void initialize() {
	pros::delay(500);  // Stop the user from doing anything while legacy ports configure

	// Configure chassis controls
	chassis.opcontrol_curve_buttons_toggle(false);	// Enables modifying the controller curve with controller buttons
	chassis.opcontrol_drive_activebrake_set(2.0);	// Sets the active brake kP
	chassis.opcontrol_curve_default_set(0.0,
										0.0);  // Set defaults for controller curve

	// Set default drive constants
	default_constants();

	// Add autons to auton selector
	auton_sel.selector_populate({{right_split, "right_split", "right side 3 + 6", gray},
								 {left_split, "left_split", "left side 3 + 6", lv_color_lighten(gray, 125)},
								 {right_awp, "right_awp", "right side 4 + 6 + 3 solo AWP", violet},
								 //{right_greed, "right_greed", "right side 9 in long goal", lv_color_darken(green, 60)},
								 //{left_greed, "left_greed", "left side 9 in long goal", green},
								 {right_rush, "right_rush", "right side 7 in long goal", lv_color_darken(red, 60)},
								 {left_rush, "left_rush", "left side 7 in long goal", lv_color_darken(red, 30)},
								 //{right_superrush, "right_rush", "right side 4 in long goal", lv_color_lighten(red, 30)},
								// {left_superrush, "left_superrush", "left side 4 in long goal", lv_color_lighten(red, 60)},
								 {skills, "skills", "skills route", lv_color_darken(blue, 60)},
								 {skills_awp, "skills_awp", "awp route but for skills", lv_color_lighten(blue, 60)},
								 {constants_test, "constants_test", "drive and turn", blue}});

	// Initialize chassis
	chassis.initialize();
	double initial = chassis.drive_imu_get();
	pros::delay(1000);
	if(abs(chassis.drive_imu_get() - initial) > 1) drifting = true;

	// Initialize auton selector, and tasks
	uiInit();
	pros::Task StanleyTask(stanleyTask, "stanley controller");
	pros::Task ColorTask(colorTask, "color sort");
	pros::Task AntiJamTask(antiJamTask, "antijam");
	pros::Task ControllerTask(masterControllerTask, "master controller printing");
	pros::Task PathViewerTask(pathViewerTask, "path viewer");
	pros::Task AngleCheckTask(angleCheckTask, "angle checker");
	pros::Task MotorUpdateTask(motorUpdateTask, "motor info updater");
	master.rumble(chassis.drive_imu_calibrated() && !drifting ? "." : "---");
}

void disabled() {}

void competition_initialize() { autonMode = BRAIN; }

void autonomous() {
	chassis.pid_targets_reset();   // Resets PID targets to 0
	chassis.drive_imu_reset();	   // Reset gyro position to 0
	chassis.drive_sensor_reset();  // Reset drive sensors to 0
	chassis.odom_xyt_set(0_in, 0_in,
						 0_deg);				// Reset current position to a default pose
	chassis.drive_brake_set(MOTOR_BRAKE_HOLD);	// Set motors to hold.  This helps
												// autonomous consistency

	autonMode = ODOM;				// Sets which "mode" the auton is run in (between "PLAIN"
									// and "ODOM")
	autonPath = {};					// Clears saved auton path
	auton_sel.selector_callback();	// Calls selected auton from autonomous selector
}

void opcontrol() {
	chassis.drive_brake_set(pros::E_MOTOR_BRAKE_BRAKE);
	autonMode = PLAIN;	// Sets "mode" to "PLAIN", to ensure that any drive macros
						// in opcontrol only use PID

	while(true) {
		if(!probing && !overrideDrive) chassis.opcontrol_tank();  // Tank control

		setIntakeOp();	  // Intake controls
		setRedirectOp();  // Redirect controls
		setScraperOp();	  // Scraper controls
		setAlignerOp();	  // Aligner controls
		setWingOp();	  // Wing controls
		setDescoreOp();	  // Hook controls
		setStraightOp();

		setIntakeTeam();	// Team intake overrides
		setDescoreTeam();	// Team descore overrides
		setStraightTeam();	// Team driving overrides

		pros::delay(ez::util::DELAY_TIME);
	}
}
