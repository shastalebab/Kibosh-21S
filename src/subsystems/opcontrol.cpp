#include "EZ-Template/util.hpp"
#include "main.h"  // IWYU pragma: keep
#include "pros/misc.h"
#include "subsystems.hpp"

//
// Operator control
//

bool interrupt = false;
int masterTarget = 0;

bool shift() { return master.get_digital(pros::E_CONTROLLER_DIGITAL_R2); }

void setIntakeOp() {
	if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) sendHaptic(".");

	if(interrupt) return;

	if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {	 // storing/scoring
		setIntake((shift() && redirect.get()) ? 90 : 127, !shift());
	} else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {	// low goal slow/fast
		setIntake(shift() ? -127 : -90);
	} else {  // at rest
		setIntake(0);
		intakeFront.lock = false;
		intakeBack.lock = false;
	}
	masterTarget = (util::sgn(intakeFront.target));
}

void setSorterOp() { sorter.button_toggle(master.get_digital(pros::E_CONTROLLER_DIGITAL_X)); }

void setRedirectOp() {
	if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
		if(!redirect.get() && scraper.get()) {
			sendHaptic("--");
			return;
		}
		redirect.set(!redirect.get());
	}
}

void setScraperOp() {
	if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
		if(!scraper.get() && redirect.get()) {
			redirect.set(false);
			pros::delay(200);
		}
		scraper.set(!scraper.get());
	}
}

void setDescoreOp() { descore.button_toggle(master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)); }

void setParkOp() { park.button_toggle(master.get_digital(pros::E_CONTROLLER_DIGITAL_A)); }

//
// Team control
//

pros::Controller team(pros::E_CONTROLLER_PARTNER);

void setIntakeTeam() {
	if(team.get_digital(pros::E_CONTROLLER_DIGITAL_L1) &&
	   (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) || master.get_digital(pros::E_CONTROLLER_DIGITAL_L2))) {
		interrupt = true;
		setIntake(-127 * masterTarget);
		if(team.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) sendHaptic(".-");
	} else
		interrupt = false;
}

void setSorterTeam() { sorter.button_toggle(team.get_digital(pros::E_CONTROLLER_DIGITAL_L2)); }

void setBrakesTeam() {
	if(team.get_digital(pros::E_CONTROLLER_DIGITAL_A))
		setBrakes(true);
	else if(!master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT))
		setBrakes(false);
}