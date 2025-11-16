#include "main.h"  // IWYU pragma: keep

//
// Operator control
//

bool interruptintake = false;
bool interruptDescore = false;
int masterTarget = 0;

bool shift() { return master.get_digital(pros::E_CONTROLLER_DIGITAL_R2); }

void setIntakeOp() {
	if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) sendHaptic(".");

	if(interruptintake) return;

	if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {	 // storing/scoring
		setIntake(127, !shift());
	} else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {	// low goal slow/fast
		setIntake(shift() ? -127 : -90);
	} else {  // at rest
		setIntake(0);
		intakeFront.lock = false;
		intakeBack.lock = false;
	}
	masterTarget = (util::sgn(intakeFront.target));
}

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

void setWingOp() { wing.button_toggle(master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)); }

void setDescoreOp() {
	if(interruptDescore) return;

	if(wing.get())
		descore.set(!master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT));
	else
		descore.set(false);
}

void setParkOp() { park.button_toggle(master.get_digital(pros::E_CONTROLLER_DIGITAL_A)); }

//
// Team control
//

pros::Controller team(pros::E_CONTROLLER_PARTNER);

void setIntakeTeam() {
	if(team.get_digital(pros::E_CONTROLLER_DIGITAL_L1) &&
	   (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) || master.get_digital(pros::E_CONTROLLER_DIGITAL_L2))) {
		interruptintake = true;
		setIntake(-127 * masterTarget);
		if(team.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) sendHaptic(".-");
	} else
		interruptintake = false;
}

void setDescoreTeam() {
	if(team.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
		if(team.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) sendHaptic("-.");
		interruptDescore = true;

		if(wing.get()) {
			descore.set(!team.get_digital(pros::E_CONTROLLER_DIGITAL_L2));
		} else
			descore.set(false);
	} else
		interruptDescore = false;
}

void setBrakesTeam() {
	if(team.get_digital(pros::E_CONTROLLER_DIGITAL_A))
		setBrakes(true);
	else if(!master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT))
		setBrakes(false);
}