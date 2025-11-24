#include "main.h"  // IWYU pragma: keep


//
// Operator control
//

bool interruptintake = false;
bool interruptDescore = false;
bool interruptDrive = false;
bool overrideDrive = false;
double imu_cur = 0;
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
		if(aligner.get()) aligner.set(false);
		redirect.set(!redirect.get());
	}
}

void setScraperOp() {
	int delayTime = 0;
	if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
		if(!scraper.get()) {
			if(redirect.get()) {
				redirect.set(false);
				delayTime = 200;
			}
			if(aligner.get()) {
				aligner.set(false);
				delayTime = 200;
			}
			pros::delay(delayTime);
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

void setAlignerOp() {
	if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
		if(!redirect.get()) {
			if(scraper.get()) {
				scraper.set(false);
				pros::delay(200);
			}
			aligner.set(!aligner.get());
		}
	}
}

void setStraightOp() {
	if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
		if(util::turn_shortest(imu_cur - 90, chassis.drive_imu_get()) > util::turn_shortest(imu_cur + 90, chassis.drive_imu_get())) imu_cur += 180;
		overrideDrive = !overrideDrive;
		interruptDrive = false;
		sendHaptic("-");
	}
	if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
		imu_cur = chassis.drive_imu_get();
		sendHaptic(". .");
	}
	setStraight(imu_cur);
}

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

void setStraightTeam() {
	if(team.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1) && !interruptDrive) {
		interruptDrive = true;
		sendHaptic("---");
	}
	if(interruptDrive) {
		overrideDrive = team.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
	}
}