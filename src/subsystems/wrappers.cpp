#include "main.h"  // IWYU pragma: keep

//
// Wrappers & Utility functions
//

void setIntake(int speed, bool indexer_on) {
	if(autonMode != BRAIN) {
		bool use_delay = false;
		if(!indexer_on && indexer.get()) use_delay = true;

		indexer.set(indexer_on);
		if(use_delay) {
			setIntake(0);
			pros::delay(250);
		}
		setIntake(speed);
	}
}

void setIntake(int front, int back) {
	if(autonMode != BRAIN) {
		if(intakeFront.lock != true) intakeFront.motors[0]->move(front);
		if(intakeBack.lock != true) intakeBack.motors[0]->move(back);
		intakeFront.target = front;
		intakeBack.target = back;
	}
}

void setIntake(int speed) {
	setIntake(speed, speed);
}

void setRedirect(bool state) {
	if(autonMode != BRAIN) {
		redirect.set(state);
	}
}

void setScraper(bool state) {
	if(autonMode != BRAIN) {
		scraper.set(state);
	}
}

void setWing(bool state) {
	if(autonMode != BRAIN) {
		wing.set(state);
	}
}

void setDescore(bool state) {
	if(autonMode != BRAIN) {
		descore.set(state);
	}
}

void setAligner(bool state) {
	if(autonMode != BRAIN) {
		aligner.set(state);
	}
}

void setBrakes(bool state) {
	if(autonMode != BRAIN) {
		brakes.set(state);
	}
}

int cooldown = 0;
int headingMod = 0;
int iter = 0;

void setStraight(int init_heading) {
	if(overrideDrive) {
		double stick_left = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		double stick_right = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
		double input = (stick_left + stick_right) / 2;

		if(cooldown == 0 && stick_left != 0 && input == 0) iter++;
		if(cooldown > 0) cooldown--;

		if(iter > 10) {
			headingMod += (45 * util::sgn(stick_left));
			cooldown = 10;
			iter = 0;
		}

		driveHeading.target_set(util::turn_shortest(init_heading + headingMod, chassis.drive_imu_get()));
		double correction = driveHeading.compute(chassis.drive_imu_get());

		double l_out = input + correction;
		double r_out = input - correction;

		chassis.drive_set(l_out, r_out);
	} else
		headingMod = 0;
}