#include "main.h"  // IWYU pragma: keep

//
// Wrappers & Utility functions
//

void setIntake(int speed, bool indexer_on) {
	if(autonMode != BRAIN) {
		bool use_delay = false;
		if(!indexer_on && indexer.get()) use_delay = true;

		indexer.set(indexer_on);
		if(use_delay) pros::delay(250);
		setIntake(speed);
	}
}

void setIntake(int speed) {
	if(autonMode != BRAIN) {
		if(intakeFront.lock != true) intakeFront.motors[0]->move(speed);
        if(intakeBack.lock != true) intakeBack.motors[0]->move(speed);
		intakeFront.target = speed;
        intakeBack.target = speed;
	}
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

void setPark(bool state) {
	if(autonMode != BRAIN) {
		park.set(state);
	}
}

void setBrakes(bool state) {
    if(autonMode != BRAIN) {
		brakes.set(state);
	}
}