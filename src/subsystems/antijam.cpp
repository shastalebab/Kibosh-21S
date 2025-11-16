#include "main.h"  // IWYU pragma: keep

//
// Anti-jam
//

Jammable intakeFront = Jammable({&intakeFirst}, 30, 10, 200, 55, false);
Jammable intakeBack = Jammable({&intakeSecond}, 20, 5, 80, 55, false);

void Jammable::checkJam() {
	double currentTemp = 0.0;
	double currentVelocity = 600.0;
	int it = 0;

	for(auto motor : this->motors) {
		currentTemp += motor->get_temperature();
		if(motor->get_actual_velocity() < currentVelocity) currentVelocity = motor->get_actual_velocity();
		it++;
	}

	currentTemp /= it;
	currentVelocity /= it;

	if(currentTemp > this->maxTemp) return;

	if(abs(this->target) > 0 && abs(currentVelocity) < this->limit) {
		this->clock++;
		if(this->clock > this->attempts) {
			if(this->pause) {
				this->lock = true;
				for(auto motor : this->motors) {
					motor->move(0);
				}
			} else {
				this->lock = true;
				for(auto motor : this->motors) {
					motor->move(-(this->target));
				}
				pros::delay(this->delayTime);
				for(auto motor : this->motors) {
					motor->move(this->target);
				}
				this->lock = false;
			}
			this->clock = 0;
		}
	} else
		this->clock = 0;
}

void antiJamTask() {
	while(true) {
		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1) || master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
			pros::delay(200);
		}
		intakeFront.checkJam();
        intakeBack.checkJam();
		pros::delay(10);
	}
}