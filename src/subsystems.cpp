#include "main.h"  // IWYU pragma: keep

// Internal targets to aid tasks
Colors allianceColor = NEUTRAL;
int frontTarget = 0;
int backTarget = 0;
bool inputLock = false;
bool jamDelay = false;

// Complex motors
Jammable none = Jammable({&intakeNone}, &frontTarget, 20, 50, 50, false, false);
Jammable front = Jammable({&intakeFront}, &frontTarget, 20, 50, 60, true, false);
Jammable back = Jammable({&intakeBack}, &backTarget, 20, 50, 50, false, false);

Jammable* targetMotor = &back;

//
// Wrappers & Utility functions
//

void setIntake(int first_speed, int second_speed, bool indexer_up, bool redirect_up) {
	if(autonMode != BRAIN) {
		indexer.set(indexer_up);
		redirect.set(redirect_up);
		setIntake(first_speed, second_speed);
	}
}

void setIntake(int first_speed, int second_speed, bool redirect_up) {
	if(autonMode != BRAIN) {
		redirect.set(redirect_up);
		setIntake(first_speed, second_speed);
	}
}

void setIntake(int first_speed, int second_speed) {
	if(autonMode != BRAIN) {
		if(front.lock != true) {
			for(auto motor : front.motors) {
				motor->move(first_speed);
			}
		}
		if(back.lock != true) {
			for(auto motor : back.motors) {
				motor->move(second_speed);
			}
			if(inputLock)
				for(auto motor : targetMotor->motors) {
					motor->move(-*targetMotor->target);
				}
			frontTarget = first_speed;
			backTarget = second_speed;
		}
	}
}

void setScraper(bool state) {
	if(autonMode != BRAIN) {
		scraper.set(state);
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

void setAlliance(Colors alliance) { allianceColor = alliance; }

void sendHaptic(string input) { controllerInput = input; }

//
// Operator control
//

bool shift() { return master.get_digital(pros::E_CONTROLLER_DIGITAL_R2); }

void setIntakeOp() {
	if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1) || master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1) ||
	   master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2))
		jamDelay = true;
	if(shift()) {
		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {	 // short score
			setIntake(127, -127, true, false);
		} else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {	// low goal evil scoring
			setIntake(-127, -127);
		} else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {	// mid goal scoring
			setIntake(90, 90, false, true);
		} else {
			setIntake(0, 0, false);
			front.lock = false;
			back.lock = false;
		}
	} else {
		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {	 // storing
			setIntake(127, 127, true, false);
		} else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {	// low goal
			setIntake(-90, -90);
		} else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {	// top goal scoring
			setIntake(127, 127, false, true);
		} else {
			setIntake(0, 0, false);
			front.lock = false;
			back.lock = false;
		}
	}

	if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
		colorToggle();
		cout << getDistanceActual() << "\n";
		sendHaptic("-");
	}
	if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) sendHaptic(".");
}

void setScraperOp() { scraper.button_toggle(master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)); }

void setDescoreOp() { descore.button_toggle(master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)); }

void setParkOp() { park.button_toggle(master.get_digital(pros::E_CONTROLLER_DIGITAL_A)); }

//
// Color sort
//

void colorToggle() {
	if(allianceColor == RED)
		allianceColor = BLUE;
	else if(allianceColor == BLUE)
		allianceColor = RED;
	else
		allianceColor = NEUTRAL;
	colorSet(allianceColor, allianceInd);
}

void colorSet(Colors color, lv_obj_t* object) {
	// Set on screen elements to the corresponding color
	lv_color32_t color_use = theme_accent;
	if(color == RED)
		color_use = red;
	else if(color == BLUE)
		color_use = blue;
	lv_obj_set_style_bg_color(object, color_use, LV_PART_MAIN);
}

Colors colorGet() {
	double hue = 0;
	if(proximitySens.get_proximity() > 150 && proximitySens.get_proximity() <= 255) {
		hue = colorSens.get_hue();
		if((hue > 340 && hue < 360) || (hue > 0 && hue < 20))
			return RED;
		else if(hue > 210 && hue < 300)
			return BLUE;
	}
	return NEUTRAL;
}

bool colorCompare(Colors color) {
	if((int)allianceColor != 1 && (int)color != 1) return allianceColor != color;
	return false;
}

void colorTask() {
	Colors color;
	int sortTime = 0;
	bool sleep = false;
	while(true) {
		colorSens.set_integration_time(10);
		proximitySens.set_integration_time(10);
		colorSens.set_led_pwm(50);
		proximitySens.set_led_pwm(50);
		color = colorGet();
		colorSet(color, colorInd);
		if(!pros::competition::is_disabled()) {
			if(colorCompare(color) && !sleep) {
				if(sortTime < 10) {
					inputLock = true;
					for(auto motor : targetMotor->motors) {
						motor->move((-*targetMotor->target));
					}
					sortTime++;
					pros::delay(180);
				} else {
					sleep = true;
					for(auto motor : targetMotor->motors) {
						motor->move(*targetMotor->target);
					}
				}
			} else {
				sortTime = 0;
				sleep = false;
				inputLock = false;
			}
		}
		pros::delay(10);
	}
}

//
// Anti-jam
//

void Jammable::checkJam() {
	double currentTemp = 0.0;
	double currentVelocity = 0.0;
	int it = 0;

	for(auto motor : this->motors) {
		currentTemp += motor->get_temperature();
		currentVelocity += motor->get_actual_velocity();
		it++;
	}

	currentTemp /= it;
	currentVelocity /= it;

	if(currentTemp > this->maxTemp) return;

	if(this->ignoreSort == false && inputLock == true) return;

	if(abs(*(this->target)) > 0 && abs(currentVelocity) < this->limit) {
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
					motor->move(-(*(this->target)));
				}
				pros::delay(100);
				for(auto motor : this->motors) {
					motor->move(*(this->target));
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
		if(jamDelay) {
			pros::delay(200);
			jamDelay = false;
		}
		front.checkJam();
		back.checkJam();
		pros::delay(10);
	}
}

//
// Controller event handler
//

void controllerTask() {
	string pattern = "";
	int timer = 0;
	float tempDrive;
	float tempIntake;
	int selected_k = 0;
	while(true) {
		if(lv_tileview_get_tile_act(main_tv) == pidTuner && !pros::competition::is_connected()) {
			PID::Constants constants = selectedTabObj->pid_targets.pid->constants_get();

			if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
				selected_k++;
				selected_k %= 3;
			} else if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
				pros::c::controller_clear(pros::E_CONTROLLER_MASTER);
				pros::delay(50);
				pros::c::controller_print(pros::E_CONTROLLER_MASTER, 1, 0, "%s      ", selectedTabObj->name.c_str());
				pros::delay(2000);
			} else if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
				lv_event_send(selectedTabObj->graph, LV_EVENT_CLICKED, NULL);
			} else {
				if(master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
					switch(selected_k) {
						default:
						case 0:
							constants.kp += selectedTabObj->pid_targets.kp;
							break;
						case 1:
							constants.ki += selectedTabObj->pid_targets.ki;
							break;
						case 2:
							constants.kd += selectedTabObj->pid_targets.kd;
							break;
					}
				} else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
					switch(selected_k) {
						default:
						case 0:
							constants.kp -= selectedTabObj->pid_targets.kp;
							if(constants.kp < 0) constants.kp = 0;
							break;
						case 1:
							constants.ki -= selectedTabObj->pid_targets.ki;
							if(constants.ki < 0) constants.ki = 0;
							break;
						case 2:
							constants.kd -= selectedTabObj->pid_targets.kd;
							if(constants.kd < 0) constants.kd = 0;
							break;
					}
				}
			}

			selectedTabObj->pid_targets.pid->constants_set(constants.kp, constants.ki, constants.kd);

			ofstream pidValues;

			if(pros::usd::is_installed()) {
				pidValues.open("/usd/constants.txt", ios::out | ios::trunc);

				if(pidValues.is_open()) {
					for(auto i : tabList) {
						if(i.usePid) {
							pidValues << "( kp: " << i.pid_targets.pid->constants_get().kp << " ki: " << i.pid_targets.pid->constants_get().ki
									  << " kd: " << i.pid_targets.pid->constants_get().kd << " )\n";
						}
					}
					pidValues.close();
				}
			}

			pros::c::controller_print(pros::E_CONTROLLER_MASTER, 0, 0, "%c kp: %.2f     ", selected_k == 0 ? '>' : ' ', constants.kp);
			pros::delay(50);
			pros::c::controller_print(pros::E_CONTROLLER_MASTER, 1, 0, "%c ki: %.2f     ", selected_k == 1 ? '>' : ' ', constants.ki);
			pros::delay(50);
			pros::c::controller_print(pros::E_CONTROLLER_MASTER, 2, 0, "%c kd: %.2f     ", selected_k == 2 ? '>' : ' ', constants.kd);
		} else {
			// Update timer and rumble controller
			if(!pros::competition::is_autonomous() && !pros::competition::is_disabled()) {
				if(pattern == "") {
					if(timer == 475)
						pattern = "";  // "- -"
					else if(timer >= 500 && timer < 525)
						pattern = "";  // "."
					else
						pattern = controllerInput;
				}
				if(timer % 5 == 0 || controllerInput != "") {
					master.rumble(pattern.c_str());
					controllerInput = "";
					pattern = "";
				}
				timer++;
			}
			pros::delay(50);

			// Update temperature variables and print to controller
			tempDrive = (chassis.left_motors[0].get_temperature() + chassis.left_motors[1].get_temperature() + chassis.left_motors[2].get_temperature() +
						 chassis.right_motors[0].get_temperature() + chassis.right_motors[1].get_temperature() + chassis.right_motors[2].get_temperature()) /
						6;
			tempIntake = (intakeFront.get_temperature() + intakeSecond.get_temperature() + intakeBack.get_temperature()) / 3;

			if(tempDrive <= 30)
				pros::c::controller_print(pros::E_CONTROLLER_MASTER, 0, 0, "drive: cool, %.0f°C     ", tempDrive);
			else if(tempDrive > 30 && tempDrive <= 50)
				pros::c::controller_print(pros::E_CONTROLLER_MASTER, 0, 0, "drive: warm, %.0f°C     ", tempDrive);
			else if(tempDrive > 50)
				pros::c::controller_print(pros::E_CONTROLLER_MASTER, 0, 0, "drive: hot, %.0f°C     ", tempDrive);
			pros::delay(50);

			if(tempIntake <= 30)
				pros::c::controller_print(pros::E_CONTROLLER_MASTER, 1, 0, "intke: cool, %.0f°C     ", tempIntake);
			else if(tempIntake > 30 && tempIntake <= 50)
				pros::c::controller_print(pros::E_CONTROLLER_MASTER, 1, 0, "intke: warm, %.0f°C     ", tempIntake);
			else if(tempIntake > 50)
				pros::c::controller_print(pros::E_CONTROLLER_MASTER, 1, 0, "intke: hot, %.0f°C     ", tempIntake);
			pros::delay(50);

			// Print selected auton to controller
			if(!(!pros::competition::is_autonomous() && !pros::competition::is_disabled()))
				pros::c::controller_print(pros::E_CONTROLLER_MASTER, 2, 0, (auton_sel.selector_name + "        ").c_str());
		}
		pros::delay(50);
	}
}