#include "main.h"  // IWYU pragma: keep

//
// Controller event handler & related functions
//

void sendHaptic(string input) { controllerInput = input; }

void masterControllerTask() {
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
				if(controllerInput != "") {
					master.rumble(pattern.c_str());
					controllerInput = "";
					pattern = "";
				}
				timer++;
			}
			pros::delay(50);

			// Update temperature variables and print to controller
            tempDrive = 0;
            tempIntake = 0;
            
			for(auto motor : chassis.left_motors) tempDrive += motor.get_temperature();
			for(auto motor : chassis.right_motors) tempDrive += motor.get_temperature();
			tempDrive /= (chassis.left_motors.size() + chassis.right_motors.size());

			tempIntake = (intakeFront.motors[0]->get_temperature() + intakeBack.motors[0]->get_temperature()) / 2;

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
			pros::c::controller_print(pros::E_CONTROLLER_MASTER, 2, 0, (auton_sel.selector_name + "        ").c_str());
		}
		pros::delay(50);
	}
}

void teamControllerTask() {

}