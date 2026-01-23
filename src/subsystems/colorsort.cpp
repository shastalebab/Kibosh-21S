#include "main.h"  // IWYU pragma: keep

//
// Color Sort
//

Colors allianceColor = NEUTRAL;
Sorting primed = UNPRIMED;
void setAlliance(Colors alliance) { allianceColor = alliance; }

void colorToggle() {
	if(allianceColor == RED)
		allianceColor = BLUE;
	else if(allianceColor == BLUE)
		allianceColor = RED;
	else
		allianceColor = NEUTRAL;
	colorSet(allianceColor, allianceInd);
}

void setSortPrime(Sorting state) {
	primed = state;
}

Sorting getSortPrime() { return primed; }

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
	if(proximitySens.get_proximity() > 50 && proximitySens.get_proximity() <= 255) {
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
	while(true) {
		colorSens.set_integration_time(10);
		proximitySens.set_integration_time(10);
		colorSens.set_led_pwm(50);
		proximitySens.set_led_pwm(50);
		color = colorGet();
		colorSet(color, colorInd);
		if(!pros::competition::is_disabled() && colorCompare(color)) {
			if(primed == PRIMED) {
					setIntake(intakeFront.target, 0);
					primed = UNPRIMED;
                    pros::delay(250);
			} else if(primed == DELAYED) {
					delayMillis(400);
					setIntake(intakeFront.target, 0);
					primed = UNPRIMED;
                    pros::delay(250);
			}
		}
		pros::delay(10);
	}
}
