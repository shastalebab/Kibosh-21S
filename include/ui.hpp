#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"
#include "autons.hpp"
#include "subsystems.hpp"

const lv_color32_t theme_color = lv_color_hex(0x00060a);
const lv_color32_t theme_accent = lv_color_hex(0xe3f4ff);
const lv_color32_t red = lv_color_hex(0xff3643);
const lv_color32_t blue = lv_color_hex(0x01b1f0);
const lv_color32_t green = lv_color_hex(0x22d428);
const lv_color32_t violet = lv_color_hex(0xb672f2);
const lv_color32_t pink = lv_color_hex(0xffade7);
const lv_color32_t gray = lv_color_hex(0x575757);
const lv_color32_t light_gray = lv_color_lighten(gray, 125);

// Auton selector
void angleCheckTask();
void pathViewerTask();

// PID Tuner/Motor Info
void motorUpdateTask();

class AutonObj {
   public:
	function<void()> callback = shift;
	string name = "no name";
	string desc = "no description";
	lv_color32_t color = theme_color;
};

class AutonSel {
   public:
	vector<AutonObj> autons = {};
	string selector_name = "";
	function<void()> selector_callback = shift;
	void selector_populate(vector<AutonObj> auton_list);
};

class PidTunerValues {
   public:
	double kp;
	double ki;
	double kd;
	PID* pid;

	PidTunerValues() {
		kp = 0;
		ki = 0;
		kd = 0;
		pid = nullptr;
	}

	PidTunerValues(double Kp, double Ki, double Kd, PID* Pid) {
		kp = Kp;
		ki = Ki;
		kd = Kd;
		pid = Pid;
	}
};

class MotorDisp {
   public:
	pros::Motor* motor;
	string name;
	lv_color32_t color;
	lv_color32_t bg_color;
	double maxtemp;

	MotorDisp() {
		motor = nullptr;
		name = "";
		color = theme_accent;
		bg_color = theme_color;
		maxtemp = 0;
	}

	MotorDisp(pros::Motor* Motor, double MaxTemp) {
		motor = Motor;
		name = "";
		color = theme_accent;
		bg_color = theme_color;
		maxtemp = MaxTemp;
	}

	MotorDisp(pros::Motor* Motor, string Name, lv_color32_t Color, double MaxTemp) {
		motor = Motor;
		name = Name;
		color = Color;
		bg_color = theme_color;
		maxtemp = MaxTemp;
	}
};

class MotorTab {
   public:
   	string name;
	lv_color32_t color;
	double* error;
	double target;
	vector<MotorDisp> motors;
	function<void(int)> callback;
	bool usePid;
	
	PidTunerValues pid_targets;
	vector<lv_obj_t*> target_objs;

	lv_obj_t* tab;
	lv_obj_t* graph;
	lv_coord_t errorPoints[200];

	void addTab();

	MotorTab() {
		name = "";
		color = theme_color;
		error = nullptr;
		target = 0;
		motors = {};
		callback = drive_test;
		usePid = false;
		pid_targets = {};
		tab = nullptr;
		graph = nullptr;
		target_objs = {};
	}

	MotorTab(string Name, lv_color32_t Color, double* Error, double Target, vector<MotorDisp> Motors, function<void(int)> Callback, bool UsePid, PidTunerValues Pid_Targets, lv_obj_t* Tab) {
		name = Name;
		color = Color;
		error = Error;
		target = Target;
		motors = Motors;
		callback = Callback;
		usePid = UsePid;
		pid_targets = Pid_Targets;
		tab = Tab;
		graph = nullptr;
		target_objs = {};
	}
};

extern string controllerInput;
extern AutonSel auton_sel;

// Main UI
extern lv_obj_t* main_tv;
extern lv_obj_t* autoSelector;
extern lv_obj_t* pidTuner;
extern lv_obj_t* colorInd;
extern lv_obj_t* allianceInd;

// PID Tuner/Motor Info
extern vector<MotorTab> tabList;
extern MotorTab* selectedTabObj;
extern bool probing;

void uiInit();

// Auton Selector UI
void autoSelectorInit();
void pidTunerInit();