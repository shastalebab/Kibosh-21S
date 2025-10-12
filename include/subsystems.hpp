#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"
#include "pros/distance.hpp"
#include "pros/motors.hpp"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

inline pros::Optical colorSens(19);
inline pros::Optical proximitySens(20);
inline pros::Distance distanceSens(2);

inline pros::Motor intakeNone(21);
inline pros::Motor intakeFront(1);
inline pros::Motor intakeSecond(10);
inline pros::Motor intakeBack(-8);
inline ez::Piston scraper('A');
inline ez::Piston descore('B');
inline ez::Piston indexer('C');
inline ez::Piston redirect('D');
inline ez::Piston park('E');

class Jammable {
   private:
	int clock = 0;

   public:
	vector<pros::Motor*> motors;
	int* target;
	int limit;
	int attempts;
	float maxTemp;
	bool ignoreSort;
	bool pause;

	bool lock;
	void checkJam();

	Jammable() {
		motors = {&intakeNone};
		target = nullptr;
		attempts = 20;
		limit = 4;
		maxTemp = 55;
		ignoreSort = true;
		pause = false;
		lock = false;
	}
	Jammable(vector <pros::Motor*> Motors, int* Target, int Limit, int Attempts, float MaxTemp, bool IgnoreSort, bool Pause) {
		motors = Motors;
		target = Target;
		attempts = Attempts;
		limit = Limit;
		maxTemp = MaxTemp;
		ignoreSort = IgnoreSort;
		pause = Pause;
		lock = false;
	}
};

enum Colors { BLUE = 0, NEUTRAL = 1, RED = 2 };

extern Colors allianceColor;
extern Jammable none;
extern Jammable front;
extern Jammable back;

extern Jammable* targetMotor;

bool shift();

void setIntake(int first_speed, int second_speed, bool indexer_up, bool redirect_up);
void setIntake(int first_speed, int second_speed, bool redirect_up);
void setIntake(int first_speed, int second_speed);
void setScraper(bool state);
void setDescore(bool state);
void setPark(bool state);

void setAlliance(Colors alliance);
void colorToggle();
void colorSet(Colors color, lv_obj_t* object);

void sendHaptic(string input);

void setIntakeOp();
void setScraperOp();
void setDescoreOp();
void setParkOp();

void colorTask();
void antiJamTask();
void controllerTask();