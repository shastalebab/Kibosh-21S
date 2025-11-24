#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"
#include "pros/distance.hpp"
#include "pros/motors.hpp"

extern Drive chassis;
extern pros::Controller team;
extern bool overrideDrive;

// Your motors, sensors, etc. should go here.  Below are examples

inline pros::Optical colorSens(2);
inline pros::Optical proximitySens(3);
inline pros::Distance distanceSens(8);

inline pros::Motor intakeFirst(-1);
inline pros::Motor intakeSecond(10);
inline ez::Piston scraper('D');
inline ez::Piston wing('B');
inline ez::Piston indexer('H');
inline ez::Piston descore('G');
inline ez::Piston redirect('E');
inline ez::Piston aligner('F');
inline ez::Piston brakes('G');

inline ez::PID driveHeading(6.25, 0.1, 39.25);

class Jammable {
   private:
	int clock = 0;

   public:
	vector<pros::Motor*> motors;
	int target;
	int limit;
	int attempts;
	int delayTime;
	float maxTemp;
	bool pause;

	bool lock;
	void checkJam();

	Jammable() {
		motors = {};
		target = 0;
		attempts = 20;
		limit = 4;
		delayTime = 100;
		maxTemp = 55;
		pause = false;
		lock = false;
	}
	Jammable(vector <pros::Motor*> Motors, int Limit, int Attempts, int DelayTime, float MaxTemp, bool Pause) {
		motors = Motors;
		attempts = Attempts;
		limit = Limit;
		delayTime = DelayTime;
		maxTemp = MaxTemp;
		pause = Pause;
		lock = false;
	}
};

enum Colors { BLUE = 0, NEUTRAL = 1, RED = 2 };

extern Colors allianceColor;
extern Jammable intakeFront;
extern Jammable intakeBack;

bool shift();

void setIntake(int speed, bool indexer_on);
void setIntake(int speed);

void setRedirect(bool state);
void setScraper(bool state);
void setWing(bool state);
void setDescore(bool state);
void setAligner(bool state);
void setBrakes(bool state);

void setAlliance(Colors alliance);
void colorToggle();
void colorSet(Colors color, lv_obj_t* object);

void sendHaptic(string input);

void setStraight(int init_heading);

void setIntakeOp();
void setRedirectOp();
void setScraperOp();
void setWingOp();
void setDescoreOp();
void setAlignerOp();
void setStraightOp();

void setIntakeTeam();
void setDescoreTeam();
void setBrakesTeam();
void setStraightTeam();

void colorTask();
void antiJamTask();
void masterControllerTask();
void teamControllerTask();