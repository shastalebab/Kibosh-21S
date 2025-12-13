#pragma once

#include "EZ-Template/api.hpp"
#include "EZ-Template/util.hpp"
#include "drive.hpp"

class Stanley {
   public:
	vector<Coordinate> stanleyPoints = {};
	double ke;
	PID stanleyDrivePID;
	PID stanleyTurnPID;
	slew slewLeft;
	slew slewRight;
    
    bool active;

	pair<double, double> compute();
	void drive_set_point(Coordinate point, drive_directions dir, int speed, bool slew);
    void drive_set(double distance, int speed, bool slew);
	void drive_wait_until(double distance);
    void drive_wait();

	Stanley() {
		stanleyPoints = {};
		stanleyDrivePID = {16.5, 0.0, 170.25};
		stanleyTurnPID = {6.25, 0.1, 39.25};
		slewLeft = {3, 70};
		slewRight = {3, 70};
		ke = 3.0;
        active = false;
	}

	Stanley(PID drive_pid, PID turn_pid, slew slew, double k_e) {
		stanleyPoints = {};
		stanleyDrivePID = drive_pid;
		stanleyTurnPID = turn_pid;
		slewLeft = slew;
		slewRight = slew;
		ke = k_e;
        active = false;
	}
};

void stanleyTask();

extern Stanley stanley;