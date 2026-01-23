#pragma once

#include "EZ-Template/api.hpp"
#include "EZ-Template/util.hpp"
#include "drive.hpp"

class Stanley {
	private:
	double crosstrack(Coordinate cur, Coordinate min);
	double xy_error(Coordinate cur, Coordinate target);
	double xy_current_fake;
	double new_current_fake;
	double xy_delta_fake;
	double xy_last_fake;
	bool was_stanley_just_set;
	int sgn_init;
   public:
	vector<Coordinate> stanleyPoints = {};
	double ke;
	PID stanleyDrivePID;
	PID stanleyTurnPID;
	slew slewLeft = slew(3, 60);
	slew slewRight = slew(3, 60);
    
    bool active;

	pair<double, double> compute();
	void drive_set_path(vector<Coordinate> points, drive_directions dir, int speed, bool slew);
	void drive_set_point(Coordinate point, drive_directions dir, int speed, bool slew);
    void drive_set(double distance, int speed, bool slew);
	void drive_wait_until(double distance);
    void drive_wait();

	Stanley() {
		stanleyPoints = {};
		stanleyDrivePID = {16.5, 0.0, 170.25};
		stanleyTurnPID = {6.25, 0.1, 39.25};
		slewLeft = ez::slew(3, 60);
		slewRight = ez::slew(3, 60);
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