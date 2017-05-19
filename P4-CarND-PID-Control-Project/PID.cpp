#include "PID.h"
#include <iostream>
#include <math.h>
using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;

	this->p_error = 0;
	this->i_error = 0;
	this->d_error = 0;

	this->output_value = 0;
	this->step_counter = 0;
	this->max_steps = 500;
	this->tolerance = 0.2;
	this->total_err = 0;
	this->best_err = 1000;

	this->Dpp = 0.9;
  	this->Dpi = 0.9;
  	this->Dpd = 1;

  	this->increasing = true;
  	this->cnt = 0;
}

void PID::UpdateError(double cte) {
	d_error = cte - p_error;
	p_error = cte;
	i_error += cte;

	output_value = -Kp*p_error - Kd*d_error - Ki*i_error;
	output_value = output_value<-1?-1:output_value;
	output_value = output_value>1?1:output_value;

	// cout << "p_error = " << p_error	<< ", d_error = " << d_error << ", i_error = " << i_error << endl;
	// cout << "Kp = " << Kp << ", Kd = " << Kd << ", Ki = " << Ki << endl;
	cout << "step_counter = " << step_counter << endl;
	cout << "Kp = " << Kp << ", p_error= " << p_error << "=> P = " << -Kp*p_error << endl;
	cout << "Ki = " << Ki << ", i_error= " << i_error << "=> I = " << Ki*i_error << endl;
	cout << "Kd = " << Kd << ", d_error= " << d_error << "=> D = " << Kd*d_error << endl;

	step_counter++;
	if (step_counter >= max_steps/2) {
		total_err += pow(cte, 2);
	} else {
		total_err += cte;
	}
}

double PID::TotalError() {
	return total_err/step_counter;
}

bool PID::Twiddle() {

	bool to_reset = false;
	if (step_counter == max_steps) {
		cout << "############################## TWIDDLING ##############################" << endl;
		double err = TotalError();

		cout << "total_err = " << total_err << endl;
		cout << "best_err = " << best_err << endl;
		cout << "err = " << err << endl;
		cout << "Dpp = " << Dpp << ", Dpi = " << Dpi << ", Dpd = " << Dpd << endl;

		if (err < best_err) {
			cout << "Inside IF Loop-----------------------------------" << cnt << endl;
			switch (cnt % 3) {
				case 0:
					if (Dpp < 1) Dpp *= 1.00001;
					Ki = Ki * Dpi;
					break;
				case 1:
					if (Dpi < 1) Dpi *= 1.00001;
					Kd = Kd * Dpd;
					break;
				case 2:
					if (Dpd < 1) Dpd *= 1.00001;
					Kp = Kp * Dpp;
					break;
			}
			cnt++;
			best_err = err;
			increasing = true;
		} else {
			cout << "Inside ELSE Loop***********************************" << cnt << endl;
			cnt++;
			if (increasing) {
				cout << "Inside INNER IF Loop ###########################" << cnt << endl;
				increasing = false;
				switch (cnt % 3) {
					case 0: Kp = Kp / Dpp*Dpp;
						break;
					case 1: Ki = Ki / Dpi*Dpi;
						break;
					case 2: Kd = Kd / Dpd*Dpd;
						break;
				}
			} else {
				cout << "Inside INNER ELSE Loop @@@@@@@@@@@@@@@@@@@@@@@@@@" << cnt << endl;
				increasing = true;
				switch (cnt % 3) {
					case 0: Kp = Kp * Dpp;
						Ki = Ki *Dpi;
						break;
					case 1: Ki = Ki *Dpi;
						Kd = Kd *Dpd;
						break;
					case 2: Kd = Kd *Dpd;
						Kp = Kp * Dpp;
						break;
				}
			}
		}

		step_counter = 0;
	}
	return to_reset;
}


