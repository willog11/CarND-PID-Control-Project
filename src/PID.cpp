#include "PID.h"
#include <limits>
#include <math.h> 
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd, bool twiddle_enabled) {
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;
	this->twiddle_enabled = twiddle_enabled;

	p_error = 0;
	i_error = 0;
	d_error = 0;

	best_error = std::numeric_limits<double>::max();
	total_error = 0.0;
	current_state = 0;

	dp[0] = this->Kp * 0.1;
	dp[1] = this->Ki * 0.1;
	dp[2] = this->Kd * 0.1;

	twiddle_index = 0; // Tau P
	eval_steps = 100;
	steps = 0;
}

void PID::UpdateError(double cte) {

	// Update PID errors
	d_error = cte - p_error;
	i_error += cte;
	p_error = cte;

	if (twiddle_enabled)
		Twiddle(cte);
}

double PID::TotalError() {
	// Calculate PID value
	return -Kp * p_error - Ki * i_error - Kd * d_error;
}



void PID::Twiddle(double cte, double tol) {

	if (steps % eval_steps >= eval_steps)
		total_error += pow(cte, 2);

	switch (current_state)
	{
		case 0:
			UpdateTau(twiddle_index, dp[twiddle_index]);
			current_state = 1;
			break;
		case 1:
			if (total_error < best_error)
			{
				// Improvement made
				best_error = total_error;
				dp[twiddle_index] *= 1.1;
				current_state = 0;
				break;
			}
			else
			{
				// No improvement - subtract
				double update_factor = (-2 * dp[twiddle_index]);
				UpdateTau(twiddle_index, update_factor);
				current_state = 2;
				break;
			}
		case 2:
				if (total_error < best_error)
				{
					// Improvement made
					best_error = total_error;
					dp[twiddle_index] *= 1.1;
				}
				else
				{
					// No improvement - add back
					UpdateTau(twiddle_index, dp[twiddle_index]);
					dp[twiddle_index] *= 0.9;
				}
				current_state = 0;
				break;
		default:
			break;
	}

	step++;
	
	cout << "Twiddle::() Best error: " << best_error << "Total error: " << total_error << endl;

}

double PID::SumArray(double* p) {
	double sum = 0;

	for (int i = 0; i < 3; i++)
	{
		sum += p[i];
	}
	return sum;
}

void PID::UpdateTau(int index, double value) {
	if (index == 0)
	{
		Kp += value;
		cout << "UpdateTau::() Kp: " << Kp << endl;
	}
	if (index == 1)
	{
		Ki += value;
		cout << "UpdateTau::() Ki: " << Ki << endl;
	}
	if (index == 2)
	{
		Kd += value;
		cout << "UpdateTau::() Kd: " << Kd << endl;
	}
}

