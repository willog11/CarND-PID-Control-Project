#include "PID.h"
#include <limits>
#include <math.h> 

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	Kp = Kp;
	Ki = Ki;
	Kd = Kd;

	p_error = 0;
	i_error = 0;
	d_error = 0;

	best_error = std::numeric_limits<double>::max();
	current_state = 0;
}

void PID::UpdateError(double cte) {

	d_error = cte - p_error;
	i_error += cte;
	p_error = cte;

	Twiddle(cte);
}

double PID::TotalError() {
	return -Kp * p_error - Ki * i_error - Kd * d_error;
}

void PID::Twiddle(double cte, double tol) {
	double dp[3] = { 1.0, 1.0, 1.0 };
	double error = pow(cte, 2);

	if (SumArray(dp) > tol)
	{
		for (int i = 0; i < 3; i++)
		{
			switch (current_state)
			{
				case 0:
					UpdateTau(i, dp[i]);
					current_state = 1;
					break;
				case 1:
					if (error < best_error)
					{
						best_error = error;
						dp[i] *= 1.1;
						current_state = 0;
						break;
					}
					else
					{
						double update_factor = (-2 * dp[i]);
						UpdateTau(i, update_factor);
						current_state = 2;
						break;
					}
				case 2:

						if (error < best_error)
						{
							best_error = error;
							dp[i] *= 1.1;
						}
						else
						{
							UpdateTau(i, dp[i]);
							dp[i] *= 0.9;
						}
						current_state = 0;
						break;
				default:
					break;

			}

		}
	}

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
		Kp += value;

	if (index == 1)
		Ki += value;

	if (index == 2)
		Kd += value;

}
