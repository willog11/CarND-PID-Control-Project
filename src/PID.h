#ifndef PID_H
#define PID_H

class PID {
public:
	/*
	* Errors
	*/
	double p_error;
	double i_error;
	double d_error;

	/*
	* Coefficients
	*/ 
	double Kp;
	double Ki;
	double Kd;

	/*
	* Constructor
	*/
	PID();

	/*
	* Destructor.
	*/
	virtual ~PID();

	/*
	* Initialize PID.
	*/
	void Init(double Kp, double Ki, double Kd);

	/*
	* Update the PID error variables given cross track error.
	*/
	void UpdateError(double cte);

	/*
	* Calculate the total PID error.
	*/
	double TotalError();

	/*
	* Update the PID error variables given cross track error.
	*/
	void Twiddle(double cte, double tol = 0.2);

private:
	double SumArray(double* p);
	void UpdateTau(int index, double value);

	double best_error;
	int current_state;
};

#endif /* PID_H */
