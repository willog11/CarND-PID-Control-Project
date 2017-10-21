#ifndef PID_H
#define PID_H

#define USE_TWIDDLE FALSE

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


#ifdef USE_TWIDDLE
	/*
	* Use twiddle implementation to update tau variables.
	*/
	void Twiddle(double cte, double tol = 0.2);
#endif

private:

#ifdef USE_TWIDDLE
	double SumArray(double* p);
	void UpdateTau(int index, double value);

	double best_error;
	double total_error;
	int current_state;
#endif

};

#endif /* PID_H */
