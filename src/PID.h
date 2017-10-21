#ifndef PID_H
#define PID_H

#if 0
	#define USE_TWIDDLE
#endif

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
	/*
	* Calculate the sum of the array.
	*/
	double SumArray(double* p);

	/*
	* Update the tau values according to their index.
	*/
	void UpdateTau(int index, double value);

	/*
	* Twiddle variables.
	*/
	double best_error;
	double total_error;
	int current_state;
#endif

};

#endif /* PID_H */
