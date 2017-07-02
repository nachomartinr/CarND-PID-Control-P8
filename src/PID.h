#ifndef PID_H
#define PID_H

class PID {

public:
  /*
  * Errors
  */
  double p_error_;
  double i_error_;
  double d_error_;

  /*
  * Coefficients
  */ 
  double Kp_;
  double Ki_;
  double Kd_;

  long long iter_;
  double totalError_;

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
  * Update PID params without reseting errors
  */
  void UpdateParams(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the command value.
  */

  double GetCommand();

  /*
  * Calculate the total error
  */
  void GetTotalError(double &totalError, double &averageError);
};

#endif /* PID_H */
