#include "PID.h"
#include <cmath>
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {

	Kp_ = Kp;
  Ki_ = Ki; 
  Kd_ = Kd;
	p_error_ = 0.0;
	i_error_ = 0.0;
	d_error_ = 0.0;

  totalError_ = 0.0;

}

void PID::UpdateParams(double Kp, double Ki, double Kd) {
  Kp_ = Kp;
  Ki_ = Ki; 
  Kd_ = Kd;
}

void PID::UpdateError(double cte) {

  d_error_ = cte - p_error_;
  p_error_ = cte;
  i_error_ += cte;

}

double PID::GetCommand() {

  double control = (-(Kp_ * p_error_)) - (Kd_ * d_error_) - (Ki_ * i_error_);


  if (fabs(control) > 1.0) {
    control = copysign(1.0, control);
  }

  return control;
}

void PID::GetTotalError(double &totalError, double &averageError) {

  ++iter_;
  totalError_ += (p_error_ * p_error_);
  totalError = totalError_;
  averageError = totalError_ / iter_;

}

