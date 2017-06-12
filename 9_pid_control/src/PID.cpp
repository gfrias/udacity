#include "PID.h"

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    p_error = 0.0;
    d_error = 0.0;
    i_error = 0.0;
    iterations = 0;
  
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    
}

void PID::UpdateError(double cte) {
  if (iterations >= 30) {
    iterations = 0;
    i_error = 0.0;
  }
  
    i_error += cte;
    d_error = cte - p_error;
    p_error = cte;
    total_error += (cte * cte);
}

double PID::TotalError() {
    return total_error;
}

