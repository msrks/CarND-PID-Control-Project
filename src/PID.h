#include <uWS/uWS.h>

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

  std::vector<double> p;
  std::vector<double> dp;
  int step;
  double total_err;
  double best_err;
  int idx;
  bool rollback;
  bool initialized;
  
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

  double SteerValue();

  void Restart(uWS::WebSocket<uWS::SERVER> ws);

  void Twiddle(uWS::WebSocket<uWS::SERVER> ws);
};

#endif /* PID_H */
