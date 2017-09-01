#include "PID.h"
#include <math.h>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {
  p = {0, 0, 0};
  dp = {0, 0, 0};
  best_err = std::numeric_limits<double>::max();
  idx = 0;
  rollback = false;
  initialized = false;
}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  if (!initialized) {
    p[0] = Kp;
    p[1] = Ki;
    p[2] = Kd;
    dp[0] = p[0]*0.1;
    dp[1] = p[1]*0.1;
    dp[2] = p[2]*0.1;
    initialized = true;
  }
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  p_error = d_error = i_error = 0.0;

  step = 0;
  total_err = 0.0;
}

void PID::UpdateError(double cte) {
  if (step == 0) {
    p_error = cte;
  }
  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;
  if (step > 300) {
    total_err += pow(cte,2);
  }
  step++;
}

double PID::TotalError() {
  return total_err / (step - 300);
}

double PID::SteerValue() {
  double steer = -Kp * p_error - Kd * d_error - Ki * i_error;
  steer = steer < -1 ? -1 : steer;
  steer = steer > 1 ? 1 : steer;
  return steer;
}

void PID::Restart(uWS::WebSocket<uWS::SERVER> ws) {
  std::string reset_msg = "42[\"reset\",{}]";
  ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
}

void PID::Twiddle(uWS::WebSocket<uWS::SERVER> ws) {
  if (step < 6000) return;
  double err = TotalError();
  // debug
  std::cout << "p: [" << p[0] << ", " << p[1] << ", " << p[2] << "]" << std::endl;
  std::cout << "pd: [" << dp[0] << ", " << dp[1] << ", " << dp[2] << "]" << " ";
  std::cout << "best_err: " << best_err << " " << "err: " << err << std::endl;
  // first iter
  if (best_err == std::numeric_limits<double>::max()) {
    best_err = err;
    p[0] += dp[0];
    Init(p[0], p[1], p[2]);
    Restart(ws);
    return;
  }
  // twiddle
  if (err < best_err) {
    best_err = err;
    dp[idx] *= 1.1;
    rollback = false;
  } else {
    if (rollback) {
      p[idx] += dp[idx];
      dp[idx] *= 0.9;
      rollback = false;
    } else {
      p[idx] -= 2 * dp[idx];
      rollback = true;
    }
  }
  if (!rollback) {
    idx = (idx +1) % 3;
    p[idx] += dp[idx];
  }
  Init(p[0], p[1], p[2]);
  Restart(ws);
}
