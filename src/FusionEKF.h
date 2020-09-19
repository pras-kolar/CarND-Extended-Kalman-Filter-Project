#ifndef FusionEKF_H_
#define FusionEKF_H_

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "kalman_filter.h"
#include "tools.h"

class FusionEKF {
public:
  /**
  * Constructor.
  */
  FusionEKF();

  /**
  * Destructor.
  */
  virtual ~FusionEKF();

  /**
  * Kalman Filter execution flow starts here.
  */
  void ProcessMeasurement(const MeasurementPackage &measurement_pack);

  /**
  * Declaration of Kalman Filter update and prediction
  */
  KalmanFilter ekf_;

private:
  // measurement initialized or not (first measurement) ?
  bool is_initialized_;

  // previous timestamp
  long long previous_timestamp_;

  // declaration to compute Jacobian and RMSE
  Tools tools;
  Eigen::MatrixXd R_laser_;
  Eigen::MatrixXd R_radar_;
  Eigen::MatrixXd H_laser_;
  Eigen::MatrixXd Hj_;
};

#endif /* FusionEKF_H_ */