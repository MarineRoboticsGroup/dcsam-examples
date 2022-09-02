/**
 * PosePointFactor.h
 * A simple factor representing the cost:
 *    f(T) : = ||Tp - q||^2
 * where p,q \in R^3 and T \in SE(3)
 */
#pragma once

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

class PosePointFactor : public gtsam::NonlinearFactor {
 private:
  gtsam::Point3 p_;
  gtsam::Point3 q_;

 public:
  PosePointFactor(const gtsam::Key &k, const gtsam::Point3 &p,
                  const gtsam::Point3 &q)
      : p_(p), q_(q) {
    keys_.push_back(k);
  }

  gtsam::Vector error_vector(const gtsam::Values &values) const {
    gtsam::Pose3 T = values.at<gtsam::Pose3>(keys_[0]);
    gtsam::Point3 transformed_p = T.transformFrom(p_);
    gtsam::Point3 delta = transformed_p - q_;
    return gtsam::Vector3(delta.x(), delta.y(), delta.z());
  }

  double error(const gtsam::Values &values) const override {
    gtsam::Pose3 T = values.at<gtsam::Pose3>(keys_[0]);
    gtsam::Point3 transformed_p = T.transformFrom(p_);
    gtsam::Point3 delta = transformed_p - q_;

    // Return the squared norm of the error.
    return pow(delta.norm(), 2);
  }

  size_t dim() const override { return 3; };

  boost::shared_ptr<gtsam::GaussianFactor> linearize(
      const gtsam::Values &values) const override {
    gtsam::Pose3 T = values.at<gtsam::Pose3>(keys_[0]);
    Eigen::Matrix<double, 3, 6> H;
    gtsam::Point3 transformed_p = T.transformFrom(p_, &H);
    return boost::make_shared<gtsam::JacobianFactor>(keys_[0], H,
                                                     -error_vector(values));
  }
};
