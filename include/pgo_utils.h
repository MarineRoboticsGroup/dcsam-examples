#pragma once

#include <dcsam/DCMixtureFactor.h>
#include <dcsam/DCSAM.h>
#include <dcsam/DiscretePriorFactor.h>
#include <dcsam/HybridFactorGraph.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/GncOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/dataset.h>

#include <chrono>
#include <iostream>
#include <random>

// Generate incorrect loop closures

/**
 * A factor is considered a loop closure if it is between two non-consecutive
 * poses.
 */
bool isLoopClosure(const gtsam::NonlinearFactor& factor) {
  // Convert key indices to int to avoid wraparound issues
  gtsam::KeyVector keys = factor.keys();
  if (keys.size() != 2) {
    return false;
  }
  int i = static_cast<int>(gtsam::Symbol(keys[0]).index());
  int j = static_cast<int>(gtsam::Symbol(keys[1]).index());
  return (abs(i - j) > 1);
}

gtsam::NonlinearFactorGraph generateRandomLoopClosures(
    const gtsam::KeyVector& keys, size_t num,
    const gtsam::SharedNoiseModel& inlier_model, bool is3D = true,
    size_t random_seed = 7) {
  // Set up random generator for selecting indices
  // We fix the random seed at 7 (selected arbitrarily) so that results can be
  // reproduced in a deterministic manner, but any random seed will be fine :)
  std::mt19937 rng(random_seed);

  // Set up random generator for poses (translation + rotation)
  std::uniform_real_distribution<double> t_distribution(-5.0, 5.0);
  std::uniform_real_distribution<double> u_distribution(0.0, 1.0);
  std::uniform_real_distribution<double> R_distribution(0.0, 6.28);

  gtsam::NonlinearFactorGraph graph;
  for (size_t k = 0; k < num; k++) {
    // Randomly select i
    std::uniform_int_distribution<int> uniform(0, keys.size());

    auto i = uniform(rng);

    // Randomly select j
    auto j = uniform(rng);

    // Ensure the edge (i,j) is a loop closure.
    while (abs(j - i) <= 1) {
      j = uniform(rng);
    }

    /// Generate the measurement
    // Translation part
    double dx = t_distribution(rng);
    double dy = t_distribution(rng);
    double dz = t_distribution(rng);

    // Quaternion random sampling http://planning.cs.uiuc.edu/node198.html
    double u1 = u_distribution(rng);
    double u2 = u_distribution(rng);
    double u3 = u_distribution(rng);

    double qx = sqrt(1.0 - u1) * sin(2.0 * M_PI * u2);
    double qy = sqrt(1.0 - u1) * cos(2.0 * M_PI * u2);
    double qz = sqrt(u1) * sin(2.0 * M_PI * u3);
    double qw = sqrt(u1) * cos(2.0 * M_PI * u3);

    if (is3D) {
      gtsam::Point3 t(dx, dy, dz);
      gtsam::Rot3 R(
          Eigen::Quaternion<double>(qw, qx, qy, qz).toRotationMatrix());

      gtsam::Pose3 meas(R, t);
      // gtsam::noiseModel::Diagonal::shared_ptr noise =
      //     gtsam::noiseModel::Diagonal::Sigmas(sigmas);

      gtsam::BetweenFactor<gtsam::Pose3> lc(keys[i], keys[j], meas,
                                            inlier_model);

      graph.add(lc);
    } else {
      gtsam::Point2 t(dx, dy);
      // Sample rotation from -pi to pi
      gtsam::Rot2 R(M_PI * (2.0 * u1 - 1.0));

      gtsam::Pose2 meas(R, t);

      gtsam::BetweenFactor<gtsam::Pose2> lc(keys[i], keys[j], meas,
                                            inlier_model);
      graph.add(lc);
    }
  }

  return graph;
}

double hybrid_error(const dcsam::HybridFactorGraph& hfg,
                    const dcsam::DCValues& values) {
  double dcError = 0.0;
  for (auto dcfactor : hfg.dcGraph()) {
    dcError += dcfactor->error(values.continuous, values.discrete);
  }
  double discreteError = 0.0;
  for (auto dfactor : hfg.discreteGraph()) {
    discreteError -= log(dfactor->operator()(values.discrete));
  }
  return log(hfg.nonlinearGraph().error(values.continuous)) + dcError +
         discreteError;
}

void writeDiscreteMarginals(const gtsam::KeySet& discreteKeys,
                            const gtsam::DiscreteMarginals& discreteMarg,
                            const std::string& fname) {
  std::ofstream file(fname);
  for (const gtsam::Key& key : discreteKeys) {
    // Construct a discrete key from gtsam::Key plus cardinality (2 for
    // inlier/outlier)
    gtsam::DiscreteKey dk(key, 2);
    // Recover probabilities from the marginal conditioned on continuous
    // states.
    gtsam::Vector probabilities = discreteMarg.marginalProbabilities(dk);

    // We have probabilities[0] = p(inlier | C, meas) and probabilities[1] =
    // p(outlier | . )
    double outlier_prob = probabilities[1];
    file << outlier_prob << std::endl;
  }
}
