#include <dcsam/DCMixtureFactor.h>
#include <dcsam/DCSAM.h>
#include <dcsam/DiscretePriorFactor.h>
#include <dcsam/HybridFactorGraph.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/GncOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/dataset.h>

#include <chrono>
#include <iostream>
#include <random>

#include "pgo_utils.h"

/**
 * graph - the uncorrupted factor graph
 * initial_values - the initial guess (usually odometry)
 * outlier_pct - fraction in [0,1] of outliers
 * random_seed - the random seed to use
 * is3D - true if the dataset is 3D
 * dataset_name - the name of the dataset
 */
void run_experiment3D(const gtsam::NonlinearFactorGraph& graph,
                      const gtsam::Values& initial_values, double outlier_pct,
                      size_t random_seed, std::string dataset_name) {
  /// 1. Build outlier graph.
  // Add prior on the pose having index (key) = 0
  gtsam::NonlinearFactorGraph graphWithOutliers = graph;

  gtsam::Vector6 prior_sigmas;
  prior_sigmas << 1e-8, 1e-8, 1e-8, 1e-6, 1e-6, 1e-6;
  gtsam::noiseModel::Diagonal::shared_ptr priorModel =
      gtsam::noiseModel::Diagonal::Sigmas(prior_sigmas);
  graphWithOutliers.add(
      gtsam::PriorFactor<gtsam::Pose3>(0, gtsam::Pose3(), priorModel));
  std::cout << "Adding prior on pose 0 " << std::endl;

  // Give a very coarse estimate for outlier sigmas. Derived from Olson and
  // Agarwal 2013 who recommend 10^7 for outlier variance, we conservatively
  // round up to 4000 for sigmas.
  gtsam::Vector6 outlier_sigmas;
  outlier_sigmas << 4000, 4000, 4000, 4000, 4000, 4000;
  gtsam::noiseModel::Diagonal::shared_ptr outlierModel =
      gtsam::noiseModel::Diagonal::Sigmas(outlier_sigmas);

  double outlier_prob = 0.5;
  std::vector<double> lc_probabilities{(1.0 - outlier_prob), outlier_prob};

  // We'll use this to save the inlier model so we can create the inlier
  // hypotheses.
  gtsam::SharedNoiseModel inlier_model;

  // Make a HybridFactorGraph to store the problem data.
  dcsam::HybridFactorGraph hfg;

  // Keep track of loop closure index.
  size_t k = 0;

  // Keep track of known inliers (odometry) for GNC.
  size_t known_inlier_idx = 0;
  gtsam::GncParams<gtsam::LevenbergMarquardtParams>::IndexVector knownInliers;

  // Add all good measurements.
  for (const auto& factor : graphWithOutliers) {
    // Ensure that we correctly retrieve a BetweenFactor
    boost::shared_ptr<gtsam::BetweenFactor<gtsam::Pose3>> bwFactor =
        boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3>>(factor);
    if (bwFactor && isLoopClosure(*bwFactor)) {
      // If the meaasurement is a loop closure, we add inlier and outlier
      // hypotheses.
      auto keys = bwFactor->keys();

      // We'll save the last inlier model to use for all our random
      // measuremenets.
      inlier_model = bwFactor->noiseModel();
      // std::cout << inlier_model->sigmas() << std::endl;

      gtsam::Vector6 outlier_sigmas = 4000.0 * inlier_model->sigmas();

      gtsam::noiseModel::Diagonal::shared_ptr outlierModel =
        gtsam::noiseModel::Diagonal::Sigmas(outlier_sigmas);

      // Build a vector of components factors (inlier model, outlier model)
      std::vector<gtsam::BetweenFactor<gtsam::Pose3>> components;
      components.push_back(*bwFactor);
      components.push_back(gtsam::BetweenFactor<gtsam::Pose3>(
          keys[0], keys[1], bwFactor->measured(), outlierModel));

      // Create a discrete key to index into components. Cardinality is 2
      // since a loop closure is either an inlier (0) or outlier (1).
      gtsam::DiscreteKey dk(gtsam::Symbol('d', k), 2);
      dcsam::DCMixtureFactor dcmf(bwFactor->keys(), dk, components, false);
      hfg.push_dc(dcmf);

      dcsam::DiscretePriorFactor dpf(dk, lc_probabilities);
      hfg.push_discrete(dpf);
      k++;

    } else {
      hfg.push_nonlinear(factor);
      // if (bwFactor) {
        knownInliers.push_back(known_inlier_idx);
      // }
    }
    // Skip the prior factor when counting odometry measurements
    // if (bwFactor) {
      known_inlier_idx++;
    // }
  }

  /// 2. For each method: solve graph.

  size_t num_original_lc = k;

  std::cout << "Processed " << num_original_lc << " loop closures."
            << std::endl;

  // We want num_outliers such that:
  //    outlier_pct = num_outliers / (num_outliers + num_original)
  //
  // A little algebra reveals:
  //    num_outliers = num_original_lc * (outlier_pct / (1 - outlier_pct))
  //
  // Cast is kosher since we know outlier_pct >= 0 and num_original_lc is
  // of type size_t.
  size_t num_outliers =
      (size_t)((outlier_pct / (1.0 - outlier_pct)) * num_original_lc);

  gtsam::NonlinearFactorGraph outlierGraph = generateRandomLoopClosures(
      graph.keyVector(), num_outliers, inlier_model, true, random_seed);

  // Add all outlier measurements.
  for (const auto& factor : outlierGraph) {
    // Ensure that we correctly retrieve a BetweenFactor
    boost::shared_ptr<gtsam::BetweenFactor<gtsam::Pose3>> bwFactor =
        boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3>>(factor);
    if (bwFactor && isLoopClosure(*bwFactor)) {
      // If the meaasurement is a loop closure, we add inlier and outlier
      // hypotheses.
      auto keys = bwFactor->keys();

      inlier_model = bwFactor->noiseModel();

      gtsam::Vector6 outlier_sigmas = 4000.0 * inlier_model->sigmas();

      gtsam::noiseModel::Diagonal::shared_ptr outlierModel =
        gtsam::noiseModel::Diagonal::Sigmas(outlier_sigmas);

      // Build a vector of components factors (inlier model, outlier model)
      std::vector<gtsam::BetweenFactor<gtsam::Pose3>> components;
      components.push_back(*bwFactor);
      components.push_back(gtsam::BetweenFactor<gtsam::Pose3>(
          keys[0], keys[1], bwFactor->measured(), outlierModel));

      // Create a discrete key to index into components. Cardinality is 2
      // since a loop closure is either an inlier (0) or outlier (1).
      gtsam::DiscreteKey dk(gtsam::Symbol('d', k), 2);
      dcsam::DCMixtureFactor dcmf(bwFactor->keys(), dk, components, false);
      hfg.push_dc(dcmf);

      // Set up the discrete weights for each factor.
      dcsam::DiscretePriorFactor dpf(dk, lc_probabilities);
      hfg.push_discrete(dpf);

      // Add outlier factor to graph so we can visualize later.
      graphWithOutliers.add(*bwFactor);
      k++;
    } else {
      hfg.push_nonlinear(factor);
    }
  }
  std::cout << "HFG size: " << hfg.size() << std::endl;

  // Currently DCSAM requires that we provide initial discrete values, but
  // they aren't actually used since we perform the discrete solve first, so
  // we just set them all to 0.
  dcsam::DiscreteValues initial_discrete;
  for (size_t i = 0; i < k; i++) {
    initial_discrete[gtsam::Symbol('d', i)] = 0;
  }

  // Create a solver instance and use it to optimize the graph.
  dcsam::DCSAM dcsam;

  using std::chrono::duration;
  using std::chrono::duration_cast;
  using std::chrono::milliseconds;

  std::vector<double> dcsam_compute_times, dcsam_costs, dcsam_hybrid_costs;
  auto t1 = std::chrono::high_resolution_clock::now();
  dcsam.update(hfg, initial_values, initial_discrete);
  auto t2 = std::chrono::high_resolution_clock::now();
  // Get milliseconds as int
  auto ms_int = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
  // Convert to double
  double ms_double = static_cast<double>(ms_int.count());
  // Get compute time in sec
  dcsam_compute_times.push_back(ms_double / 1000.0);

  // Perform a few more iterations to converge.
  dcsam::DCValues result;
  std::vector<dcsam::DCValues> dc_results;
  result = dcsam.calculateEstimate();
  dc_results.push_back(result);
  dcsam_costs.push_back(graph.error(result.continuous));
  dcsam_hybrid_costs.push_back(hybrid_error(hfg, result));
  gtsam::writeG2o(graphWithOutliers, result.continuous, "out_robust0.g2o");
  for (size_t iter = 0; iter < 49; iter++) {
    t1 = std::chrono::high_resolution_clock::now();
    dcsam.update();
    t2 = std::chrono::high_resolution_clock::now();
    ms_int = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
    ms_double = static_cast<double>(ms_int.count());
    dcsam_compute_times.push_back(ms_double / 1000.0);

    result = dcsam.calculateEstimate();
    dc_results.push_back(result);
    dcsam_costs.push_back(graph.error(result.continuous));
    dcsam_hybrid_costs.push_back(hybrid_error(hfg, result));

    std::string fname = "out_robust" + std::to_string(iter + 1) + ".g2o";
    gtsam::writeG2o(graphWithOutliers, result.continuous, fname);
    // dcsam::DCMarginals marg =
    //     dcsam.getMarginals(dcsam.getNonlinearFactorGraph(),
    //     result.continuous,
    //                        dcsam.getDiscreteFactorGraph());
    // std::string marg_fname = "marg_robust" + std::to_string(iter + 1) +
    // ".txt"; writeDiscreteMarginals(dcsam.getDiscreteFactorGraph().keys(),
    // marg.discrete,
    //                        marg_fname);

    if (dc_results[iter + 1].continuous.equals(dc_results[iter].continuous) &&
        dc_results[iter + 1].discrete.equals(dc_results[iter].discrete)) {
      // We have reached convergence
      std::cout << "We have reached convergence!" << std::endl;
      break;
    }
  }

  // Perform a few more iterations
  // for (size_t j = 0; j < 10; j++) {
  //   t1 = std::chrono::high_resolution_clock::now();
  //   dcsam.update();
  //   t2 = std::chrono::high_resolution_clock::now();
  //   ms_int = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
  //   ms_double = static_cast<double>(ms_int.count());
  //   dcsam_compute_times.push_back(ms_double / 1000.0);
  //   result = dcsam.calculateEstimate();
  //   dcsam_costs.push_back(graph.error(result.continuous));
  //   dcsam_hybrid_costs.push_back(hybrid_error(hfg, result));
  // }
  // auto t2 = std::chrono::high_resolution_clock::now();

  // Get milliseconds as int
  // auto ms_int = std::chrono::duration_cast<std::chrono::milliseconds>(t2 -
  // t1);

  // Convert to double
  // double ms_double = static_cast<double>(ms_int.count());

  // Total time in seconds:
  // double total_time = ms_double / 1000.0;

  // std::cout << "Total time [DCSAM]: " << total_time << " seconds." <<
  // std::endl; std::cout << "Avg. time/iter [DCSAM]: " << total_time / 7.0 << "
  // seconds."
  //           << std::endl;
  assert(dcsam_costs.size() == dcsam_compute_times.size());
  assert(dcsam_costs.size() == dcsam_hybrid_costs.size());
  double total_time_dcsam = 0.0;
  for (size_t idx = 0; idx < dcsam_costs.size(); idx++) {
    total_time_dcsam += dcsam_compute_times[idx];
    std::cout << "idx, time, cost, dcCost" << std::endl;
    std::cout << idx << ", " << dcsam_compute_times[idx] << ", "
              << dcsam_costs[idx] << ", " << dcsam_hybrid_costs[idx]
              << std::endl;
    std::cout << "total time: " << total_time_dcsam << std::endl;
  }

  double total_time_lm, total_time_gnc;

  result = dcsam.calculateEstimate();
  std::cout << "Initial cost: " << graph.error(initial_values) << std::endl;
  std::cout << "Final cost [DCSAM]: " << graph.error(result.continuous)
            << std::endl;

  // Make sure we add the prior factor to the outlier graph before passing to LM
  // and GNC. This addresses gauge ambiguity.
  gtsam::NonlinearFactorGraph graphWithOutliersAndPrior =
      graphWithOutliers.clone();

  // graphWithOutliersAndPrior.add(
  //     gtsam::PriorFactor<gtsam::Pose3>(0, gtsam::Pose3(), priorModel));

  // Solve the non-robust version of the graph
  gtsam::LevenbergMarquardtOptimizer lm(graphWithOutliersAndPrior,
                                        initial_values);

  t1 = std::chrono::high_resolution_clock::now();
  gtsam::Values lm_result = lm.optimize();
  t2 = std::chrono::high_resolution_clock::now();

  std::cout << "Final cost [LM]: " << graph.error(lm_result) << std::endl;

  // Get milliseconds as int
  ms_int = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);

  // Convert to double
  ms_double = static_cast<double>(ms_int.count());
  total_time_lm = ms_double / 1000.0;

  std::cout << "Total time [LM]: " << total_time_lm << " seconds." << std::endl;

  /// Solve with GNC
  gtsam::GncParams<gtsam::LevenbergMarquardtParams> gncParams;

  // Let GNC know that odometry measurements are inliers:
  std::cout << "Known inliers size: " << knownInliers.size() << std::endl;
  gncParams.knownInliers = knownInliers;
  gncParams.lossType = gtsam::GncLossType::TLS;
  // Collect summary stats from GNC
  // gncParams.verbosity =
  //     gtsam::GncParams<gtsam::LevenbergMarquardtParams>::Verbosity::SUMMARY;
  gncParams.maxIterations = 50;

  // Make the optimizer and solve.
  auto gnc =
      gtsam::GncOptimizer<gtsam::GncParams<gtsam::LevenbergMarquardtParams>>(
          graphWithOutliersAndPrior, initial_values, gncParams);

  std::cout << "Default inlier cost threshold = "
            << gnc.getInlierCostThresholds()[0] << std::endl;

  // We found that setting the inlier cost threshold to 0.5 gave us the best
  // results on the `parking-garage.g2o` dataset, and setting the to 4 gave the
  // best results on `sphere2500.g2o`
  // gnc.setInlierCostThresholds(4.0);
  gnc.setInlierCostThresholds(0.5);

  t1 = std::chrono::high_resolution_clock::now();
  gtsam::Values gnc_result = gnc.optimize();
  t2 = std::chrono::high_resolution_clock::now();

  std::cout << "Final cost [GNC]: " << graph.error(gnc_result) << std::endl;

  // Get milliseconds as int
  ms_int = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);

  // Convert to double
  ms_double = static_cast<double>(ms_int.count());
  total_time_gnc = ms_double / 1000.0;

  std::cout << "Total time [GNC]: " << total_time_gnc << " seconds."
            << std::endl;

  std::cout << "OUTLIER PCT" << std::to_string((int)(100 * outlier_pct))
            << std::endl;
  std::string path_prefix = "../../output/robust_pgo_vanilla/" + dataset_name +
                            "/" + std::to_string((int)(100 * outlier_pct)) +
                            "/" + std::to_string(random_seed) + "/";

  std::cout << "path_prefix: " << path_prefix << std::endl;

  // use "writeG2o" to save file.
  gtsam::writeG2o(graphWithOutliers, lm_result,
                  path_prefix + "out_nonrobust.g2o");
  gtsam::writeG2o(graphWithOutliers, result.continuous,
                  path_prefix + "out_robust.g2o");
  gtsam::writeG2o(graphWithOutliers, gnc_result, path_prefix + "out_gnc.g2o");

  // All loop closures _after_ this number should be visualized as outliers.
  std::ofstream file(path_prefix + "orig_graph_size_out.txt");
  file << graph.size();

  // Write timing info
  std::ofstream time_file(path_prefix + "times.txt");
  time_file << "DCSAM: " << std::to_string(total_time_dcsam) << "\n"
            << "GNC: " << std::to_string(total_time_gnc) << "\n"
            << "LM: " << total_time_lm << "\n";

  /// 3. Collect stats, final trajectory, and output to file.
  return;
}

/**
 * graph - the uncorrupted factor graph
 * initial_values - the initial guess (usually odometry)
 * outlier_pct - fraction in [0,1] of outliers
 * random_seed - the random seed to use
 * dataset_name - the name of the dataset
 */
void run_experiment2D(const gtsam::NonlinearFactorGraph& graph,
                      const gtsam::Values& initial_values, double outlier_pct,
                      size_t random_seed, std::string dataset_name) {
  /// 1. Build outlier graph.
  // Add prior on the pose having index (key) = 0
  gtsam::NonlinearFactorGraph graphWithOutliers = graph;

  gtsam::Vector3 prior_sigmas;
  prior_sigmas << 1e-6, 1e-6, 1e-8;
  gtsam::noiseModel::Diagonal::shared_ptr priorModel =
      gtsam::noiseModel::Diagonal::Sigmas(prior_sigmas);
  graphWithOutliers.add(
      gtsam::PriorFactor<gtsam::Pose2>(0, gtsam::Pose2(), priorModel));
  std::cout << "Adding prior on pose 0 " << std::endl;

  // Give a very coarse estimate for outlier sigmas. Derived from Olson and
  // Agarwal 2013 who recommend 10^7 for outlier variance, we conservatively
  // round up to 4000 for sigmas.
  gtsam::Vector3 outlier_sigmas;
  outlier_sigmas << 4000, 4000, 4000;
  gtsam::noiseModel::Diagonal::shared_ptr outlierModel =
      gtsam::noiseModel::Diagonal::Sigmas(outlier_sigmas);

  double outlier_prob = 0.5;
  std::vector<double> lc_probabilities{(1.0 - outlier_prob), outlier_prob};

  // double outlier_prob = 0.5;
  // std::vector<double> lc_probabilities{(1.0 - outlier_prob), outlier_prob};

  // We'll use this to save the inlier model so we can create the inlier
  // hypotheses.
  gtsam::SharedNoiseModel inlier_model;

  // Make a HybridFactorGraph to store the problem data.
  dcsam::HybridFactorGraph hfg;

  // Keep track of loop closure index.
  size_t k = 0;

  // Keep track of known inliers (odometry) for GNC.
  size_t known_inlier_idx = 0;
  gtsam::GncParams<gtsam::LevenbergMarquardtParams>::IndexVector knownInliers;

  // Add all good measurements.
  for (const auto& factor : graphWithOutliers) {
    // Ensure that we correctly retrieve a BetweenFactor
    boost::shared_ptr<gtsam::BetweenFactor<gtsam::Pose2>> bwFactor =
        boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose2>>(factor);
    if (bwFactor && isLoopClosure(*bwFactor)) {
      // If the meaasurement is a loop closure, we add inlier and outlier
      // hypotheses.
      auto keys = bwFactor->keys();

      // We'll save the last inlier model to use for all our random
      // measuremenets.
      inlier_model = bwFactor->noiseModel();
      // std::cout << inlier_model->sigmas() << std::endl;

      // Build a vector of components factors (inlier model, outlier model)
      std::vector<gtsam::BetweenFactor<gtsam::Pose2>> components;
      components.push_back(*bwFactor);

      gtsam::Vector3 outlier_sigmas = 4000.0 * inlier_model->sigmas();

      gtsam::noiseModel::Diagonal::shared_ptr outlierModel =
          gtsam::noiseModel::Diagonal::Sigmas(outlier_sigmas);

      components.push_back(gtsam::BetweenFactor<gtsam::Pose2>(
          keys[0], keys[1], bwFactor->measured(), outlierModel));

      // Create a discrete key to index into components. Cardinality is 2
      // since a loop closure is either an inlier (0) or outlier (1).
      gtsam::DiscreteKey dk(gtsam::Symbol('d', k), 2);
      dcsam::DCMixtureFactor dcmf(bwFactor->keys(), dk, components, false);
      hfg.push_dc(dcmf);

      dcsam::DiscretePriorFactor dpf(dk, lc_probabilities);
      hfg.push_discrete(dpf);
      k++;

    } else {
      hfg.push_nonlinear(factor);
      // if (bwFactor) {
      knownInliers.push_back(known_inlier_idx);
      // }
    }
    // Skip the prior factor when counting odometry measurements
    // if (bwFactor) {
    known_inlier_idx++;
    // }
  }

  /// 2. For each method: solve graph.

  size_t num_original_lc = k;

  std::cout << "Processed " << num_original_lc << " loop closures."
            << std::endl;

  // We want num_outliers such that:
  //    outlier_pct = num_outliers / (num_outliers + num_original)
  //
  // A little algebra reveals:
  //    num_outliers = num_original_lc * (outlier_pct / (1 - outlier_pct))
  //
  // Cast is kosher since we know outlier_pct >= 0 and num_original_lc is
  // of type size_t.
  size_t num_outliers =
      (size_t)((outlier_pct / (1.0 - outlier_pct)) * num_original_lc);

  gtsam::NonlinearFactorGraph outlierGraph = generateRandomLoopClosures(
      graph.keyVector(), num_outliers, inlier_model, false, random_seed);

  // Add all outlier measurements.
  for (const auto& factor : outlierGraph) {
    // Ensure that we correctly retrieve a BetweenFactor
    boost::shared_ptr<gtsam::BetweenFactor<gtsam::Pose2>> bwFactor =
        boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose2>>(factor);
    if (bwFactor && isLoopClosure(*bwFactor)) {
      // If the meaasurement is a loop closure, we add inlier and outlier
      // hypotheses.
      auto keys = bwFactor->keys();

      inlier_model = bwFactor->noiseModel();

      // Build a vector of components factors (inlier model, outlier model)
      std::vector<gtsam::BetweenFactor<gtsam::Pose2>> components;
      components.push_back(*bwFactor);

      gtsam::Vector3 outlier_sigmas = 4000.0 * inlier_model->sigmas();

      gtsam::noiseModel::Diagonal::shared_ptr outlierModel =
          gtsam::noiseModel::Diagonal::Sigmas(outlier_sigmas);
      components.push_back(gtsam::BetweenFactor<gtsam::Pose2>(
          keys[0], keys[1], bwFactor->measured(), outlierModel));

      // Create a discrete key to index into components. Cardinality is 2
      // since a loop closure is either an inlier (0) or outlier (1).
      gtsam::DiscreteKey dk(gtsam::Symbol('d', k), 2);
      dcsam::DCMixtureFactor dcmf(bwFactor->keys(), dk, components, false);
      hfg.push_dc(dcmf);

      // Set up the discrete weights for each factor.
      dcsam::DiscretePriorFactor dpf(dk, lc_probabilities);
      hfg.push_discrete(dpf);

      // Add outlier factor to graph so we can visualize later.
      graphWithOutliers.add(*bwFactor);
      k++;
    } else {
      hfg.push_nonlinear(factor);
    }
  }
  std::cout << "HFG size: " << hfg.size() << std::endl;

  // Currently DCSAM requires that we provide initial discrete values, but
  // they aren't actually used since we perform the discrete solve first, so
  // we just set them all to 0.
  dcsam::DiscreteValues initial_discrete;
  for (size_t i = 0; i < k; i++) {
    initial_discrete[gtsam::Symbol('d', i)] = 0;
  }

  // Create a solver instance and use it to optimize the graph.
  dcsam::DCSAM dcsam;

  using std::chrono::duration;
  using std::chrono::duration_cast;
  using std::chrono::milliseconds;

  std::vector<double> dcsam_compute_times, dcsam_costs, dcsam_hybrid_costs;
  auto t1 = std::chrono::high_resolution_clock::now();
  dcsam.update(hfg, initial_values, initial_discrete);
  auto t2 = std::chrono::high_resolution_clock::now();
  // Get milliseconds as int
  auto ms_int = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
  // Convert to double
  double ms_double = static_cast<double>(ms_int.count());
  // Get compute time in sec
  dcsam_compute_times.push_back(ms_double / 1000.0);

  // Perform a few more iterations to converge.
  dcsam::DCValues result;
  std::vector<dcsam::DCValues> dc_results;
  result = dcsam.calculateEstimate();
  dc_results.push_back(result);
  dcsam_costs.push_back(graph.error(result.continuous));
  dcsam_hybrid_costs.push_back(hybrid_error(hfg, result));
  gtsam::writeG2o(graphWithOutliers, result.continuous, "out_robust0.g2o");
  for (size_t iter = 0; iter < 49; iter++) {
    t1 = std::chrono::high_resolution_clock::now();
    dcsam.update();
    t2 = std::chrono::high_resolution_clock::now();
    ms_int = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
    ms_double = static_cast<double>(ms_int.count());
    dcsam_compute_times.push_back(ms_double / 1000.0);

    result = dcsam.calculateEstimate();
    dc_results.push_back(result);
    dcsam_costs.push_back(graph.error(result.continuous));
    dcsam_hybrid_costs.push_back(hybrid_error(hfg, result));

    std::string fname = "out_robust" + std::to_string(iter + 1) + ".g2o";
    gtsam::writeG2o(graphWithOutliers, result.continuous, fname);

    if (dc_results[iter + 1].continuous.equals(dc_results[iter].continuous) &&
        dc_results[iter + 1].discrete.equals(dc_results[iter].discrete)) {
      // We have reached convergence
      std::cout << "We have reached convergence!" << std::endl;
      break;
    }
    // dcsam::DCMarginals marg =
    //     dcsam.getMarginals(dcsam.getNonlinearFactorGraph(),
    //     result.continuous,
    //                        dcsam.getDiscreteFactorGraph());
    // std::string marg_fname = "marg_robust" + std::to_string(iter + 1) +
    // ".txt"; writeDiscreteMarginals(dcsam.getDiscreteFactorGraph().keys(),
    // marg.discrete,
    //                        marg_fname);
  }

  // Perform a few more iterations
  // for (size_t j = 0; j < 100; j++) {
  //   t1 = std::chrono::high_resolution_clock::now();
  //   dcsam.update();
  //   t2 = std::chrono::high_resolution_clock::now();
  //   ms_int = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
  //   ms_double = static_cast<double>(ms_int.count());
  //   dcsam_compute_times.push_back(ms_double / 1000.0);
  //   result = dcsam.calculateEstimate();
  //   dc_results.push_back(result);
  //   dcsam_costs.push_back(graph.error(result.continuous));
  //   dcsam_hybrid_costs.push_back(hybrid_error(hfg, result));
  // }
  // auto t2 = std::chrono::high_resolution_clock::now();

  // Get milliseconds as int
  // auto ms_int = std::chrono::duration_cast<std::chrono::milliseconds>(t2 -
  // t1);

  // Convert to double
  // double ms_double = static_cast<double>(ms_int.count());

  // Total time in seconds:
  // double total_time = ms_double / 1000.0;

  // std::cout << "Total time [DCSAM]: " << total_time << " seconds." <<
  // std::endl; std::cout << "Avg. time/iter [DCSAM]: " << total_time / 7.0 << "
  // seconds."
  //           << std::endl;
  assert(dcsam_costs.size() == dcsam_compute_times.size());
  assert(dcsam_costs.size() == dcsam_hybrid_costs.size());
  double total_time_dcsam = 0.0;
  for (size_t idx = 0; idx < dcsam_costs.size(); idx++) {
    total_time_dcsam += dcsam_compute_times[idx];
    std::cout << "idx, time, cost, dcCost" << std::endl;
    std::cout << idx << ", " << dcsam_compute_times[idx] << ", "
              << dcsam_costs[idx] << ", " << dcsam_hybrid_costs[idx]
              << std::endl;
    std::cout << "total time: " << total_time_dcsam << std::endl;
    if (idx > 0) {
      std::cout << "stationary?: "
                << (dc_results[idx].continuous.equals(
                        dc_results[idx - 1].continuous) &&
                    dc_results[idx].discrete.equals(
                        dc_results[idx - 1].discrete))
                << std::endl;
    }
  }

  double total_time_lm, total_time_gnc;

  result = dcsam.calculateEstimate();
  std::cout << "Initial cost: " << graph.error(initial_values) << std::endl;
  std::cout << "Final cost [DCSAM]: " << graph.error(result.continuous)
            << std::endl;

  // Make sure we add the prior factor to the outlier graph before passing to LM
  // and GNC. This addresses gauge ambiguity.
  gtsam::NonlinearFactorGraph graphWithOutliersAndPrior =
      graphWithOutliers.clone();

  // graphWithOutliersAndPrior.add(
  //     gtsam::PriorFactor<gtsam::Pose2>(0, gtsam::Pose2(), priorModel));

  // Solve the non-robust version of the graph
  gtsam::LevenbergMarquardtOptimizer lm(graphWithOutliersAndPrior,
                                        initial_values);

  t1 = std::chrono::high_resolution_clock::now();
  gtsam::Values lm_result = lm.optimize();
  t2 = std::chrono::high_resolution_clock::now();

  std::cout << "Final cost [LM]: " << graph.error(lm_result) << std::endl;

  // Get milliseconds as int
  ms_int = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);

  // Convert to double
  ms_double = static_cast<double>(ms_int.count());
  total_time_lm = ms_double / 1000.0;

  std::cout << "Total time [LM]: " << total_time_lm << " seconds." << std::endl;

  /// Solve with GNC
  gtsam::GncParams<gtsam::LevenbergMarquardtParams> gncParams;

  // Let GNC know that odometry measurements are inliers:
  std::cout << "Known inliers size: " << knownInliers.size() << std::endl;
  gncParams.knownInliers = knownInliers;
  gncParams.lossType = gtsam::GncLossType::TLS;
  // Collect summary stats from GNC
  // gncParams.verbosity =
  //     gtsam::GncParams<gtsam::LevenbergMarquardtParams>::Verbosity::SUMMARY;
  gncParams.maxIterations = 50;

  // Make the optimizer and solve.
  auto gnc =
      gtsam::GncOptimizer<gtsam::GncParams<gtsam::LevenbergMarquardtParams>>(
          graphWithOutliersAndPrior, initial_values, gncParams);

  std::cout << "Default inlier cost threshold = "
            << gnc.getInlierCostThresholds()[0] << std::endl;

  // We found that setting the inlier cost threshold to 0.5 gave us the best
  // results on the `parking-garage.g2o` dataset, and setting the to 4 gave the
  // best results on `sphere2500.g2o`
  // gnc.setInlierCostThresholds(4.0);

  t1 = std::chrono::high_resolution_clock::now();
  gtsam::Values gnc_result = gnc.optimize();
  t2 = std::chrono::high_resolution_clock::now();

  std::cout << "Final cost [GNC]: " << graph.error(gnc_result) << std::endl;

  // Get milliseconds as int
  ms_int = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);

  // Convert to double
  ms_double = static_cast<double>(ms_int.count());
  total_time_gnc = ms_double / 1000.0;

  std::cout << "Total time [GNC]: " << total_time_gnc << " seconds."
            << std::endl;

  std::cout << "OUTLIER PCT" << std::to_string((int)(100 * outlier_pct))
            << std::endl;
  std::string path_prefix = "../../output/robust_pgo_vanilla/" + dataset_name +
                            "/" + std::to_string((int)(100 * outlier_pct)) +
                            "/" + std::to_string(random_seed) + "/";

  std::cout << "path_prefix: " << path_prefix << std::endl;

  // use "writeG2o" to save file.
  gtsam::writeG2o(graphWithOutliers, lm_result,
                  path_prefix + "out_nonrobust.g2o");
  gtsam::writeG2o(graphWithOutliers, result.continuous,
                  path_prefix + "out_robust.g2o");
  gtsam::writeG2o(graphWithOutliers, gnc_result, path_prefix + "out_gnc.g2o");

  // All loop closures _after_ this number should be visualized as outliers.
  std::ofstream file(path_prefix + "orig_graph_size_out.txt");
  file << graph.size();

  // Write timing info
  std::ofstream time_file(path_prefix + "times.txt");
  time_file << "DCSAM: " << std::to_string(total_time_dcsam) << "\n"
            << "GNC: " << std::to_string(total_time_gnc) << "\n"
            << "LM: " << total_time_lm << "\n";

  /// 3. Collect stats, final trajectory, and output to file.
  return;
}

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cout << "Usage: " << argv[0] << " [.g2o file]"
              << " [outlier rate (int >= 0; default 0)]"
              << " [is3D (0/1; default 1)]"
              << " [num trials (default 1)]" << std::endl;
    exit(1);
  }

  // Defaults
  double outlier_pct = 0.0;
  bool is3D = true;
  size_t num_trials = 1;

  // Begin parsing input
  std::string path = argv[1];

  std::string base_filename = path.substr(path.find_last_of("/\\") + 1);
  std::string::size_type const p(base_filename.find_last_of('.'));
  std::string file_without_extension = base_filename.substr(0, p);

  if (argc >= 3) {
    std::cout << "outlier percent " << atoi(argv[2]) << std::endl;
    outlier_pct = (double)(atoi(argv[2])) / 100.0;
    std::cout << "outlier percent " << outlier_pct << std::endl;
  }
  if ((argc >= 4) && (atoi(argv[3]) != 1)) {
    is3D = false;
  }
  if ((argc >= 5)) {
    assert(atoi(argv[4]) > 0);
    num_trials = atoi(argv[4]);
  }

  std::cout << "Running robust PGO experiment\n"
            << "=============================\n"
            << "dataset:\t" << file_without_extension << "\n"
            << "outlier_pct:\t" << outlier_pct << "\n"
            << "is3D?:\t\t" << is3D << "\n"
            << "num_trials:\t" << num_trials << "\n"
            << std::endl;

  gtsam::NonlinearFactorGraph::shared_ptr graph;
  gtsam::Values::shared_ptr initial;

  boost::tie(graph, initial) = gtsam::readG2o(path, is3D);

  std::cout << "Loaded a graph of size: " << graph->size() << std::endl;

  for (size_t i = 1; i < num_trials + 1; i++) {
    if (is3D) {
      run_experiment3D(*graph, *initial, outlier_pct, i,
                       file_without_extension);
    } else {
      run_experiment2D(*graph, *initial, outlier_pct, i,
                       file_without_extension);
    }
  }
}
