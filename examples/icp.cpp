#include <dcsam/DCMixtureFactor.h>
#include <dcsam/DCSAM.h>
#include <dcsam/HybridFactorGraph.h>

#include <fstream>
#include <iostream>
#include <sstream>

#include "PosePointFactor.h"

std::vector<gtsam::Point3> read_xyz_file(const std::string& path_to_file) {
  // Vector of points to output.
  std::vector<gtsam::Point3> points;

  // A string containing the elements of each line.
  std::string line;

  // Preallocate x, y, z coordinates for each point.
  double x, y, z;

  // Open file for reading.
  std::ifstream infile(path_to_file);

  size_t i = 0;
  while (std::getline(infile, line)) {
    // Construct a stream from line.
    std::stringstream strstrm(line);

    // Extract formatted point.
    strstrm >> x >> y >> z;

    // Convert x, y, z to gtsam::Point3.
    gtsam::Point3 point(x, y, z);

    points.push_back(point);
  }

  return points;
}

int main(int argc, char** argv) {
  if (argc != 3) {
    std::cout << "Usage: " << argv[0] << " [source .xyz file]"
              << " [target .xyz file]" << std::endl;
    exit(1);
  }

  // Read source and target point clouds
  std::vector<gtsam::Point3> source = read_xyz_file(argv[1]);
  std::vector<gtsam::Point3> target = read_xyz_file(argv[2]);

  std::cout << "Loaded " << source.size()
            << " points from source cloud in file " << argv[1] << " and \n"
            << target.size() << " points from target cloud in file " << argv[2]
            << std::endl;

  if (source.size() == 0 || target.size() == 0) {
    std::cout
        << "Error: no measurements found in one or more input pointclouds."
        << std::endl;
    exit(1);
  }

  std::cout << source[0].x() << ", " << source[0].y() << ", " << source[0].z()
            << std::endl;

  gtsam::KeyVector poseKey{gtsam::Symbol('x', 1)};

  // Create factor graph
  dcsam::HybridFactorGraph hfg;
  for (size_t i = 0; i < source.size(); i++) {
    std::vector<PosePointFactor> components;
    for (size_t j = 0; j < target.size(); j++) {
      components.push_back(PosePointFactor(poseKey[0], source[i], target[j]));
    }
    gtsam::DiscreteKey dk(gtsam::Symbol('d', i), target.size());
    dcsam::DCMixtureFactor<PosePointFactor> dcm(poseKey, dk, components, true);
    hfg.push_dc(dcm);
  }

  std::cout << "Built HybridFactorGraph with " << hfg.size() << " factors."
            << std::endl;

  // Create DCSAM instance.
  dcsam::DCSAM dcsam;
  gtsam::Values initial_values;
  // Currently DCSAM requires that we provide initial discrete values, but they
  // aren't actually used since we perform the discrete solve first, so we
  // just set them all to 0.
  dcsam::DiscreteValues initial_discrete;
  for (size_t i = 0; i < source.size(); i++) {
    initial_discrete[gtsam::Symbol('d', i)] = 0;
  }
  initial_values.insert(poseKey[0], gtsam::Pose3::identity());

  // Compute the first iterate.
  dcsam.update(hfg, initial_values, initial_discrete);
  dcsam::DCValues estimate = dcsam.calculateEstimate();
  std::cout << "pose out: " << std::endl;
  estimate.continuous.at<gtsam::Pose3>(poseKey[0]).print();
  std::ofstream file1("out1.txt");
  file1 << estimate.continuous.at<gtsam::Pose3>(poseKey[0]).matrix();

  // Perform a few more iterations, writing to file each time:
  dcsam.update();
  estimate = dcsam.calculateEstimate();
  std::cout << "pose out 2: " << std::endl;
  estimate.continuous.at<gtsam::Pose3>(poseKey[0]).print();
  std::ofstream file2("out2.txt");
  file2 << estimate.continuous.at<gtsam::Pose3>(poseKey[0]).matrix();

  dcsam.update();
  estimate = dcsam.calculateEstimate();
  std::cout << "pose out 3: " << std::endl;
  estimate.continuous.at<gtsam::Pose3>(poseKey[0]).print();
  std::ofstream file3("out3.txt");
  file3 << estimate.continuous.at<gtsam::Pose3>(poseKey[0]).matrix();

  dcsam.update();
  estimate = dcsam.calculateEstimate();
  std::cout << "pose out 4: " << std::endl;
  estimate.continuous.at<gtsam::Pose3>(poseKey[0]).print();
  std::ofstream file4("out4.txt");
  file4 << estimate.continuous.at<gtsam::Pose3>(poseKey[0]).matrix();

  dcsam.update();
  estimate = dcsam.calculateEstimate();
  std::cout << "pose out 5: " << std::endl;
  estimate.continuous.at<gtsam::Pose3>(poseKey[0]).print();
  std::ofstream file5("out5.txt");
  file5 << estimate.continuous.at<gtsam::Pose3>(poseKey[0]).matrix();
}
