# dcsam-examples

This repository contains example code for the experiments provided in
our paper "[Discrete-Continuous Smoothing and Mapping](https://arxiv.org/abs/2204.11936)."

**NOTE: As of 1/30/2023 the latest version of DC-SAM (and dcsam-examples) on `main` depends on GTSAM release 4.2a8.** If you are using GTSAM 4.1.1, check out our [pre-4.2 release tag](https://github.com/MarineRoboticsGroup/dcsam-examples/releases/tag/pre-4.2) and the [corresponding pre-4.2 version of DC-SAM](https://github.com/MarineRoboticsGroup/dcsam/releases/tag/pre-4.2). This is the version of DC-SAM (and dcsam-examples) you would have used if you cloned the repository prior to 1/30/2023. Many thanks to [Parker Lusk](https://github.com/plusk01) for bringing us into the future.

The file `icp.cpp` implements the iterative closest point (ICP) method for point
cloud registration.

The file `robust_pgo_mc.cpp` implements robust pose-graph optimization by
introducing inlier/outlier discrete hypotheses for untrusted loop closure edges.

To build the ICP and robust PGO examples:

```bash
~ $ mkdir build
~ $ cd build
~/build $ cmake ..
~/build $ make -j
```

To run the ICP example:
```bash
~/build/examples $ ./icp (path to source cloud) (path to target cloud)
```

To run the robust pose-graph optimization example:
```bash
~/build/examples $ ./robust_pgo_mc [.g2o file] [outlier rate (int >= 0; default 0)] [is3D (0/1; default 1)] [num trials (default 1)]
```
