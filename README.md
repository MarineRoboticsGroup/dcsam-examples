# dcsam-examples

This repository contains example code for the experiments provided in
our paper "[Discrete-Continuous Smoothing and Mapping](https://arxiv.org/abs/2204.11936)."

The file `icp.cpp` implements the iterative closest point (ICP) method for point
cloud registration.

The file `robust_pgo.cpp` implements robust pose-graph optimization by
introducing inlier/outlier discrete hypotheses for untrusted loop closure edges.

To build the ICP and robust PGO examples:

```bash
~/$ mkdir build
~/$ cd build
~/build$ cmake ..
~/build$ make -j
```

To run the ICP example:
```bash
~/build/examples$ ./icp (path to source cloud) (path to target cloud)
```

To run the robust pose-graph optimization example:
```bash
~/build/examples$ ./robust_pgo (path to .g2o file)
```
