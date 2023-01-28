# dcsam

[![Build Status](http://mrg-beast.csail.mit.edu:8080/buildStatus/icon?job=dcsam%2Fmain)](http://mrg-beast.csail.mit.edu:8080/job/dcsam/job/main/) [![docs](https://img.shields.io/badge/docs-latest-blue.svg)](https://marinerobotics.mit.edu/dcsam)

This library, built using GTSAM, provides factor type definitions and a new solver to perform approximate inference on discrete-continuous (hybrid) factor graph models typically encountered in robotics applications.

### References

A technical report describing this library and our solver can be found [here](https://arxiv.org/abs/2204.11936). If you found this code useful, please cite it as:
```bibtex
@article{doherty2022discrete,
  author={Doherty, Kevin J. and Lu, Ziqi and Singh, Kurran and Leonard, John J.},
  journal={IEEE Robotics and Automation Letters},
  title={Discrete-{C}ontinuous {S}moothing and {M}apping},
  year={2022},
  volume={7},
  number={4},
  pages={12395-12402},
  doi={10.1109/LRA.2022.3216938}
}
```

## Prerequisites

- [GTSAM](https://github.com/borglab/gtsam) @ `4.2a8`

To retrieve the appropriate version of GTSAM:
```sh
~/$ git clone https://github.com/borglab/gtsam
~/$ cd gtsam
~/gtsam/$ git checkout 4.2a8
```
Follow instructions in the GTSAM repository to build and install with your desired configuration.


### Optional

- [gtest](https://github.com/google/googletest) for building tests.

## Building

### Building the project

To build using `cmake`:

```bash
~/dcsam/$ mkdir build
~/dcsam/$ cd build
~/dcsam/build$ cmake ..
~/dcsam/build$ make -j
```

### Run tests

To run unit tests, first build with testing enabled:
```bash
~/$ mkdir build
~/$ cd build
~/build$ cmake .. -DENABLE_TESTS=ON
~/build$ make -j
```

Now you can run the tests as follows:

```bash
~/build$ make test
```

### Examples

For example usage, check out [the DC-SAM examples repo](https://github.com/MarineRoboticsGroup/dcsam-examples) or take a look through `testDCSAM.cpp`.

## Developing

We're using [pre-commit](https://pre-commit.com/) for automatic linting. To install `pre-commit` run:
```
pip3 install pre-commit
```
You can verify your installation went through by running `pre-commit --version` and you should see something like `pre-commit 2.7.1`.

To get started using `pre-commit` with this codebase, from the project repo run:
```
pre-commit install
```
Now, each time you `git add` new files and try to `git commit` your code will automatically be run through a variety of linters. You won't be able to commit anything until the linters are happy with your code.
