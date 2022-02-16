# dcsam

[![Build Status](http://mrg-beast.csail.mit.edu:8080/buildStatus/icon?job=dcsam%2Fmain)](http://mrg-beast.csail.mit.edu:8080/job/dcsam/job/main/)

This library, built using GTSAM, provides factor type definitions and a new solver to perform approximate inference on discrete-continuous (hybrid) factor graph models typically encountered in robotics applications.

**NOTE: This library is currently under active development by the Marine Robotics Group at MIT. As such, the API is subject to potential breaking changes at any point.**

### References

We will soon be making available a technical report describing this library and our solver and providing documentation. For now, if you found this code useful, let us know!

## Prerequisites

- [GTSAM 4.1](https://github.com/borglab/gtsam)

### Optional

- [gtest](https://github.com/google/googletest) for building tests.

## Building

### Building the project

To build using `cmake`:

```bash
~/$ mkdir build
~/$ cd build
~/build$ cmake ..
~/build$ make -j
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

For example usage, for now the best resource is to take a look through `testDCSAM.cpp`.

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

For modules in the `thirdparty` directory, we will not do any linting. If you are adding code in a `thirdparty` directory, you should commit with:
```
git commit --no-verify -m "My commit message here...."
```
