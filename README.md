# dcsam

Factored inference for discrete-continuous smoothing and mapping problems.

### References

This method is fully described in the following report:
```
@article{doherty2021dcsam,
title={DCSAM},
author={Doherty, Kevin J. and Lu, Ziqi and Leonard, John J.},
journal={Technical report},
year={2021}
}
```

## Prerequisites

- [GTSAM 4.1](https://github.com/borglab/gtsam)

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

To run unit tests:

```bash
~/build$ make test
```

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


