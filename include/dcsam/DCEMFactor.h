/**
 * @file DCEMFactor.h
 * @brief Discrete-Continuous EM factor
 * @author Kevin Doherty, kdoherty@mit.edu
 * Copyright 2021 The Ambitious Folks of the MRG
 */

#pragma once

#include <math.h>

#include <algorithm>
#include <limits>
#include <utility>
#include <vector>

#include "dcsam/DCFactor.h"
#include "dcsam/DCSAM_utils.h"

namespace dcsam {

/**
 * @brief Implementation of a discrete-continuous EM factor
 *
 * The error function is defined as:
 * r(x) = sum_i w'_i * r_i(x),
 *
 * where
 * w'_i = w_i * p(z | x, h_i); sum_i w'_i = 1.
 * and h_i here represents the "i-th" hypothesis
 *
 * The error returned from this factor is a weighted combination of the
 * component factor errors. "x" can be comprised jointly of discrete and
 * continuous values. Prior hypothesis weights can be included if desired via
 * the `weights` parameter.
 */
template <class DCFactorType>
class DCEMFactor : public DCFactor {
 private:
  std::vector<DCFactorType> factors_;
  std::vector<double> log_weights_;
  bool normalized_;

 public:
  using Base = DCFactor;

  DCEMFactor() = default;

  explicit DCEMFactor(const gtsam::KeyVector& continuousKeys,
                      const gtsam::DiscreteKeys& discreteKeys,
                      const std::vector<DCFactorType> factors,
                      const std::vector<double> weights, const bool normalized)
      : Base(continuousKeys, discreteKeys), normalized_(normalized) {
    factors_ = factors;
    for (int i = 0; i < weights.size(); i++) {
      log_weights_.push_back(log(weights[i]));
    }
  }

  explicit DCEMFactor(const gtsam::KeyVector& continuousKeys,
                      const gtsam::DiscreteKeys& discreteKeys,
                      const std::vector<DCFactorType> factors,
                      const bool normalized)
      : Base(continuousKeys, discreteKeys), normalized_(normalized) {
    factors_ = factors;
    for (int i = 0; i < factors_.size(); i++) {
      log_weights_.push_back(0);
    }
  }

  DCEMFactor& operator=(const DCEMFactor& rhs) {
    this->factors_ = rhs.factors_;
    this->log_weights_ = rhs.log_weights_;
    this->normalized_ = rhs.normalized_;
  }

  virtual ~DCEMFactor() = default;

  double error(const gtsam::Values& continuousVals,
               const DiscreteValues& discreteVals) const override {
    // Retrieve the error for each component.
    std::vector<double> errors =
        computeComponentErrors(continuousVals, discreteVals);

    // Weights for each component are obtained by normalizing the errors.
    std::vector<double> componentWeights = expNormalize(errors);

    // Compute the total error as the weighted sum of component errors.
    double total_error = 0.0;
    for (size_t i = 0; i < errors.size(); i++) {
      total_error += componentWeights[i] * errors[i];
    }
    return total_error;
  }

  std::vector<double> computeComponentErrors(
      const gtsam::Values& continuousVals,
      const DiscreteValues& discreteVals) const {
    // Container for errors, where:
    //   error_i = error of component factor i - log_weights_i
    std::vector<double> errors;
    for (int i = 0; i < factors_.size(); i++) {
      double error =
          factors_[i].error(continuousVals, discreteVals) - log_weights_[i];
      if (!normalized_)
        error += factors_[i].logNormalizingConstant(continuousVals);
      errors.push_back(error);
    }
    return errors;
  }

  size_t getActiveFactorIdx(const gtsam::Values& continuousVals,
                            const DiscreteValues& discreteVals) const {
    double min_error = std::numeric_limits<double>::infinity();
    size_t min_error_idx;
    for (int i = 0; i < factors_.size(); i++) {
      double error =
          factors_[i].error(continuousVals, discreteVals) - log_weights_[i];
      if (!normalized_)
        error += factors_[i].logNormalizingConstant(continuousVals);

      if (error < min_error) {
        min_error = error;
        min_error_idx = i;
      }
    }
    return min_error_idx;
  }

  // TODO(kevin) Need to adjust Jacobian size!
  size_t dim() const override {
    size_t total = 0;
    // Each component factor `i` requires `factors_[i].dim()` rows in the
    // overall Jacobian.
    for (size_t i = 0; i < factors_.size(); i++) {
      total += factors_[i].dim();
    }
    return total;
  }

  bool equals(const DCFactor& other, double tol = 1e-9) const {
    if (!dynamic_cast<const DCEMFactor*>(&other)) return false;
    const DCEMFactor& f(static_cast<const DCEMFactor&>(other));
    if (factors_.size() != f.factors_.size()) return false;
    for (size_t i = 0; i < factors_.size(); i++) {
      if (!factors_[i].equals(f.factors_[i])) return false;
    }
    return ((log_weights_ == f.log_weights_) && (normalized_ == f.normalized_));
  }

  /*
   * Jacobian magic
   */
  boost::shared_ptr<gtsam::GaussianFactor> linearize(
      const gtsam::Values& continuousVals,
      const DiscreteValues& discreteVals) const override {
    std::vector<boost::shared_ptr<gtsam::GaussianFactor>> gfs;

    // Create a set of matrices for all keys
    std::vector<std::vector<gtsam::Matrix>> Hs;

    // Start by computing all errors, so we can get the component weights.
    std::vector<double> errors =
        computeComponentErrors(continuousVals, discreteVals);

    // Weights for each component are obtained by normalizing the errors.
    std::vector<double> componentWeights = expNormalize(errors);

    // We want to temporarily build a GaussianFactorGraph to construct the
    // Jacobian for this whole factor.
    gtsam::GaussianFactorGraph gfg;

    for (size_t i = 0; i < factors_.size(); i++) {
      // First get the GaussianFactor obtained by linearizing `factors_[i]`
      boost::shared_ptr<gtsam::GaussianFactor> gf =
          factors_[i].linearize(continuousVals, discreteVals);

      // Recover the Jacobian `A` and right-hand-side vector `b` with
      // noise models "baked in."
      std::pair<gtsam::Matrix, gtsam::Vector> jacobianAb = gf->jacobian();

      // Vector specifying vertical block dimensions. We want one block with
      // the right number of columns for jacobianAb.
      std::vector<size_t> dimensions(1, jacobianAb.first.cols());

      // Create a vertical block matrix with one vertical block of size
      // A.cols() and height equal to the number of rows in the factor.
      // Passing `appendOneDimension=true` adds a dimension for the `b` vector
      // automatically.
      gtsam::VerticalBlockMatrix Ab(dimensions, factors_[i].dim(), true);

      // Create a `JacobianFactor` from the system [A b] and add it to the
      // `GaussianFactorGraph`.
      gtsam::JacobianFactor jf(factors_[i].keys(), Ab);
      gfg.add(jf);
    }

    // Stack Jacobians to build combined factor.
    // gtsam::JacobianFactor fullJacobian(gfg);

    // size_t min_error_idx = getActiveFactorIdx(continuousVals, discreteVals);
    // return factors_[min_error_idx].linearize(continuousVals, discreteVals);

    return boost::make_shared<gtsam::JacobianFactor>(gfg);
  }

  gtsam::DecisionTreeFactor toDecisionTreeFactor(
      const gtsam::Values& continuousVals,
      const DiscreteValues& discreteVals) const override {
    size_t min_error_idx = getActiveFactorIdx(continuousVals, discreteVals);
    return factors_[min_error_idx].toDecisionTreeFactor(continuousVals,
                                                        discreteVals);
  }

  gtsam::FastVector<gtsam::Key> getAssociationKeys(
      const gtsam::Values& continuousVals,
      const DiscreteValues& discreteVals) const {
    size_t min_error_idx = getActiveFactorIdx(continuousVals, discreteVals);
    return factors_[min_error_idx].keys();
  }

  void updateWeights(const std::vector<double>& weights) {
    if (weights.size() != log_weights_.size()) {
      std::cerr << "Attempted to update weights with incorrectly sized vector."
                << std::endl;
      return;
    }
    for (int i = 0; i < weights.size(); i++) {
      log_weights_[i] = log(weights[i]);
    }
  }
};
}  // namespace dcsam
