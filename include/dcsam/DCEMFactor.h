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
 * w'_i = w_i * p(z | x, l_i); sum_i w'_i = 1.
 *
 * The error returned from this factor is a weighted combination of the
 * component factor errors.
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
    if (factors_.size() > 0) {
      return factors_[0].dim();
    } else {
      return 0;
    }
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

  // TODO(kevin) Need to adjust linearization!
  boost::shared_ptr<gtsam::GaussianFactor> linearize(
      const gtsam::Values& continuousVals,
      const DiscreteValues& discreteVals) const override {
    std::vector<boost::shared_ptr<gtsam::GaussianFactor>> gfs;
    // Create a set of matrices for all keys
    std::vector<std::vector<gtsam::Matrix>> Hs;
    // For each factor, compute the jacobian
    for (const gtsam::Key& k : keys_) {
      for (size_t i = 0; i < factors_.size(); i++) {
        boost::shared_ptr<gtsam::GaussianFactor> gf =
            factors_[i].linearize(continuousVals, discreteVals);
        // for each key in factor keys, get the appropriate jacobian and add it
        // to the stack?
        std::pair<gtsam::Matrix, gtsam::Vector> jacobianAb = gf->jacobian();
      }
    }
    // Stack Jacobians
    size_t min_error_idx = getActiveFactorIdx(continuousVals, discreteVals);
    return factors_[min_error_idx].linearize(continuousVals, discreteVals);
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
