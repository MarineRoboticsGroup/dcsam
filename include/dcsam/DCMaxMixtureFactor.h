/**
 * @file DCMaxMixtureFactor.h
 * @brief Discrete-Continuous Max-Mixture factor providing several extra
 * interfaces for weight updates and association retrieval
 * @author Kurran Singh, singhk@mit.edu
 *
 * Copyright 2021 The Ambitious Folks of the MRG
 */

#pragma once

#include <math.h>

#include <algorithm>
#include <limits>
#include <vector>

#include "DCFactor.h"

namespace dcsam {

/**
 * @brief Implementation of a discrete-continuous max-mixture factor
 *
 * r(x) = min_i -log(w_i) + r_i(x)
 *
 * The error returned from this factor is the minimum error + weight
 * over all of the component factors
 * See Olson and Agarwal RSS 2012 for details
 */
template <class DCFactorType>
class DCMaxMixtureFactor : public DCFactor {
 private:
  std::vector<DCFactorType> factors_;
  std::vector<double> log_weights_;
  bool normalized_;

 public:
  using Base = DCFactor;

  DCMaxMixtureFactor() = default;

  explicit DCMaxMixtureFactor(const gtsam::KeyVector& continuousKeys,
                              const gtsam::DiscreteKeys& discreteKeys,
                              const std::vector<DCFactorType> factors,
                              const std::vector<double> weights,
                              const bool normalized)
      : Base(continuousKeys, discreteKeys), normalized_(normalized) {
    factors_ = factors;
    for (size_t i = 0; i < weights.size(); i++) {
      log_weights_.push_back(log(weights[i]));
    }
  }

  explicit DCMaxMixtureFactor(const gtsam::KeyVector& continuousKeys,
                              const gtsam::DiscreteKeys& discreteKeys,
                              const std::vector<DCFactorType> factors,
                              const bool normalized)
      : Base(continuousKeys, discreteKeys), normalized_(normalized) {
    factors_ = factors;
    for (size_t i = 0; i < factors_.size(); i++) {
      log_weights_.push_back(0);
    }
  }

  DCMaxMixtureFactor& operator=(const DCMaxMixtureFactor& rhs) {
    this->factors_ = rhs.factors_;
    this->log_weights_ = rhs.log_weights_;
    this->normalized_ = rhs.normalized_;
  }

  virtual ~DCMaxMixtureFactor() = default;

  double error(const gtsam::Values& continuousVals,
               const DiscreteValues& discreteVals) const override {
    size_t min_error_idx = getActiveFactorIdx(continuousVals, discreteVals);
    assert(0 <= min_error_idx);
    assert(min_error_idx < factors_.size());
    double min_error =
        factors_[min_error_idx].error(continuousVals, discreteVals);
    if (normalized_) return min_error - log_weights_[min_error_idx];
    return min_error +
           factors_[min_error_idx].logNormalizingConstant(continuousVals) -
           log_weights_[min_error_idx];
  }

  size_t getActiveFactorIdx(const gtsam::Values& continuousVals,
                            const DiscreteValues& discreteVals) const {
    double min_error = std::numeric_limits<double>::infinity();
    size_t min_error_idx = 0;
    for (size_t i = 0; i < factors_.size(); i++) {
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

  size_t dim() const override {
    if (factors_.size() > 0) {
      return factors_[0].dim();
    } else {
      return 0;
    }
  }

  bool equals(const DCFactor& other, double tol = 1e-9) const override {
    if (!dynamic_cast<const DCMaxMixtureFactor*>(&other)) return false;
    const DCMaxMixtureFactor& f(static_cast<const DCMaxMixtureFactor&>(other));
    if (factors_.size() != f.factors_.size()) return false;
    for (size_t i = 0; i < factors_.size(); i++) {
      if (!factors_[i].equals(f.factors_[i])) return false;
    }
    return ((log_weights_ == f.log_weights_) && (normalized_ == f.normalized_));
  }

  boost::shared_ptr<gtsam::GaussianFactor> linearize(
      const gtsam::Values& continuousVals,
      const DiscreteValues& discreteVals) const override {
    size_t min_error_idx = getActiveFactorIdx(continuousVals, discreteVals);
    return factors_[min_error_idx].linearize(continuousVals, discreteVals);
  }

  gtsam::DecisionTreeFactor uniformDecisionTreeFactor(
      const gtsam::DiscreteKey& dk) const {
    std::vector<double> probs(dk.second, (1.0 / dk.second));
    gtsam::DecisionTreeFactor uniform(dk, probs);
    return uniform;
  }

  gtsam::DecisionTreeFactor toDecisionTreeFactor(
      const gtsam::Values& continuousVals,
      const DiscreteValues& discreteVals) const override {
    size_t min_error_idx = getActiveFactorIdx(continuousVals, discreteVals);
    gtsam::DecisionTreeFactor converted;
    for (size_t i = 0; i < factors_.size(); i++) {
      if (i == min_error_idx) {
        converted = converted * factors_[min_error_idx].toDecisionTreeFactor(
                                    continuousVals, discreteVals);
      } else {
        for (const gtsam::DiscreteKey& dk : factors_[i].discreteKeys()) {
          converted = converted * uniformDecisionTreeFactor(dk);
        }
      }
    }
    return converted;
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
