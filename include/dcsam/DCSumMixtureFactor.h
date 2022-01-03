/**
 * @file DCSumMixtureFactor.h
 * @brief Discrete-Continuous Sum-Mixture factor.
 * @author Kevin Doherty, kdoherty@mit.edu
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
 * @brief Implementation of a discrete-continuous sum-mixture factor
 *
 * r(x) = - log (sum_i w_i * exp ( - r_i(x) ))
 *
 * We follow the derivation of Rosen et. al 2013 (RISE) and use the
 * numerically-stable "max-sum" model of Pfeiffer et al. 2021.
 */
template <class DCFactorType>
class DCSumMixtureFactor : public DCFactor {
 private:
  std::vector<DCFactorType> factors_;
  std::vector<double> log_weights_;
  bool normalized_;

 public:
  using Base = DCFactor;

  DCSumMixtureFactor() = default;

  explicit DCSumMixtureFactor(const gtsam::KeyVector& continuousKeys,
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

  explicit DCSumMixtureFactor(const gtsam::KeyVector& continuousKeys,
                              const gtsam::DiscreteKeys& discreteKeys,
                              const std::vector<DCFactorType> factors,
                              const bool normalized)
      : Base(continuousKeys, discreteKeys), normalized_(normalized) {
    factors_ = factors;
    for (size_t i = 0; i < factors_.size(); i++) {
      log_weights_.push_back(0);
    }
  }

  DCSumMixtureFactor& operator=(const DCSumMixtureFactor& rhs) {
    this->factors_ = rhs.factors_;
    this->log_weights_ = rhs.log_weights_;
    this->normalized_ = rhs.normalized_;
  }

  virtual ~DCSumMixtureFactor() = default;

  double error(const gtsam::Values& continuousVals,
               const DiscreteValues& discreteVals) const override {
    size_t min_error_idx = getActiveFactorIdx(continuousVals, discreteVals);
    double min_error =
        factors_[min_error_idx].error(continuousVals, discreteVals);
    if (normalized_) return min_error;
    return min_error +
           factors_[min_error_idx].logNormalizingConstant(continuousVals);
  }

  size_t getActiveFactorIdx(const gtsam::Values& continuousVals,
                            const DiscreteValues& discreteVals) const {
    double min_error = std::numeric_limits<double>::infinity();
    size_t min_error_idx;
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

  bool equals(const DCFactor& other, double tol = 1e-9) const {
    if (!dynamic_cast<const DCSumMixtureFactor*>(&other)) return false;
    const DCSumMixtureFactor& f(static_cast<const DCSumMixtureFactor&>(other));
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
