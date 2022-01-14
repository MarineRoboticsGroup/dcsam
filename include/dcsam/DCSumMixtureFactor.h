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
 * r(x) = - log (sum_i w_i * eta_i exp ( - r_i(x) )),
 *
 * where eta_i is the normalization constant for the i-th Gaussian component and
 * w_i is the corresponding weight.
 *
 * We follow the derivation of Rosen et. al 2013 (RISE) and use numerically
 * stable log-sum-exp for sum-mixtures of Gaussians, as in Pfeiffer et al. 2021.
 *
 * Requires computation of "beta" an upper-bound on the probability of the
 * observed data for any assignment to the unknown variables in `keys`. For
 * sum-mixtures of Gaussians, it suffices to consider beta := sum_i w_i * eta_i.
 */
template <class DCFactorType>
class DCSumMixtureFactor : public DCFactor {
 private:
  std::vector<DCFactorType> factors_;
  std::vector<double> log_weights_;
  bool normalized_;
  double log_beta_;  // A constant upper-bound on p(observed | variables)

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

    // Compute `beta`. NOTE: DCFactor logNormalizingConstant requires that we
    // pass continuous values here, even though they are not used.
    // We simply pass an empty set of Values
    std::vector<double> logWeightedNormalizingConstants;
    for (size_t i = 0; i < factors_.size(); i++) {
      // factors_[i].logNormalizingConstant returns the *negative* log of
      // the normalizing constant for factors_[i].
      double logNormalizingConstant =
          -factors_[i].logNormalizingConstant(gtsam::Values());
      logWeightedNormalizingConstants.push_back(logNormalizingConstant +
                                                log_weights_[i]);
    }

    // Since beta = sum_i (w_i * eta_i), we have:
    // log beta = log sum_i (w_i * eta_i)
    //          = log sum_i exp(log (w_i * eta_i))
    //          = log sum_i exp(log w_i + log eta_i)
    log_beta_ = logSumExp(logWeightedNormalizingConstants);
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
    this->log_beta_ = rhs.log_beta_;
    this->normalized_ = rhs.normalized_;
  }

  virtual ~DCSumMixtureFactor() = default;

  double error(const gtsam::Values& continuousVals,
               const DiscreteValues& discreteVals) const override {
    // Retrieve the log prob for each component.
    std::vector<double> logprobs =
        computeComponentLogProbs(continuousVals, discreteVals);

    // Weights for each component are obtained by normalizing the errors.
    std::vector<double> componentWeights = expNormalize(logprobs);

    // Compute the total error as the weighted sum of component errors.
    double total_error = 0.0;
    for (size_t i = 0; i < logprobs.size(); i++) {
      total_error += componentWeights[i] * (-logprobs[i]);
    }
    return total_error;
  }

  /**
   * Compute the square-root residual function defined as:
   *
   * sqrt_res := sqrt(log_beta_ - log sum_i w_i * eta_i * exp ( ... ))
   *
   * Per Rosen et al. 2013.
   */
  double sqrt_residual(const gtsam::Values& continuousVals,
                       const DiscreteValues& discreteVals) const {
    return sqrt(log_beta_ - error(continuousVals, discreteVals));
  }

  std::vector<double> computeComponentLogProbs(
      const gtsam::Values& continuousVals,
      const DiscreteValues& discreteVals) const {
    // Container for errors, where:
    //   error_i = error of component factor i - log_weights_i
    std::vector<double> logprobs;
    for (size_t i = 0; i < factors_.size(); i++) {
      double error =
          factors_[i].error(continuousVals, discreteVals) - log_weights_[i];
      if (!normalized_)
        error += factors_[i].logNormalizingConstant(continuousVals);
      logprobs.push_back(-error);
    }
    return logprobs;
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
    return ((log_weights_ == f.log_weights_) &&
            (normalized_ == f.normalized_) &&
            gtsam::fpEqual(log_beta_, f.log_beta_, tol));
  }

  boost::shared_ptr<gtsam::GaussianFactor> linearize(
      const gtsam::Values& continuousVals,
      const DiscreteValues& discreteVals) const override {
    size_t min_error_idx = getActiveFactorIdx(continuousVals, discreteVals);

    // Start by computing all errors, so we can get the component weights.
    std::vector<double> logprobs =
        computeComponentLogProbs(continuousVals, discreteVals);

    // Weights for each component are obtained by normalizing the errors.
    std::vector<double> componentWeights = expNormalize(logprobs);

    for (size_t i = 0; i < factors_.size(); i++) {
      // std::cout << "i = " << i << std::endl;
      // First get the GaussianFactor obtained by linearizing `factors_[i]`
      boost::shared_ptr<gtsam::GaussianFactor> gf =
          factors_[i].linearize(continuousVals, discreteVals);

      gtsam::JacobianFactor jf_component(*gf);

      // Recover the [A b] matrix with Jacobian A and right-hand side vector b,
      // with noise models "baked in," as a vertical block matrix.
      // A = Sigma^(1/2)*J
      gtsam::VerticalBlockMatrix Ab = jf_component.matrixObject();

      gtsam::Vector whitenedError = jf_component.error_vector(
          continuousVals.localCoordinates(continuousVals));

      // Copy Ab so we can reweight it appropriately.
      gtsam::VerticalBlockMatrix Ab_weighted = Ab;

      // Populate Ab_weighted with weighted Jacobian sqrt(w)*A and right-hand
      // side vector sqrt(w)*b.
      double sqrt_weight = sqrt(componentWeights[i]);

      for (size_t k = 0; k < Ab_weighted.nBlocks(); k++) {
        Ab_weighted(k) = sqrt_weight * Ab(k);
      }

      // Create a `JacobianFactor` from the system [A b] and add it to the
      // `GaussianFactorGraph`.
      gtsam::JacobianFactor jf(factors_[i].keys(), Ab_weighted);
      // gfg.add(jf);
    }

    // Get component for "dominant" factor
    boost::shared_ptr<gtsam::GaussianFactor> gf_max =
        factors_[min_error_idx].linearize(continuousVals, discreteVals);

    return gf_max;
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
