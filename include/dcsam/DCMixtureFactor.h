/**
 * @file DCMixtureFactor.h
 * @brief DC Mixture factor
 * @author Kevin Doherty, kdoherty@mit.edu
 *
 * Copyright 2021 The Ambitious Folks of the MRG
 */

#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/Symbol.h>
#include <math.h>

#include <algorithm>
#include <limits>
#include <vector>

#include "DCFactor.h"

namespace dcsam {

/**
 * @brief Implementation of a discrete conditional mixture factor. Implements a
 * joint discrete-continuous factor where the discrete variable serves to
 * "select" a mixture component corresponding to a gtsam::NonlinearFactor type
 * of measurement.
 */
template <class NonlinearFactorType>
class DCMixtureFactor : public DCFactor {
 private:
  gtsam::DiscreteKey dk_;
  std::vector<NonlinearFactorType> factors_;
  bool normalized_;

 public:
  using Base = DCFactor;

  DCMixtureFactor() = default;

  DCMixtureFactor(const gtsam::KeyVector& keys, const gtsam::DiscreteKey& dk,
                  const std::vector<NonlinearFactorType>& factors,
                  bool normalized = false)
      : dk_(dk), factors_(factors), normalized_(normalized) {
    // Compiler doesn't like `keys_` in the initializer list.
    keys_ = keys;

    // Add `dk` to `dkeys` list.
    discreteKeys_.push_back(dk);
  }

  DCMixtureFactor& operator=(const DCMixtureFactor& rhs) {
    Base::operator=(rhs);
    this->dk_ = rhs.dk_;
    this->factors_ = rhs.factors_;
  }

  ~DCMixtureFactor() = default;

  double error(const gtsam::Values& continuousVals,
               const DiscreteValues& discreteVals) const override {
    // Retrieve the assignment to our discrete key.
    const size_t assignment = discreteVals.at(dk_.first);

    // `assignment` indexes the nonlinear factors we have stored to compute the
    // error.
    const double factorError = factors_[assignment].error(continuousVals);
    if (normalized_) return factorError;
    return factorError + this->nonlinearFactorLogNormalizingConstant(
                             this->factors_[assignment], continuousVals);
  }

  size_t dim() const override {
    // TODO(kevin) Need to modify this? Maybe we take discrete vals as parameter
    // and DCContinuousFactor will pass this in as needed.
    return (factors_.size() > 0) ? factors_[0].dim() : 0;
  }

  bool equals(const DCFactor& other, double tol = 1e-9) const override {
    // We attempt a dynamic cast from DCFactor to DCMixtureFactor. If it fails,
    // return false.
    if (!dynamic_cast<const DCMixtureFactor*>(&other)) return false;

    // If the cast is successful, we'll properly construct a DCMixtureFactor
    // object from `other`
    const DCMixtureFactor& f(static_cast<const DCMixtureFactor&>(other));

    // Ensure that this DCMixtureFactor and `f` have the same number of
    // component factors in `factors_`.
    if (factors_.size() != f.factors_.size()) return false;

    // If the number of factors is the same, we compare them individually (they
    // should be in the same order!). If any fail to match, return false.
    for (size_t i = 0; i < factors_.size(); i++) {
      if (!factors_[i].equals(f.factors_[i])) return false;
    }

    // If everything above passes, and the keys_, discreteKeys_ and normalized_
    // member variables are identical, return true.
    return (std::equal(keys_.begin(), keys_.end(), f.keys().begin()) &&
            (discreteKeys_ == f.discreteKeys_) &&
            (normalized_ == f.normalized_));
  }

  boost::shared_ptr<gtsam::GaussianFactor> linearize(
      const gtsam::Values& continuousVals,
      const DiscreteValues& discreteVals) const override {
    // Retrieve the assignment to our discrete key.
    const size_t assignment = discreteVals.at(dk_.first);

    // `assignment` indexes the nonlinear factors we have stored to compute the
    // error.
    return factors_[assignment].linearize(continuousVals);
  }
};

}  // namespace dcsam
