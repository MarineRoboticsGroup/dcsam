/**
 * @file DCContinuousFactor.h
 * @brief Custom discrete-continuous factor
 * @author Kevin Doherty, kdoherty@mit.edu
 * Copyright 2021 The Ambitious Folks of the MRG
 */

#pragma once

#include <gtsam/discrete/DiscreteFactor.h>
#include <gtsam/discrete/DiscreteKey.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/Symbol.h>
#include <math.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <vector>

#include "DCFactor.h"
#include "DCSAM_types.h"

namespace dcsam {

/**
 * @brief Implementation of a discrete-continuous factor. This is used
 * *internally* within the DCSAM solver. Essentially this class wraps a DCFactor
 * (given as argument) into a continuous factor (gtsam::NonlinearFactor) that
 * can be passed to GTSAM/iSAM/iSAM2 for continuous optimization with a fixed
 * assignment to discrete variables.
 *
 * After running an iteration of continuous optimization (and separately,
 * discrete optimization), the `updateDiscrete` function is used to ensure the
 * stored discrete value assignment matches the most recent estimate for
 * discrete variables.
 *
 * The discrete analogue is DCDiscreteFactor.
 */
class DCContinuousFactor : public gtsam::NonlinearFactor {
 private:
  gtsam::DiscreteKeys discreteKeys_;
  boost::shared_ptr<DCFactor> dcfactor_;
  DiscreteValues discreteVals_;

 public:
  using Base = gtsam::NonlinearFactor;

  DCContinuousFactor() = default;

  explicit DCContinuousFactor(boost::shared_ptr<DCFactor> dcfactor)
      : discreteKeys_(dcfactor->discreteKeys()), dcfactor_(dcfactor) {
    keys_ = dcfactor->keys();
  }

  double error(const gtsam::Values& continuousVals) const override {
    assert(allInitialized());
    return dcfactor_->error(continuousVals, discreteVals_);
  }

  boost::shared_ptr<gtsam::GaussianFactor> linearize(
      const gtsam::Values& continuousVals) const override {
    assert(allInitialized());
    return dcfactor_->linearize(continuousVals, discreteVals_);
  }

  DCContinuousFactor& operator=(const DCContinuousFactor& rhs) {
    Base::operator=(rhs);
    discreteKeys_ = rhs.discreteKeys_;
    dcfactor_ = rhs.dcfactor_;
    discreteVals_ = rhs.discreteVals_;
    return *this;
  }

  ~DCContinuousFactor() = default;

  void updateDiscrete(const DiscreteValues& discreteVals) {
    for (const gtsam::DiscreteKey& dk : discreteKeys_) {
      const gtsam::Key k = dk.first;
      if (discreteVals.find(k) != discreteVals.end())
        discreteVals_[k] = discreteVals.at(k);
    }
  }

  size_t dim() const override { return dcfactor_->dim(); }

  bool allInitialized() const {
    for (const gtsam::DiscreteKey& dk : discreteKeys_) {
      const gtsam::Key k = dk.first;
      if (discreteVals_.find(k) == discreteVals_.end()) return false;
    }
    return true;
  }
};

}  // namespace dcsam
