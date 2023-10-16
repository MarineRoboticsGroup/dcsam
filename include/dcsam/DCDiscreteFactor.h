/**
 *
 * @file DCDiscreteFactor.h
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
 * (given as argument) into a discrete factor (gtsam::DiscreteFactor) that
 * can be passed to GTSAM for discrete optimization with a fixed
 * assignment to continuous variables.
 *
 * After running an iteration of discrete optimization (and separately,
 * continuous optimization), the `updateContinuous` function is used to ensure
 * the stored continuous value assignment matches the most recent estimate for
 * continuous variables.
 *
 * The continuous analogue is DCContinuousFactor.
 */
class DCDiscreteFactor : public gtsam::DiscreteFactor {
 private:
  gtsam::DiscreteKeys discreteKeys_;
  gtsam::KeyVector continuousKeys_;
  boost::shared_ptr<DCFactor> dcfactor_;
  gtsam::Values continuousVals_;
  DiscreteValues discreteVals_;

 public:
  using Base = gtsam::DiscreteFactor;

  DCDiscreteFactor() = default;

  DCDiscreteFactor(const gtsam::DiscreteKeys& discreteKeys,
                   boost::shared_ptr<DCFactor> dcfactor)
      : discreteKeys_(discreteKeys),
        continuousKeys_(dcfactor->keys()),
        dcfactor_(dcfactor) {
    // Since this is a DiscreteFactor, its `keys_` member variable stores the
    // discrete keys only.
    for (const gtsam::DiscreteKey& k : discreteKeys_) keys_.push_back(k.first);
  }

  explicit DCDiscreteFactor(boost::shared_ptr<DCFactor> dcfactor)
      : discreteKeys_(dcfactor->discreteKeys()),
        continuousKeys_(dcfactor->keys()),
        dcfactor_(dcfactor) {
    // Since this is a DiscreteFactor, its `keys_` member variable stores the
    // discrete keys only.
    for (const gtsam::DiscreteKey& k : discreteKeys_) keys_.push_back(k.first);
  }

  DCDiscreteFactor& operator=(const DCDiscreteFactor& rhs) {
    Base::operator=(rhs);
    discreteKeys_ = rhs.discreteKeys_;
    dcfactor_ = rhs.dcfactor_;
    continuousKeys_ = rhs.continuousKeys_;
    continuousVals_ = rhs.continuousVals_;
    discreteVals_ = rhs.discreteVals_;
    return *this;
  }

  virtual ~DCDiscreteFactor() = default;

  bool equals(const DiscreteFactor& other, double tol = 1e-9) const {
    if (!dynamic_cast<const DCDiscreteFactor*>(&other)) return false;
    const DCDiscreteFactor& f(static_cast<const DCDiscreteFactor&>(other));
    return (dcfactor_->equals(*f.dcfactor_) &&
            (discreteKeys_ == f.discreteKeys_) &&
            continuousVals_.equals(f.continuousVals_) &&
            discreteVals_ == f.discreteVals_);
  }

  gtsam::DecisionTreeFactor toDecisionTreeFactor() const {
    assert(allInitialized());
    return dcfactor_->toDecisionTreeFactor(continuousVals_, discreteVals_);
  }

  gtsam::DecisionTreeFactor operator*(
      const gtsam::DecisionTreeFactor& f) const {
    assert(allInitialized());
    return dcfactor_->conditionalTimes(f, continuousVals_, discreteVals_);
  }

  double operator()(const DiscreteValues& values) const {
    assert(allInitialized());
    return exp(-dcfactor_->error(continuousVals_, values));
  }

  void updateContinuous(const gtsam::Values& continuousVals) {
    for (const gtsam::Key& k : continuousKeys_) {
      // If key `k` is not set continuousVals, skip it.
      if (!continuousVals.exists(k)) continue;

      if (continuousVals_.exists(k)) {
        // If key `k` is set in stored continuousVals_, update its value
        continuousVals_.update(k, continuousVals.at(k));
      } else {
        // If key `k` is not in the stored continuousVals_, create a new entry
        // with key `k` and set its value to the one specified in the argument
        // `continuousVals`
        continuousVals_.insert(k, continuousVals.at(k));
      }
    }
  }

  void updateDiscrete(const DiscreteValues& discreteVals) {
    for (const gtsam::DiscreteKey& dk : discreteKeys_) {
      const gtsam::Key k = dk.first;
      if (discreteVals.find(k) != discreteVals.end())
        discreteVals_[k] = discreteVals.at(k);
    }
  }

  bool allInitialized() const {
    for (const gtsam::Key& k : continuousKeys_) {
      if (!continuousVals_.exists(k)) return false;
    }
    for (const gtsam::Key k : keys_) {
      if (discreteVals_.find(k) == discreteVals_.end()) return false;
    }
    return true;
  }

  std::string markdown(const gtsam::KeyFormatter& keyFormatter,
                       const Names& names) const override {
    return toDecisionTreeFactor().markdown(keyFormatter, names);
  }

  std::string html(const gtsam::KeyFormatter& keyFormatter,
                   const Names& names) const override {
    return toDecisionTreeFactor().markdown(keyFormatter, names);
  }
  
};

}  // namespace dcsam
