/**
 *
 * @file DiscretePriorFactor.h
 * @brief Discrete prior factor
 * @author Kevin Doherty, kdoherty@mit.edu
 * Copyright 2021 The Ambitious Folks of the MRG
 */

#pragma once

#include <gtsam/discrete/DiscreteFactor.h>
#include <gtsam/discrete/DiscreteKey.h>

#include <vector>

namespace dcsam {

/**
 * @brief Implementation of a discrete prior factor
 *
 * This factor specifies a prior distribution over a discrete variable. The user
 * provides a discrete key `dk` consisting of a key (e.g. `gtsam::Symbol`) and
 * the cardinality of the discrete variable. The vector `probs` specifies a
 * distribution over the possible values that could be taken by the variable.
 *
 * For example, for a variable `d1` with 2 possible values, p(d1 = i) = probs[i]
 * with i being 0 or 1. The length of the vector `probs` therefore must be equal
 * to the cardinality of the discrete variable.
 */
class DiscretePriorFactor : public gtsam::DiscreteFactor {
 protected:
  gtsam::DiscreteKey dk_;
  std::vector<double> probs_;

 public:
  using Base = gtsam::DiscreteFactor;

  DiscretePriorFactor() = default;

  DiscretePriorFactor(const gtsam::DiscreteKey& dk,
                      const std::vector<double> probs)
      : dk_(dk), probs_(probs) {
    // Ensure that length of probs is equal to the cardinality of the discrete
    // variable (for gtsam::DiscreteKey dk, dk.second is the cardinality).
    assert(probs.size() == dk.second);

    // For gtsam::DiscreteKey dk, dk.first is the Key for this variable.
    keys_.push_back(dk.first);
  }

  bool equals(const DiscreteFactor& other, double tol = 1e-9) const {
    if (!dynamic_cast<const DiscretePriorFactor*>(&other)) return false;
    const DiscretePriorFactor& f(
        static_cast<const DiscretePriorFactor&>(other));
    if (probs_.size() != f.probs_.size() || (dk_ != f.dk_)) {
      return false;
    } else {
      for (size_t i = 0; i < probs_.size(); i++)
        if (abs(probs_[i] - f.probs_[i]) > tol) return false;
      return true;
    }
  }

  DiscretePriorFactor& operator=(const DiscretePriorFactor& rhs) {
    Base::operator=(rhs);
    dk_ = rhs.dk_;
    probs_ = rhs.probs_;
    return *this;
  }

  gtsam::DecisionTreeFactor toDecisionTreeFactor() const {
    gtsam::DecisionTreeFactor converted(dk_, probs_);
    return converted;
  }

  gtsam::DecisionTreeFactor operator*(
      const gtsam::DecisionTreeFactor& f) const {
    return toDecisionTreeFactor() * f;
  }

  double operator()(const DiscreteValues& values) const {
    size_t assignment = values.at(dk_.first);
    return probs_[assignment];
  }
};

}  // namespace dcsam
