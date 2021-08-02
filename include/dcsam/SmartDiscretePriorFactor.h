/**
 *
 * @file SmartDiscretePriorFactor.h
 * @brief Updateable prior factor
 * @author Kevin Doherty, kdoherty@mit.edu
 * Copyright 2021 The Ambitious Folks of the MRG
 */

#pragma once

#include <vector>

#include "dcsam/DiscretePriorFactor.h"

namespace dcsam {

/**
 * @brief Implementation of a "smart" updateable discrete prior factor
 *
 * Simply augments DiscretePriorFactor with `updateProbs` function to modify the
 * `probs_` member variable directly.
 */
class SmartDiscretePriorFactor : public DiscretePriorFactor {
 public:
  using Base = DiscretePriorFactor;

  SmartDiscretePriorFactor() = default;

  SmartDiscretePriorFactor(const gtsam::DiscreteKey& dk,
                           const std::vector<double> probs)
      : Base(dk, probs) {}

  void updateProbs(const std::vector<double>& probs) {
    assert(probs.size() == probs_.size());
    probs_ = probs;
  }
};

}  // namespace dcsam
