/**
 *
 * @file SmartSemanticBearingRangeFactor.h
 * @brief Updateable semantic bearing-range factor
 * @author Kurran Singh, singhk@mit.edu
 * Copyright 2021 The Ambitious Folks of the MRG
 */

#pragma once

#include <vector>

#include "dcsam/SemanticBearingRangeFactor.h"

namespace dcsam {

/**
 * @brief Implementation of a "smart" updateable semantic bearing-range factor
 *
 * Simply augments SemanticBearingRangeFactor with `updateProbs` function to modify the
 * `probs_` member variable directly.
 */
class SmartSemanticBearingRangeFactor : public SemanticBearingRangeFactor {
 public:
  using Base = SemanticBearingRangeFactor;

  SmartSemanticBearingRangeFactor() = default;

  SmartSemanticBearingRangeFactor(const gtsam::DiscreteKey& dk,
                           const std::vector<double> probs)
      : Base(dk, probs) {}

  void updateProbs(const std::vector<double>& probs) {
    assert(probs.size() == probs_.size());
    probs_ = probs;
  }
};

}  // namespace dcsam
