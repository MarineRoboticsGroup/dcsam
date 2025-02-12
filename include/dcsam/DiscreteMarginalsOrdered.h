/**
 *
 * @file DiscreteMarginalsOrdererd.h
 * @brief Custom discrete marginals
 * @author Kevin Doherty, kdoherty@mit.edu
 * Copyright 2021 The Ambitious Folks of the MRG
 */

#pragma once

#include <gtsam/discrete/DiscreteFactorGraph.h>
#include <gtsam/discrete/DiscreteMarginals.h>

#include <utility>

#include "DCSAM_types.h"

namespace dcsam {

/**
 * @brief Simple discrete marginals class allowing specific ordering
 */
class DiscreteMarginalsOrdered : public gtsam::DiscreteMarginals {
 public:
  using Base = gtsam::DiscreteMarginals;
  DiscreteMarginalsOrdered(const gtsam::DiscreteFactorGraph &graph,
                           const gtsam::Ordering::OrderingType &orderingType =
                               gtsam::Ordering::OrderingType::NATURAL)
      : Base(gtsam::DiscreteFactorGraph()) {
    gtsam::Ordering ordering;
    if (orderingType == gtsam::Ordering::OrderingType::COLAMD) {
      ordering = gtsam::Ordering::Colamd(graph);
    } else if (orderingType == gtsam::Ordering::OrderingType::METIS) {
      ordering = gtsam::Ordering::Metis(graph);
    } else {
      ordering = gtsam::Ordering::Natural(graph);
    }
    bayesTree_ = graph.eliminateMultifrontal(ordering, CustomEliminateDiscrete);
  }

  static std::pair<gtsam::DiscreteConditional::shared_ptr,
                   gtsam::DiscreteFactor::shared_ptr>
  CustomEliminateDiscrete(const gtsam::DiscreteFactorGraph &factors,
                          const gtsam::Ordering &frontalKeys) {
    // PRODUCT: multiply all factors.
    const auto product = factors.scaledProduct();

    // sum out frontals to get the factor on the separator.
    const auto sum = product->sum(frontalKeys);

    // NOTE: Sum keys seems to be empty often - is this normal?
    // Ordering keys for the conditional so that frontalKeys is in front.
    gtsam::Ordering orderedKeys;
    orderedKeys.insert(orderedKeys.end(), frontalKeys.begin(),
                       frontalKeys.end());
    orderedKeys.insert(orderedKeys.end(), sum->keys().begin(),
                       sum->keys().end());

    const auto cond = std::make_shared<gtsam::DiscreteConditional>(
        product->toDecisionTreeFactor(), sum->toDecisionTreeFactor(),
        orderedKeys);

    return {cond, sum};
  }
};

}  // namespace dcsam
