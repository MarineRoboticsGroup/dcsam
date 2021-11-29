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
    bayesTree_ = graph.eliminateMultifrontal(ordering);
  }
};

}  // namespace dcsam
