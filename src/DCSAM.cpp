/**
 * @file DCSAM.cpp
 * @brief Discrete-Continuous Smoothing and Mapping for Factored Models
 * @author Kevin Doherty, kdoherty@mit.edu
 * Copyright 2021 The Ambitious Folks of the MRG
 *
 */

#include "dcsam/DCSAM.h"

#include "dcsam/DCContinuousFactor.h"
#include "dcsam/DCDiscreteFactor.h"

namespace dcsam {

DCSAM::DCSAM() {
  // Setup isam
  isam_params_.relinearizeThreshold = 0.01;
  isam_params_.relinearizeSkip = 1;
  isam_params_.setOptimizationParams(gtsam::ISAM2DoglegParams());
  isam_ = gtsam::ISAM2(isam_params_);
}

void DCSAM::update(const gtsam::NonlinearFactorGraph &graph,
                   const gtsam::DiscreteFactorGraph &dfg,
                   const DCFactorGraph &dcfg, const gtsam::Values &values) {
  gtsam::NonlinearFactorGraph combined;
  gtsam::DiscreteFactorGraph discreteCombined;

  for (auto &factor : graph) combined.add(factor);
  for (auto &factor : dfg) discreteCombined.push_back(factor);

  // Each DCFactor will be split into a separate discrete and continuous
  // component
  for (auto &dcfactor : dcfg) {
    DCDiscreteFactor dcDiscreteFactor(dcfactor);
    discreteCombined.push_back(dcDiscreteFactor);
  }

  updateDiscrete(discreteCombined, values);
  DiscreteValues currDiscrete = solveDiscrete();

  for (auto &dcfactor : dcfg) {
    // NOTE: I think maybe this should be a boost::shared_ptr to avoid copy
    // construction
    DCContinuousFactor dcContinuousFactor(dcfactor);
    dcContinuousFactor.updateDiscrete(currDiscrete);
    combined.push_back(dcContinuousFactor);
  }
  updateContinuousInfo(currDiscrete, combined, values);
  gtsam::Values currContinuous = isam_.calculateEstimate();
  updateDiscrete(discreteCombined, currContinuous);
}

void DCSAM::update(const HybridFactorGraph &hfg,
                   const gtsam::Values &initialGuess) {
  update(hfg.nonlinearGraph(), hfg.discreteGraph(), hfg.dcGraph(),
         initialGuess);
}

void DCSAM::update() {
  update(gtsam::NonlinearFactorGraph(), gtsam::DiscreteFactorGraph(),
         DCFactorGraph());
}

void DCSAM::updateDiscrete(
    const gtsam::DiscreteFactorGraph &dfg = gtsam::DiscreteFactorGraph(),
    const gtsam::Values &continuousVals = gtsam::Values()) {
  for (auto &factor : dfg) {
    dfg_.push_back(factor);
  }
  updateDiscreteInfo(continuousVals);
}

void DCSAM::updateDiscreteInfo(const gtsam::Values &continuousVals) {
  if (continuousVals.empty()) return;
  // TODO(any): inefficient, consider storing indices of DCFactors
  // Update the DC factors with new continuous information.
  for (size_t j = 0; j < dfg_.size(); j++) {
    boost::shared_ptr<DCDiscreteFactor> dcDiscrete =
        boost::dynamic_pointer_cast<DCDiscreteFactor>(dfg_[j]);
    if (dcDiscrete) {
      dcDiscrete->updateContinuous(continuousVals);
    }
  }
}

void DCSAM::updateContinuous() { isam_.update(); }

// NOTE: for iSAM(2) requires special procedure (cf Jose Luis Blanco Github
// post)
// see here:
// https://github.com/borglab/gtsam/pull/25/files#diff-277639578c861563a471b12776f86ad0b6317f61103adaae455e0cbe05899747R58
void DCSAM::updateContinuousInfo(const DiscreteValues &discreteVals,
                                 const gtsam::NonlinearFactorGraph &newFactors,
                                 const gtsam::Values &initialGuess) {
  // ISAM2UpdateParams updateParams;
  // gtsam::FastMap<gtsam::FactorIndex, gtsam::KeySet> newAffectedKeys;
  // for (size_t j = 0; j < dcContinuousFactors.size(); j++) {
  //   dcContinuousFactors[j]->updateDiscrete(discreteVals);
  //   for (const gtsam::Key &k : dcContinuousFactors[j]->keys()) {
  //     newAffectedKeys[dcIdxToFactor.at(j)].insert(k);
  //   }
  // }
  // updateParams.newAffectedKeys = std::move(newAffectedKeys);

  // NOTE: Slow for now, see above for faster method?
  gtsam::ISAM2UpdateParams updateParams;
  gtsam::FastMap<gtsam::FactorIndex, gtsam::KeySet> newAffectedKeys;

  gtsam::NonlinearFactorGraph graph = isam_.getFactorsUnsafe();
  for (size_t j = 0; j < graph.size(); j++) {
    boost::shared_ptr<DCContinuousFactor> dcContinuousFactor =
        boost::dynamic_pointer_cast<DCContinuousFactor>(graph[j]);
    if (dcContinuousFactor) {
      dcContinuousFactor->updateDiscrete(discreteVals);
      for (const gtsam::Key &k : dcContinuousFactor->keys()) {
        newAffectedKeys[j].insert(k);
      }
    }
  }
  updateParams.newAffectedKeys = std::move(newAffectedKeys);
  // NOTE: I am not yet 100% sure this is the right way to handle this update.
  isam_.update(newFactors, initialGuess, updateParams);
}

DiscreteValues DCSAM::solveDiscrete() {
  DiscreteValues discreteVals = (*dfg_.optimize());
  return discreteVals;
}

DCValues DCSAM::calculateEstimate() {
  gtsam::Values continuousVals = isam_.calculateEstimate();
  DiscreteValues discreteVals = (*dfg_.optimize());
  DCValues dcValues(continuousVals, discreteVals);
  return dcValues;
}

// NOTE separate dcmarginals class?
DCMarginals DCSAM::getMarginals(const gtsam::NonlinearFactorGraph &graph,
                                const gtsam::Values &continuousEst,
                                const gtsam::DiscreteFactorGraph &dfg) {
  return DCMarginals{.continuous = gtsam::Marginals(graph, continuousEst),
                     .discrete = gtsam::DiscreteMarginals(dfg)};
}

}  // namespace dcsam
