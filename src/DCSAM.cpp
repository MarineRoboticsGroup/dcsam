/**
 * @file DCSAM.cpp
 * @brief Discrete-Continuous Smoothing and Mapping for Factored Models
 * @author Kevin Doherty, kdoherty@mit.edu
 * Copyright 2020 The Ambitious Folks of the MRG
 *
 */

#include "dcsam/DCSAM.h"

#include "dcsam/DCContinuousFactor.h"
#include "dcsam/DCDiscreteFactor.h"
#include "dcsam/DiscreteMarginalsOrdered.h"

namespace dcsam {

DCSAM::DCSAM() {
  // Setup isam
  isam_params_.relinearizeThreshold = 0.01;
  isam_params_.relinearizeSkip = 1;
  isam_params_.setOptimizationParams(gtsam::ISAM2DoglegParams());
  isam_ = gtsam::ISAM2(isam_params_);
}

DCSAM::DCSAM(const gtsam::ISAM2Params &isam_params)
    : isam_params_(isam_params) {
  isam_ = gtsam::ISAM2(isam_params_);
}

void DCSAM::update(const gtsam::NonlinearFactorGraph &graph,
                   const gtsam::DiscreteFactorGraph &dfg,
                   const DCFactorGraph &dcfg,
                   const gtsam::Values &initialGuessContinuous,
                   const DiscreteValues &initialGuessDiscrete,
                   const gtsam::FactorIndices &removeFactorIndices,
                   const gtsam::FactorIndices &removeDiscreteFactorIndices) {

  // First things first: get rid of factors that are to be removed so updates
  // to follow take the removals into account 
  isam_.update(gtsam::NonlinearFactorGraph(), gtsam::Values(), removeFactorIndices);
  for (auto& i : removeDiscreteFactorIndices) {
    dfg_.remove(i);
  }

  // Next: combine currContinuous_ estimate with the new values
  // from initialGuessContinuous to produce the full continuous variable state.
  for (const gtsam::Key k : initialGuessContinuous.keys()) {
    if (currContinuous_.exists(k))
      currContinuous_.update(k, initialGuessContinuous.at(k));
    else
      currContinuous_.insert(k, initialGuessContinuous.at(k));
  }

  // Also combine currDiscrete_ estimate with new values from
  // initialGuessDiscrete to give a full discrete variable state.
  for (const auto &kv : initialGuessDiscrete) {
    // This will update the element with key `kv.first` if one exists, or add a
    // new element with key `kv.first` if not.
    currDiscrete_[kv.first] = initialGuessDiscrete.at(kv.first);
  }

  // We'll combine the nonlinear factors with DCContinuous factors before
  // passing to the continuous solver; likewise for the discrete factors and
  // DCDiscreteFactors.
  gtsam::NonlinearFactorGraph combined;
  gtsam::DiscreteFactorGraph discreteCombined;

  // Populate combined and discreteCombined with the provided nonlinear and
  // discrete factors, respectively.
  for (auto &factor : graph) combined.add(factor);
  for (auto &factor : dfg) discreteCombined.push_back(factor);

  // Each DCFactor will be split into a separate discrete and continuous
  // component
  for (auto &dcfactor : dcfg) {
    DCDiscreteFactor dcDiscreteFactor(dcfactor);
    auto sharedDiscrete =
        boost::make_shared<DCDiscreteFactor>(dcDiscreteFactor);
    discreteCombined.push_back(sharedDiscrete);
    dcDiscreteFactors_.push_back(sharedDiscrete);
  }

  // Set discrete information in DCDiscreteFactors.
  updateDiscrete(discreteCombined, currContinuous_, currDiscrete_);

  // Update current discrete state estimate.
  if (!initialGuessContinuous.empty() && initialGuessDiscrete.empty() &&
      discreteCombined.empty()) {
    // This is an odometry?
  } else {
    currDiscrete_ = solveDiscrete();
  }

  for (auto &dcfactor : dcfg) {
    DCContinuousFactor dcContinuousFactor(dcfactor);
    auto sharedContinuous =
        boost::make_shared<DCContinuousFactor>(dcContinuousFactor);
    sharedContinuous->updateDiscrete(currDiscrete_);
    combined.push_back(sharedContinuous);
    dcContinuousFactors_.push_back(sharedContinuous);
  }

  // Only the initialGuess needs to be provided for the continuous solver (not
  // the entire continuous state).
  updateContinuousInfo(currDiscrete_, combined, initialGuessContinuous, 
                                                    removeFactorIndices);
  currContinuous_ = isam_.calculateEstimate();
  // Update discrete info from last solve and
  updateDiscrete(discreteCombined, currContinuous_, currDiscrete_);
}

void DCSAM::update(const HybridFactorGraph &hfg,
                   const gtsam::Values &initialGuessContinuous,
                   const DiscreteValues &initialGuessDiscrete,
                   const gtsam::FactorIndices &removeFactorIndices,
                   const gtsam::FactorIndices &removeDiscreteFactorIndices) {
  update(hfg.nonlinearGraph(), hfg.discreteGraph(), hfg.dcGraph(), 
         initialGuessContinuous, initialGuessDiscrete, 
         removeFactorIndices, removeDiscreteFactorIndices);
}

void DCSAM::update() {
  update(gtsam::NonlinearFactorGraph(), gtsam::DiscreteFactorGraph(),
         DCFactorGraph());
}

void DCSAM::updateDiscrete(
    const gtsam::DiscreteFactorGraph &dfg = gtsam::DiscreteFactorGraph(),
    const gtsam::Values &continuousVals = gtsam::Values(),
    const DiscreteValues &discreteVals = DiscreteValues()) {
  for (auto &factor : dfg) {
    dfg_.push_back(factor);
  }
  updateDiscreteInfo(continuousVals, discreteVals);
}

void DCSAM::updateDiscreteInfo(const gtsam::Values &continuousVals,
                               const DiscreteValues &discreteVals) {
  if (continuousVals.empty()) return;
  for (auto factor : dcDiscreteFactors_) {
    boost::shared_ptr<DCDiscreteFactor> dcDiscrete =
        boost::static_pointer_cast<DCDiscreteFactor>(factor);
    dcDiscrete->updateContinuous(continuousVals);
    dcDiscrete->updateDiscrete(discreteVals);
  }
}

void DCSAM::updateContinuous() {
  isam_.update();
  currContinuous_ = isam_.calculateEstimate();
}

void DCSAM::updateContinuousInfo(const DiscreteValues &discreteVals,
                                 const gtsam::NonlinearFactorGraph &newFactors,
                                 const gtsam::Values &initialGuess,
                                 const gtsam::FactorIndices &removeFactorIndices) {

  gtsam::ISAM2UpdateParams updateParams;
  gtsam::FastMap<gtsam::FactorIndex, gtsam::KeySet> newAffectedKeys;
  for (size_t j = 0; j < dcContinuousFactors_.size(); j++) {
    boost::shared_ptr<DCContinuousFactor> dcContinuousFactor =
        boost::static_pointer_cast<DCContinuousFactor>(dcContinuousFactors_[j]);
    dcContinuousFactor->updateDiscrete(discreteVals);
    for (const gtsam::Key &k : dcContinuousFactor->keys()) {
      newAffectedKeys[j].insert(k);
    }
  }
  updateParams.newAffectedKeys = std::move(newAffectedKeys);
  updateParams.removeFactorIndices = removeFactorIndices; 
  // NOTE: I am not yet 100% sure this is the right way to handle this update.
  isam_.update(newFactors, initialGuess, updateParams);
}

DiscreteValues DCSAM::solveDiscrete() const {
  DiscreteValues discreteVals = (*dfg_.optimize());
  return discreteVals;
}

DCValues DCSAM::calculateEstimate() const {
  // NOTE: if we have these cached from solves, we could presumably just return
  // the cached values.
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
                     .discrete = dcsam::DiscreteMarginalsOrdered(dfg)};
}

}  // namespace dcsam
