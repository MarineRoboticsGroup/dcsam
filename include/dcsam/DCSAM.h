/**
 * @file DCSAM.h
 * @brief Discrete-Continuous Smoothing and Mapping for Factored Models
 * @author Kevin Doherty, kdoherty@mit.edu
 * Copyright 2020 The Ambitious Folks of the MRG
 */

#pragma once

#include <gtsam/discrete/DiscreteFactor.h>
#include <gtsam/discrete/DiscreteFactorGraph.h>
#include <gtsam/discrete/DiscreteKey.h>
#include <gtsam/discrete/DiscreteMarginals.h>
#include <gtsam/inference/FactorGraph.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <map>
#include <set>
#include <tuple>
#include <utility>
#include <vector>

#include "dcsam/DCContinuousFactor.h"
#include "dcsam/DCFactor.h"
#include "dcsam/DCFactorGraph.h"
#include "dcsam/DCSAM_types.h"
#include "dcsam/HybridFactorGraph.h"

namespace dcsam {

class DCSAM {
 public:
  DCSAM();

  explicit DCSAM(const gtsam::ISAM2Params &isam_params);

  /**
   * For this solver, runs an iteration of alternating minimization between
   * discrete and continuous variables, adding any user-supplied factors (with
   * initial guess) first.
   *
   * 1. Adds new discrete factors (if any) as supplied by a user to the
   * discrete factor graph, then adds any discrete-continuous factors to the
   * discrete factor graph, appropriately initializing their continuous
   * variables to those from the last solve and any supplied by the initial
   * guess.
   *
   * 2. Update the solution for the discrete variables.
   *
   * 3. For all new discrete-continuous factors to be passed to the continuous
   * solver, update/set the latest discrete variables (prior to adding).
   *
   * 4. In one step: add new factors, new values, and earmarked old factor keys
   * to iSAM. Specifically, loop over DC factors already in iSAM, updating their
   * discrete information, then call isam_.update() with the (initialized) new
   * DC factors, any new continuous factors, and the initial guess to be
   * supplied.
   *
   * 5. Calculate the latest continuous variables from iSAM.
   *
   * 6. Update the discrete factors in the discrete factor graph dfg_ with the
   * latest information from the continuous solve.
   *
   * @param graph - a gtsam::NonlinearFactorGraph containing any
   * *continuous-only* factors to add.
   * @param dfg - a gtsam::DiscreteFactorGraph containing any *discrete-only*
   * factors to add.
   * @param dcfg - a DCFactorGraph containing any joint discrete-continuous
   * factors to add.
   * @param initialGuess - an initial guess for any new continuous keys that.
   * appear in the updated factors (or if one wants to force override previously
   * obtained continuous values).
   */
  void update(const gtsam::NonlinearFactorGraph &graph,
              const gtsam::DiscreteFactorGraph &dfg, const DCFactorGraph &dcfg,
              const gtsam::Values &initialGuessContinuous = gtsam::Values(),
              const DiscreteValues &initialGuessDiscrete = DiscreteValues());

  /**
   * A HybridFactorGraph is a container holding a NonlinearFactorGraph, a
   * DiscreteFactorGraph, and a DCFactorGraph, so internally this function
   * simply issues a call to `update` with these internal graphs passed as
   * parameters: that is:
   *
   * update(hfg.nonlinearGraph(), hfg.discreteGraph(), hfg.dcGraph(),
   * initialGuess);
   */
  void update(const HybridFactorGraph &hfg,
              const gtsam::Values &initialGuessContinuous = gtsam::Values(),
              const DiscreteValues &initialGuessDiscrete = DiscreteValues());

  /**
   * Inline convenience function to allow "skipping" the initial guess for
   * continuous variables while adding an initial guess for discrete variables.
   */
  inline void update(const HybridFactorGraph &hfg,
                     const DiscreteValues &initialGuessDiscrete) {
    update(hfg, gtsam::Values(), initialGuessDiscrete);
  }

  /**
   * Simply used to call `update` without any new factors. Runs an iteration of
   * optimization.
   */
  void update();

  /**
   * Add factors in `graph` to member discrete factor graph `dfg_`, then update
   * any stored continuous variables using those in `values` by calling
   * `updateDiscreteInfo(values)`.
   *
   * NOTE: could this be combined with `updateDiscreteInfo` or do these
   * definitely need to be separate?
   *
   * @param graph - a discrete factor graph containing the factors to add
   * @param values - an assignment to the continuous variables (or subset
   * thereof).
   */
  void updateDiscrete(const gtsam::DiscreteFactorGraph &graph,
                      const gtsam::Values &continuousVals,
                      const DiscreteValues &discreteVals);

  /**
   * For any factors in `dfg_`, update their stored local continuous information
   * with the values from `values`.
   *
   * NOTE: could this be combined with `updateDiscrete` or do these
   * definitely need to be separate?
   *
   * @param values - an assignment to the continuous variables (or subset
   * thereof).
   */
  void updateDiscreteInfo(const gtsam::Values &continuousVals,
                          const DiscreteValues &discreteVals);

  /**
   * At the moment, this just calls `isam_.update()` internally
   * This could, for example, be used if one wanted to hold the discrete
   * variables constant and perform multiple iterations of updates to the
   * continuous variables.
   *
   * NOTE: this function behaves quite differently from the similarly named
   * `updateDiscrete` function, maybe we consider refactoring some of these
   * function names.
   */
  void updateContinuous();

  /**
   * Given the latest discrete values (dcValues), a set of new factors
   * (newFactors), and an initial guess for any new keys (initialGuess), this
   * function updates the continuous values stored in any DC factors (in the
   * member `isam_` instance), marks any affected keys as such, and calls
   * `isam_.update` with the new factors and initial guess. See implementation
   * for more detail.
   *
   * NOTE: this is another function that could perhaps be named better.
   */
  void updateContinuousInfo(const DiscreteValues &discreteVals,
                            const gtsam::NonlinearFactorGraph &newFactors,
                            const gtsam::Values &initialGuess);

  /**
   * Solve for discrete variables given continuous variables. Internally, calls
   * `dfg_.optimize()`
   *
   * @return an assignment (DiscreteValues) to the discrete variables in the
   * graph.
   */
  DiscreteValues solveDiscrete() const;

  /**
   * This is the primary function used to extract an estimate from the solver.
   * Internally, calls `isam_.calculateEstimate()` and `dfg_.optimize()` to
   * obtain an estimate for the continuous (resp. discrete) variables and
   * packages them into a `DCValues` pair as (continuousVals, discreteVals).
   *
   * @return a DCValues object containing an estimate
   * of the most probable assignment to the continuous (DCValues.continuous) and
   * discrete (DCValues.discrete) variables.
   */
  DCValues calculateEstimate() const;

  /**
   * Used to obtain the marginals from the solver.
   *
   * NOTE: not obviously correct (see DCSAM.cpp implementation) at the moment.
   * Should perhaps retrieve the marginals for the factor graph obtained as
   * isam_.getFactorsUnsafe() and `dfg_` rather than taking as a parameter? I
   * think this was originally intended to mimic the gtsam `Marginals` class.
   *
   * @param graph
   * @param continuousEst
   * @param dfg
   */
  DCMarginals getMarginals(const gtsam::NonlinearFactorGraph &graph,
                           const gtsam::Values &continuousEst,
                           const gtsam::DiscreteFactorGraph &dfg);

  gtsam::DiscreteFactorGraph getDiscreteFactorGraph() const { return dfg_; }

  gtsam::NonlinearFactorGraph getNonlinearFactorGraph() const {
    return isam_.getFactorsUnsafe();
  }

 private:
  // Global factor graph and iSAM2 instance
  gtsam::NonlinearFactorGraph fg_;  // NOTE: unused
  gtsam::ISAM2Params isam_params_;
  gtsam::ISAM2 isam_;
  gtsam::DiscreteFactorGraph dfg_;
  gtsam::Values currContinuous_;
  DiscreteValues currDiscrete_;


  std::vector<DCContinuousFactor::shared_ptr> dcContinuousFactors_;
  gtsam::FastVector<gtsam::DiscreteFactor::shared_ptr> dcDiscreteFactors_;
};
}  // namespace dcsam
