/**
 * @file HybridFactorGraph.h
 * @brief Custom hybrid factor graph for discrete + continuous factors
 * @author Kevin Doherty, kdoherty@mit.edu
 * Copyright 2021 The Ambitious Folks of the MRG
 */

#pragma once

#include <gtsam/discrete/DiscreteFactor.h>
#include <gtsam/discrete/DiscreteFactorGraph.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <string>

#include "DCSAM_types.h"
#include "dcsam/DCFactor.h"
#include "dcsam/DCFactorGraph.h"

namespace dcsam {

class HybridFactorGraph {
 protected:
  // Separate internal factor graphs for different types of factors
  gtsam::NonlinearFactorGraph nonlinearGraph_;
  gtsam::DiscreteFactorGraph discreteGraph_;
  DCFactorGraph dcGraph_;

 public:
  HybridFactorGraph();

  /**
   * Add a nonlinear factor to the internal nonlinear factor graph
   * @param nonlinearFactor - the factor to add
   */
  template <typename NonlinearFactorType>
  void push_nonlinear(const NonlinearFactorType &nonlinearFactor) {
    nonlinearGraph_.push_back(
        boost::make_shared<NonlinearFactorType>(nonlinearFactor));
  }

  /**
   * Add a nonlinear factor *pointer* to the internal nonlinear factor graph
   * @param nonlinearFactor - boost::shared_ptr to the factor to add
   */
  void push_nonlinear(
      boost::shared_ptr<gtsam::NonlinearFactor> nonlinearFactor);

  /**
   * Add a discrete factor to the internal discrete graph
   * @param discreteFactor - the factor to add
   */
  template <typename DiscreteFactorType>
  void push_discrete(const DiscreteFactorType &discreteFactor) {
    discreteGraph_.push_back(
        boost::make_shared<DiscreteFactorType>(discreteFactor));
  }

  /**
   * Add a discrete factor *pointer* to the internal discrete graph
   * @param discreteFactor - boost::shared_ptr to the factor to add
   */
  void push_discrete(boost::shared_ptr<gtsam::DiscreteFactor> discreteFactor);

  /**
   * Add a discrete-continuous (DC) factor to the internal DC graph
   * @param dcFactor - the factor to add
   */
  template <typename DCFactorType>
  void push_dc(const DCFactorType &dcFactor) {
    dcGraph_.push_back(boost::make_shared<DCFactorType>(dcFactor));
  }

  /**
   * Add a discrete-continuous (DC) factor *pointer* to the internal DC graph
   * @param dcFactor - boost::shared_ptr to the factor to add
   */
  void push_dc(boost::shared_ptr<DCFactor> dcFactor);

  /**
   * Simply prints the factor graph.
   */
  void print(const std::string &str = "HybridFactorGraph",
             const gtsam::KeyFormatter &keyFormatter =
                 gtsam::DefaultKeyFormatter) const;

  /**
   * Mimics the GTSAM::FactorGraph API: retrieve the keys from each internal
   * factor graph. Internally uses FastSet::merge(const FastSet &other) to
   * combine sets from the different member factor graphs.
   *
   * @return the (aggregate) set of keys in all of the internal factor graphs.
   */
  gtsam::FastSet<gtsam::Key> keys() const;

  /**
   * Utility for retrieving the internal nonlinear factor graph
   * @return the member variable nolinearGraph_
   */
  gtsam::NonlinearFactorGraph nonlinearGraph() const;

  /**
   * Utility for retrieving the internal discrete factor graph
   * @return the member variable discreteGraph_
   */
  gtsam::DiscreteFactorGraph discreteGraph() const;

  /**
   * Utility for retrieving the internal DC factor graph
   * @return the member variable dcGraph_
   */
  DCFactorGraph dcGraph() const;

  /**
   * @return true if all internal graphs are empty
   */
  bool empty() const;

  /**
   * @return true if all internal graphs of `this` are equal to those of `other`
   */
  bool equals(const HybridFactorGraph &other, double tol = 1e-9) const;

  /**
   * @return the total number of factors across all internal graphs
   */
  size_t size() const;

  /**
   * @return the total number of factors in the nonlinear factor graph
   */
  size_t size_nonlinear() const;

  /**
   * @return the total number of factors in the discrete factor graph
   */
  size_t size_discrete() const;

  /**
   * @return the total number of factors in the DC factor graph
   */
  size_t size_dc() const;

  /**
   * Clears all internal factor graphs
   */
  void clear();
};

}  // namespace dcsam
