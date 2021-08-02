/**
 * @file HybridFactorGraph.cpp
 * @brief Custom hybrid factor graph for discrete + continuous factors
 * @author Kevin Doherty, kdoherty@mit.edu
 * Copyright 2021 The Ambitious Folks of the MRG
 */

#include "dcsam/HybridFactorGraph.h"

namespace dcsam {

HybridFactorGraph::HybridFactorGraph() {}

void HybridFactorGraph::push_nonlinear(
    boost::shared_ptr<gtsam::NonlinearFactor> nonlinearFactor) {
  nonlinearGraph_.push_back(nonlinearFactor);
}

void HybridFactorGraph::push_discrete(
    boost::shared_ptr<gtsam::DiscreteFactor> discreteFactor) {
  discreteGraph_.push_back(discreteFactor);
}

void HybridFactorGraph::push_dc(boost::shared_ptr<DCFactor> dcFactor) {
  dcGraph_.push_back(dcFactor);
}

void HybridFactorGraph::print(const std::string &str,
                              const gtsam::KeyFormatter &keyFormatter) const {
  std::string nonlinearStr = str + ": NonlinearFactorGraph";
  std::string discreteStr = str + ": DiscreteFactorGraph";
  std::string dcStr = str + ": DCFactorGraph";

  nonlinearGraph_.print(nonlinearStr, keyFormatter);
  discreteGraph_.print(discreteStr, keyFormatter);
  dcGraph_.print(dcStr, keyFormatter);
}

gtsam::FastSet<gtsam::Key> HybridFactorGraph::keys() const {
  gtsam::FastSet<gtsam::Key> keys;
  // Combine keys from all the internal graphs
  keys.merge(nonlinearGraph_.keys());
  keys.merge(discreteGraph_.keys());
  keys.merge(dcGraph_.keys());
  return keys;
}

gtsam::NonlinearFactorGraph HybridFactorGraph::nonlinearGraph() const {
  return nonlinearGraph_;
}

gtsam::DiscreteFactorGraph HybridFactorGraph::discreteGraph() const {
  return discreteGraph_;
}

DCFactorGraph HybridFactorGraph::dcGraph() const { return dcGraph_; }

bool HybridFactorGraph::empty() const {
  return nonlinearGraph_.empty() && discreteGraph_.empty() && dcGraph_.empty();
}

bool HybridFactorGraph::equals(const HybridFactorGraph &other,
                               double tol) const {
  return nonlinearGraph_.equals(other.nonlinearGraph_, tol) &&
         discreteGraph_.equals(other.discreteGraph_, tol) &&
         dcGraph_.equals(other.dcGraph_, tol);
}

size_t HybridFactorGraph::size() const {
  return nonlinearGraph_.size() + discreteGraph_.size() + dcGraph_.size();
}

size_t HybridFactorGraph::size_nonlinear() const {
  return nonlinearGraph_.size();
}

size_t HybridFactorGraph::size_discrete() const {
  return discreteGraph_.size();
}

size_t HybridFactorGraph::size_dc() const { return dcGraph_.size(); }

void HybridFactorGraph::clear() {
  nonlinearGraph_.resize(0);
  discreteGraph_.resize(0);
  dcGraph_.resize(0);
}

}  // namespace dcsam
