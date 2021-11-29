/**
 * @file SemanticBearingRangeFactor.h
 * @brief Bearing-range factor that incorporates semantic classes
 * @author Kurran Singh, singhk@mit.edu
 *
 * Copyright 2021 The Ambitious Folks of the MRG
 */

#pragma once

#include <gtsam/geometry/BearingRange.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/sam/BearingRangeFactor.h>
#include <math.h>

#include <algorithm>
#include <limits>
#include <type_traits>
#include <vector>

#include "DCFactor.h"

namespace dcsam {

/**
 * @brief Factor that represents bearing and range measurements
 * that incorporate a semantic class measurement.
 */
template <typename PoseType, typename PointType,
          typename BearingType =
              typename gtsam::Bearing<PoseType, PointType>::result_type,
          typename RangeType =
              typename gtsam::Range<PoseType, PointType>::result_type>
class SemanticBearingRangeFactor : public DCFactor {
 private:
  typedef SemanticBearingRangeFactor<PoseType, PointType> This;

  gtsam::BearingRangeFactor<PoseType, PointType> factor_;
  std::vector<double> probs_;

 public:
  using Base = DCFactor;

  SemanticBearingRangeFactor() = default;

  SemanticBearingRangeFactor(const gtsam::Key& poseKey,
                             const gtsam::Key& pointKey,
                             const gtsam::DiscreteKey& discreteKey,
                             const std::vector<double> measuredProbs,
                             const BearingType& measuredBearing,
                             const RangeType& measuredRange,
                             const gtsam::SharedNoiseModel& model)
      : probs_(measuredProbs),
        factor_(poseKey, pointKey, measuredBearing, measuredRange, model) {
    gtsam::KeyVector keys{poseKey, pointKey};
    gtsam::DiscreteKeys dks(discreteKey);
    keys_ = keys;
    discreteKeys_ = dks;
    gtsam::BearingRangeFactor<PoseType, PointType> brfactor;
  }

  virtual ~SemanticBearingRangeFactor() = default;

  SemanticBearingRangeFactor& operator=(const SemanticBearingRangeFactor& rhs) {
    Base::operator=(rhs);
    this->factor_ = rhs.factor_;
    this->probs_ = rhs.probs_;
    this->keys_ = rhs.keys_;
    this->discreteKeys_ = rhs.discreteKeys_;
    return *this;
  }

  // Error is the sum of the continuous and discrete negative
  // log-likelihoods
  double error(const gtsam::Values& continuousVals,
               const DiscreteValues& discreteVals) const override {
    size_t assignment = discreteVals.at(discreteKeys_[0].first);
    double discrete_error = log(probs_[assignment]);

    // Subtraction because -log(p(A,B)) = -log p(A)p(B) = -log p(A) - log p(B)
    return factor_.error(continuousVals) - discrete_error;
  }

  // dim is the dimension of the underlying bearingrange factor
  size_t dim() const override { return factor_.dim(); }

  boost::shared_ptr<gtsam::GaussianFactor> linearize(
      const gtsam::Values& continuousVals,
      const DiscreteValues& discreteVals) const override {
    return factor_.linearize(continuousVals);
  }

  bool equals(const DCFactor& other, double tol = 1e-9) const override {
    // We attempt a dynamic cast from DCFactor to SemanticBearingRangeFactor.
    // If it fails, return false.
    if (!dynamic_cast<const SemanticBearingRangeFactor*>(&other)) return false;

    // If the cast is successful, we'll properly construct a
    // SemanticBearingRangeFactor object from `other`
    const SemanticBearingRangeFactor& f(
        static_cast<const SemanticBearingRangeFactor&>(other));

    // compare the bearingrange factors
    if (!(factor_.equals(f.factor_, tol))) return false;

    // If everything above passes, and the keys_, discreteKeys_ and probs_
    // variables are identical, return true.
    return (std::equal(keys_.begin(), keys_.end(), f.keys().begin()) &&
            (discreteKeys_ == f.discreteKeys_) && (probs_ == f.probs_));
  }

  double logNormalizingConstant(const gtsam::Values& values) const override {
    return nonlinearFactorLogNormalizingConstant(this->factor_, values);
  }
};

}  // namespace dcsam
