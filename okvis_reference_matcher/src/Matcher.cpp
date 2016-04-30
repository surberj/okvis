/*********************************************************************************
 *  OKVIS - Open Keyframe-based Visual-Inertial SLAM
 *  Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 * 
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *   * Neither the name of Autonomous Systems Lab / ETH Zurich nor the names of
 *     its contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Created on: Oct 17, 2013
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *    Modified: Andreas Forster (an.forster@gmail.com)
 *********************************************************************************/

/**
 * @file VioKeyframeWindowMatchingAlgorithm.cpp
 * @brief Source file for the VioKeyframeWindowMatchingAlgorithm class.
 * @author Stefan Leutenegger
 * @author Andreas Forster
 */

#include <okvis/Matcher.hpp>
#include <okvis/ceres/ReprojectionError.hpp>
#include <okvis/IdProvider.hpp>
#include <okvis/cameras/CameraBase.hpp>
#include <okvis/MultiFrame.hpp>

// cameras and distortions
#include <okvis/cameras/PinholeCamera.hpp>
#include <okvis/cameras/EquidistantDistortion.hpp>
#include <okvis/cameras/RadialTangentialDistortion.hpp>
#include <okvis/cameras/RadialTangentialDistortion8.hpp>

#include <opencv2/features2d/features2d.hpp> // for cv::KeyPoint

/// \brief okvis Main namespace of this package.
namespace okvis {

// Constructor.
template<class CAMERA_GEOMETRY_T>
Matcher<CAMERA_GEOMETRY_T>::Matcher(
    int matchingType, float distanceThreshold,
    bool usePoseUncertainty) {
  matchingType_ = matchingType;
  distanceThreshold_ = distanceThreshold;
  usePoseUncertainty_ = usePoseUncertainty;
}

template<class CAMERA_GEOMETRY_T>
Matcher<CAMERA_GEOMETRY_T>::~Matcher() {
}

// Set which frames to match.
template<class CAMERA_GEOMETRY_T>
void Matcher<CAMERA_GEOMETRY_T>::setFrames(
    std::shared_ptr<okvis::MultiFrame> currentFrame, std::shared_ptr<okvis::MultiFrame> referenceFrame) {

  // frames and related information
  frameA_ = currentFrame;   // TODO: get probably from estimator or frontend or whereever from OKVIS
  frameB_ = referenceFrame; // TODO: get from reference map

  // remember indices
  mfIdA_ = 0;
  mfIdB_ = 1;
  camIdA_ = 0;
  camIdB_ = 0;

  // focal length
  fA_ = frameA_->geometryAs<CAMERA_GEOMETRY_T>(camIdA_)->focalLengthU();
  fB_ = frameB_->geometryAs<CAMERA_GEOMETRY_T>(camIdB_)->focalLengthU();

  // calculate the relative transformations and uncertainties
  // TODO donno, if and what we need here - I'll see

  validRelativeUncertainty_ = false;
}

// What is the size of list A?
template<class CAMERA_GEOMETRY_T>
size_t VioKeyframeWindowMatchingAlgorithm<CAMERA_GEOMETRY_T>::sizeA() const {
  return frameA_->numKeypoints(camIdA_);
}
// What is the size of list B?
template<class CAMERA_GEOMETRY_T>
size_t VioKeyframeWindowMatchingAlgorithm<CAMERA_GEOMETRY_T>::sizeB() const {
  return frameB_->numKeypoints(camIdB_);
}

// A function that tells you how many times setMatching() will be called.
template<class CAMERA_GEOMETRY_T>
void VioKeyframeWindowMatchingAlgorithm<CAMERA_GEOMETRY_T>::reserveMatches(
    size_t /*numMatches*/) {
  //_triangulatedPoints.clear();
}

// Get the distance threshold for which matches exceeding it will not be returned as matches.
template<class CAMERA_GEOMETRY_T>
float VioKeyframeWindowMatchingAlgorithm<CAMERA_GEOMETRY_T>::distanceThreshold() const {
  return distanceThreshold_;
}

// At the end of the matching step, this function is called once
// for each pair of matches discovered.
template<class CAMERA_GEOMETRY_T>
void VioKeyframeWindowMatchingAlgorithm<CAMERA_GEOMETRY_T>::setBestMatch(
    size_t indexA, size_t indexB, double /*distance*/) {

  // assign correspondences
  uint64_t lmIdA = frameA_->landmarkId(camIdA_, indexA);
  uint64_t lmIdB = frameB_->landmarkId(camIdB_, indexB);

  if (matchingType_ == Match2D2D) {

    // check that not both are set
    if (lmIdA != 0 && lmIdB != 0) {
      return;
    }

    // re-triangulate...
    // potential 2d2d match - verify by triangulation
    Eigen::Vector4d hP_Ca;
    bool canBeInitialized;
    bool valid = probabilisticStereoTriangulator_.stereoTriangulate(
        indexA, indexB, hP_Ca, canBeInitialized,
        std::max(raySigmasA_[indexA], raySigmasB_[indexB]));
    if (!valid) {
      return;
    }

    // get the uncertainty
    if (canBeInitialized) {  // know more exactly
      Eigen::Matrix3d pointUOplus_A;
      probabilisticStereoTriangulator_.getUncertainty(indexA, indexB, hP_Ca,
                                                      pointUOplus_A,
                                                      canBeInitialized);
    }

    // check and adapt landmark status
    bool insertA = lmIdA == 0;
    bool insertB = lmIdB == 0;
    bool insertHomogeneousPointParameterBlock = false;
    uint64_t lmId = 0;  // 0 just to avoid warning
    if (insertA && insertB) {
      // ok, we need to assign a new Id...
      lmId = okvis::IdProvider::instance().newId();
      frameA_->setLandmarkId(camIdA_, indexA, lmId);
      frameB_->setLandmarkId(camIdB_, indexB, lmId);
      lmIdA = lmId;
      lmIdB = lmId;
      // and add it to the graph
      insertHomogeneousPointParameterBlock = true;
    } else {
      if (!insertA) {
        lmId = lmIdA;
        insertHomogeneousPointParameterBlock = true;
        insertA = true;
      }
      if (!insertB) {
        lmId = lmIdB;
        insertHomogeneousPointParameterBlock = true;
        insertB = true;
        }
      }
    }
    // add landmark to graph if necessary
    if (insertHomogeneousPointParameterBlock) {
      LOG(WARNING) << "add match here to cv::DMatch.";
    }

    // in image A
    okvis::MapPoint landmark;
    if (insertA
        && landmark.observations.find(
            okvis::KeypointIdentifier(mfIdA_, camIdA_, indexA))
            == landmark.observations.end()) {  // ensure no double observations...
            // TODO hp_Sa NOT USED!
      Eigen::Vector4d hp_Sa(T_SaCa_ * hP_Ca);
      hp_Sa.normalize();
      frameA_->setLandmarkId(camIdA_, indexA, lmId);
      lmIdA = lmId;
      LOG(WARNING) << "add landmark to frame A here.";
    }

    // in image B
    if (insertB
        && landmark.observations.find(
            okvis::KeypointIdentifier(mfIdB_, camIdB_, indexB))
            == landmark.observations.end()) {  // ensure no double observations...
      Eigen::Vector4d hp_Sb(T_SbCb_ * T_CbCa_ * hP_Ca);
      hp_Sb.normalize();
      frameB_->setLandmarkId(camIdB_, indexB, lmId);
      lmIdB = lmId;
      LOG(WARNING) << "add landmark to frame B here.";
    }

  } else {
    OKVIS_ASSERT_TRUE_DBG(Exception,lmIdB==0,"bug. Id in frame B already set.");

    // get projection into B
    Eigen::Vector2d kptB = projectionsIntoB_.row(indexA);
    Eigen::Vector2d keypointBMeasurement;
    frameB_->getKeypoint(camIdB_, indexB, keypointBMeasurement);

    Eigen::Vector2d err = kptB - keypointBMeasurement;
    double keypointBStdDev;
    frameB_->getKeypointSize(camIdB_, indexB, keypointBStdDev);
    keypointBStdDev = 0.8 * keypointBStdDev / 12.0;
    Eigen::Matrix2d U_tot = Eigen::Matrix2d::Identity() * keypointBStdDev
        * keypointBStdDev
        + projectionsIntoBUncertainties_.block<2, 2>(2 * indexA, 0);

    const double chi2 = err.transpose().eval() * U_tot.inverse() * err;

    if (chi2 > 4.0) {
      return;
    }

    // saturate allowed image uncertainty
    if (U_tot.norm() > 25.0 / (keypointBStdDev * keypointBStdDev * sqrt(2))) {
      numUncertainMatches_++;
      //return;
    }

    frameB_->setLandmarkId(camIdB_, indexB, lmIdA);
    lmIdB = lmIdA;
    okvis::MapPoint landmark;
    estimator_->getLandmark(lmIdA, landmark);

    // initialize in graph
    if (landmark.observations.find(
        okvis::KeypointIdentifier(mfIdB_, camIdB_, indexB))
        == landmark.observations.end()) {  // ensure no double observations...
      OKVIS_ASSERT_TRUE(Exception, estimator_->isLandmarkAdded(lmIdB),
                        "not added");
      estimator_->addObservation<camera_geometry_t>(lmIdB, mfIdB_, camIdB_,
                                                    indexB);
    }

  }
  numMatches_++;
}

} // namespace okvis