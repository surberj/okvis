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
 * @file VioKeyframeWindowMatchingAlgorithm.hpp
 * @brief Header file for the VioKeyframeWindowMatchingAlgorithm class.
 * @author Stefan Leutenegger
 * @author Andreas Forster
 */

#ifndef INCLUDE_OKVIS_MATCHER_HPP_
#define INCLUDE_OKVIS_MATCHER_HPP_

#include <memory>

#include <okvis/DenseMatcher.hpp>
#include <okvis/MatchingAlgorithm.hpp>
#include <okvis/FrameTypedefs.hpp>
#include <okvis/triangulation/ProbabilisticStereoTriangulator.hpp>
#include <okvis/MultiFrame.hpp>
#include <brisk/internal/hamming.h>

/// \brief okvis Main namespace of this package.
namespace okvis {

/**
 * \brief A MatchingAlgorithm implementation
 * \tparam CAMERA_GEOMETRY_T Camera geometry model. See also okvis::cameras::CameraBase.
 */
template<class CAMERA_GEOMETRY_T>
class Matcher : public okvis::MatchingAlgorithm {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  OKVIS_DEFINE_EXCEPTION(Exception, std::runtime_error)

  typedef CAMERA_GEOMETRY_T camera_geometry_t;

  enum MatchingTypes {
    Match3D2D = 1,  ///< Match 3D position of established landmarks to 2D keypoint position
    Match2D2D = 2   ///< Match 2D position of established landmarks to 2D keypoint position
  };

  /**
   * @brief Constructor.
   * @param matchingType        Matching type. See MatchingTypes enum.
   * @param distanceThreshold   Descriptor distance threshold.
   * @param usePoseUncertainty  Use the pose uncertainty for matching.
   */
  Matcher(int matchingType, float distanceThreshold,
                                     bool usePoseUncertainty = true);

  virtual ~Matcher();

  /**
   * @brief Set which frames to match.
   */
  void setFrames(std::shared_ptr<okvis::MultiFrame> currentFrame, std::shared_ptr<okvis::MultiFrame> referenceFrame);

  /// \brief What is the size of list A?
  virtual size_t sizeA() const;
  /// \brief What is the size of list B?
  virtual size_t sizeB() const;

  /// \brief A function that tells you how many times setMatching() will be called.
  /// \warning Currently not implemented to do anything.
  virtual void reserveMatches(size_t numMatches);

  /// \brief Get the distance threshold for which matches exceeding it will not be returned as matches.
  virtual float distanceThreshold() const;

  /// \brief At the end of the matching step, this function is called once
  ///        for each pair of matches discovered.
  virtual void setBestMatch(size_t indexA, size_t indexB, double distance);

private:
  /// \name Which frames to take
  /// \{
  uint64_t mfIdA_ = 0;
  uint64_t mfIdB_ = 0;
  size_t camIdA_ = 0;
  size_t camIdB_ = 0;

  std::shared_ptr<okvis::MultiFrame> frameA_;
  std::shared_ptr<okvis::MultiFrame> frameB_;

  /// Distances above this threshold will not be returned as matches.
  float distanceThreshold_;

  /// Stereo triangulator.
  okvis::triangulation::ProbabilisticStereoTriangulator<camera_geometry_t> probabilisticStereoTriangulator_;

  /// temporarily store all projections
  Eigen::Matrix<double, Eigen::Dynamic, 2> projectionsIntoB_;
  /// temporarily store all projection uncertainties
  Eigen::Matrix<double, Eigen::Dynamic, 2> projectionsIntoBUncertainties_;



} // namespace okvis

#endif /* INCLUDE_OKVIS_MATCHER_HPP_ */