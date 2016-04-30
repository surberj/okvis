/**
 * @file MatchToReference.hpp
 * @brief Header file for the MatchToReference class.
 * @author Julian Surber
 */

#ifndef INCLUDE_OKVIS_MATCHTOREFERENCE_HPP_
#define INCLUDE_OKVIS_MATCHTOREFERENCE_HPP_

#include <mutex>
#include <iostream>
#include <fstream>
#include <map>
#include <vector>
#include <string>

#include <glog/logging.h>
#include <okvis/Estimator.hpp>
#include <okvis/VioFrontendInterface.hpp>
#include <okvis/Keyframe.hpp>
#include <okvis/KeyframeDatabase.hpp>
#include <okvis/FrameTypedefs.hpp>
#include <okvis/DenseMatcher.hpp>

namespace okvis {

class MatchToReference /*: public VioFrontendInterface */ {
public:
  OKVIS_DEFINE_EXCEPTION(Exception, std::runtime_error)

	
	/**
	 * @brief      Constructor of the MatchToReference class
	 */
	MatchToReference(size_t numCameras);

	/**
	 * @brief      Destructor of the MatchToReference class
	 */
	virtual ~MatchToReference() {}

	void setFrame(std::shared_ptr<okvis::MultiFrame> frame);

  // hack
	void setNearestKeyframe(std::shared_ptr<okvis::MultiFrame> frame);

	void match2D2D(size_t cameraIndex);

	void solvePnPRansac();

	void get_current_T_WC(okvis::kinematics::Transformation & T_WC);

	virtual bool detectAndDescribe(size_t cameraIndex,
                                 const okvis::kinematics::Transformation& T_WC /* needed for gravityalignment */,
                                 const std::vector<cv::KeyPoint> * keypoints);

  // hack
  virtual bool detectAndDescribeKeyframe(size_t cameraIndex,
                                 const okvis::kinematics::Transformation& T_WC /* needed for gravityalignment */,
                                 const std::vector<cv::KeyPoint> * keypoints);
	
private:
  /**
   * @brief   feature detectors with the current settings.
   *          The vector contains one for each camera to ensure that there are no problems with parallel detection.
   * @warning Lock with featureDetectorMutexes_[cameraIndex] when using the detector.
   */
  std::vector<std::shared_ptr<const cv::FeatureDetector> > featureDetectors_;
  /**
   * @brief   feature descriptors with the current settings.
   *          The vector contains one for each camera to ensure that there are no problems with parallel detection.
   * @warning Lock with featureDetectorMutexes_[cameraIndex] when using the descriptor.
   */
  std::vector<std::shared_ptr<const cv::DescriptorExtractor> > descriptorExtractors_;
  /// Mutexes for feature detectors and descriptors.
  std::vector<std::unique_ptr<std::mutex> > featureDetectorMutexes_;

  bool isInitialized_;        ///< Is the pose initialised?
  const size_t numCameras_;   ///< Number of cameras in the configuration.

  /// @name BRISK detection parameters
  /// @{

  size_t briskDetectionOctaves_;            ///< The set number of brisk octaves.
  double briskDetectionThreshold_;          ///< The set BRISK detection threshold.
  double briskDetectionAbsoluteThreshold_;  ///< The set BRISK absolute detection threshold.
  size_t briskDetectionMaximumKeypoints_;   ///< The set maximum number of keypoints.

  /// @}
  /// @name BRISK descriptor extractor parameters
  /// @{

  bool briskDescriptionRotationInvariance_; ///< The set rotation invariance setting.
  bool briskDescriptionScaleInvariance_;    ///< The set scale invariance setting.

  ///@}
  /// @name BRISK matching parameters
  ///@{

  double briskMatchingThreshold_; ///< The set BRISK matching threshold.

  ///@}

  std::unique_ptr<okvis::DenseMatcher> matcher_; ///< Matcher object.

  /**
   * @brief If the hull-area around all matched keypoints of the current frame (with existing landmarks)
   *        divided by the hull-area around all keypoints in the current frame is lower than
   *        this threshold it should be a new keyframe.
   * @see   doWeNeedANewKeyframe()
   */
  float keyframeInsertionOverlapThreshold_;  //0.6
  /**
   * @brief If the number of matched keypoints of the current frame with an older frame
   *        divided by the amount of points inside the convex hull around all keypoints
   *        is lower than the threshold it should be a keyframe.
   * @see   doWeNeedANewKeyframe()
   */
  float keyframeInsertionMatchingRatioThreshold_;  //0.2

  /// (re)instantiates feature detectors and descriptor extractors. Used after settings changed or at startup.
  void initialiseBriskFeatureDetectors();


	okvis::KeyframeDatabase* databasePtr_;
	std::shared_ptr<okvis::MultiFrame> currentFrame_;
	std::shared_ptr<okvis::MultiFrame> referenceFrame_; // nearest reference Keyframe for 2D2D matching
	cv::Mat descriptorsInFrame_; // TODO: not really needed, could be replaced by currentFrame_->
	cv::Mat descriptorsFromMap_;
	std::vector<cv::DMatch> matches;
	
};

}

#endif /* INCLUDE_OKVIS_MATCHTOREFERENCE_HPP_ */