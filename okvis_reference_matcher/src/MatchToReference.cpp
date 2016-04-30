#include <okvis/MatchToReference.hpp>

#include <brisk/brisk.h>

namespace okvis {

// Constructor
MatchToReference::MatchToReference(size_t numCameras)
	: isInitialized_(false),
      numCameras_(numCameras),
      briskDetectionOctaves_(0),
      briskDetectionThreshold_(50.0),
      briskDetectionAbsoluteThreshold_(800.0),
      briskDetectionMaximumKeypoints_(450),
      briskDescriptionRotationInvariance_(true),
      briskDescriptionScaleInvariance_(false),
      briskMatchingThreshold_(60.0),
      matcher_(
          std::unique_ptr<okvis::DenseMatcher>(new okvis::DenseMatcher(4))),
      keyframeInsertionOverlapThreshold_(0.6),
      keyframeInsertionMatchingRatioThreshold_(0.2) {
  // create mutexes for feature detectors and descriptor extractors
  for (size_t i = 0; i < numCameras_; ++i) {
    featureDetectorMutexes_.push_back(
        std::unique_ptr<std::mutex>(new std::mutex()));
  }
  initialiseBriskFeatureDetectors();
}

// Set Frame
void MatchToReference::setFrame(std::shared_ptr<okvis::MultiFrame> frame) {
	currentFrame_ = frame;
}

// Set Reference Frame from Reference Map for 2D2D matching
void MatchToReference::setNearestKeyframe(std::shared_ptr<okvis::MultiFrame> frame) {
	referenceFrame_ = frame;
}

// Detection and descriptor extraction on a per image basis.
bool MatchToReference::detectAndDescribe(size_t cameraIndex,
                                 const okvis::kinematics::Transformation& T_WC,
                                 const std::vector<cv::KeyPoint> * keypoints) {
  OKVIS_ASSERT_TRUE_DBG(Exception, cameraIndex < numCameras_, "Camera index exceeds number of cameras.");
  std::lock_guard<std::mutex> lock(*featureDetectorMutexes_[cameraIndex]);

  // check there are no keypoints here
  OKVIS_ASSERT_TRUE(Exception, keypoints == nullptr, "external keypoints currently not supported");

  currentFrame_->setDetector(cameraIndex, featureDetectors_[cameraIndex]);
  currentFrame_->setExtractor(cameraIndex, descriptorExtractors_[cameraIndex]);

  currentFrame_->detect(cameraIndex);

  // ExtractionDirection == gravity direction in camera frame
  Eigen::Vector3d g_in_W(0, 0, -1);
  Eigen::Vector3d extractionDirection = T_WC.inverse().C() * g_in_W;
  currentFrame_->describe(cameraIndex, extractionDirection);

  // set detector/extractor to nullpointer? TODO
  return true;
}

// HACK: Detection and descriptor extraction on a per image basis to get a "reference map" of 1 keyframe.
bool MatchToReference::detectAndDescribeKeyframe(size_t cameraIndex,
                                 const okvis::kinematics::Transformation& T_WC,
                                 const std::vector<cv::KeyPoint> * keypoints) {
  OKVIS_ASSERT_TRUE_DBG(Exception, cameraIndex < numCameras_, "Camera index exceeds number of cameras.");
  std::lock_guard<std::mutex> lock(*featureDetectorMutexes_[cameraIndex]);

  // check there are no keypoints here
  OKVIS_ASSERT_TRUE(Exception, keypoints == nullptr, "external keypoints currently not supported");

  referenceFrame_->setDetector(cameraIndex, featureDetectors_[cameraIndex]);
  referenceFrame_->setExtractor(cameraIndex, descriptorExtractors_[cameraIndex]);

  referenceFrame_->detect(cameraIndex);

  // ExtractionDirection == gravity direction in camera frame
  Eigen::Vector3d g_in_W(0, 0, -1);
  Eigen::Vector3d extractionDirection = T_WC.inverse().C() * g_in_W;
  referenceFrame_->describe(cameraIndex, extractionDirection);

  // set detector/extractor to nullpointer? TODO
  return true;
}

void MatchToReference::match2D2D(size_t cameraIndex) {
	LOG(WARNING) << "MatchToReference::match2D2D() not implemented!";
	// based on the prior pose and uncertainty get all possible landmark candidates
	// from the reference map


/*	// perform matching between all keypoints of the frame and the landmark candidates
  okvis::Matcher<okvis::cameras::PinholeCamera<okvis::cameras::EquidistantDistortion>> matchingAlgorithm(
                                       okvis::Matcher<okvis::cameras::PinholeCamera<okvis::cameras::EquidistantDistortion>>::Match2D2D,
                                       briskMatchingThreshold_,
                                       false);
  matchingAlgorithm.setFrames(currentFrame_, referenceFrame_);
  matcher_->match<MATCHING_ALGORITHM>(matchingAlgorithm);
*/


	// perform ransac PnP to filter outliers and get a pose measurement of the frame 
	// with respect to the global frame
}

void MatchToReference::solvePnPRansac() {
	LOG(WARNING) << "MatchToReference::solvePnPRansac() not implemented!";
	// perform ransac PnP to filter outliers and get a pose measurement of the frame 
	// with respect to the global frame
}

void MatchToReference::get_current_T_WC(okvis::kinematics::Transformation & T_WC) {
	LOG(WARNING) << "MatchToReference::get_current_T_WC() not implemented!";
}

// (re)instantiates feature detectors and descriptor extractors. Used after settings changed or at startup.
void MatchToReference::initialiseBriskFeatureDetectors() {
  for (auto it = featureDetectorMutexes_.begin();
      it != featureDetectorMutexes_.end(); ++it) {
    (*it)->lock();
  }
  featureDetectors_.clear();
  descriptorExtractors_.clear();
  for (size_t i = 0; i < numCameras_; ++i) {
    featureDetectors_.push_back(
        std::shared_ptr<cv::FeatureDetector>(
#ifdef __ARM_NEON__
            new cv::GridAdaptedFeatureDetector( 
            new cv::FastFeatureDetector(briskDetectionThreshold_),
                briskDetectionMaximumKeypoints_, 7, 4 ))); // from config file, except the 7x4...
#else
            new brisk::ScaleSpaceFeatureDetector<brisk::HarrisScoreCalculator>(
                briskDetectionThreshold_, briskDetectionOctaves_, 
                briskDetectionAbsoluteThreshold_,
                briskDetectionMaximumKeypoints_)));
#endif
    descriptorExtractors_.push_back(
        std::shared_ptr<cv::DescriptorExtractor>(
            new brisk::BriskDescriptorExtractor(
                briskDescriptionRotationInvariance_,
                briskDescriptionScaleInvariance_)));
  }
  for (auto it = featureDetectorMutexes_.begin();
      it != featureDetectorMutexes_.end(); ++it) {
    (*it)->unlock();
  }
}

} // namespace okvis