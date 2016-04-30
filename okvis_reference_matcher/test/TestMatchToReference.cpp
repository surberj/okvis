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
 *  Created in: 2016
 *      Author: Julian Surber (surberj@student.ethz.ch)
 *********************************************************************************/

#include <iostream>
#include <stdio.h>
#include <gtest/gtest.h>
#include <okvis/MatchToReference.hpp>
#include <opencv2/opencv.hpp>
#include <brisk/brisk.h>
#include <okvis/IdProvider.hpp>
#include <okvis/MultiFrame.hpp>
#include "okvis/cameras/PinholeCamera.hpp"
#include "okvis/cameras/NoDistortion.hpp"
#include "okvis/cameras/RadialTangentialDistortion.hpp"
#include "okvis/cameras/EquidistantDistortion.hpp"
#include "okvis/Frame.hpp"

TEST(ReferenceMatcher, MatchToReference)
{
  OKVIS_DEFINE_EXCEPTION(Exception, std::runtime_error);

// image callback setup
    // create camera
    // for frame
    std::vector<std::shared_ptr<okvis::cameras::CameraBase> > cameras;
    cameras.push_back(
        okvis::cameras::PinholeCamera<okvis::cameras::EquidistantDistortion>::createUnispitalObject());
    // and for multiframe
    std::shared_ptr<const okvis::cameras::CameraBase> cameraGeometry0(
        okvis::cameras::PinholeCamera<okvis::cameras::EquidistantDistortion>::createTestObject());
    std::shared_ptr<const okvis::kinematics::Transformation> T_SC_init(
        new okvis::kinematics::Transformation());
    std::shared_ptr<okvis::cameras::NCameraSystem> cameraSystem(
        new okvis::cameras::NCameraSystem);
    cameraSystem->addCamera(T_SC_init, cameraGeometry0,
                            okvis::cameras::NCameraSystem::DistortionType::Equidistant);

    // create cv detector
    std::shared_ptr<cv::FeatureDetector> detector(
          new brisk::ScaleSpaceFeatureDetector<brisk::HarrisScoreCalculator>(
              34, 2, 800, 450));

    // create cv extractor
    std::shared_ptr<cv::DescriptorExtractor> extractor(
          new cv::BriskDescriptorExtractor(true, false));

// image callback
    // load image
    cv::Mat image = cv::imread(
            "/media/julian/45068791-e8b2-46ee-b57c-b8ba15e0ddfb/20160324_unispital/003/cam0/data/1458825890743364500.png",
            cv::IMREAD_GRAYSCALE);

    cv::Mat image2 = cv::imread(
            "/media/julian/45068791-e8b2-46ee-b57c-b8ba15e0ddfb/20160324_unispital/003/cam0/data/1458825890743364500.png",
            cv::IMREAD_GRAYSCALE);

    // view image
  //  cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE );
  //  cv::imshow("Display Image", image);
  //  cv::waitKey(0);

    // create okvis::Multiframe
    size_t cameraIdx = 0;
    okvis::Time t = okvis::Time::now();
    std::shared_ptr<okvis::MultiFrame> mf(new okvis::MultiFrame);
    mf->setId(okvis::IdProvider::instance().newId());
    mf->setTimestamp(t);
    mf->resetCameraSystemAndFrames(*cameraSystem);
    mf->setImage(cameraIdx,image);

    std::shared_ptr<okvis::MultiFrame> mf2(new okvis::MultiFrame);
    mf2->setId(okvis::IdProvider::instance().newId());
    mf2->setTimestamp(t);
    mf2->resetCameraSystemAndFrames(*cameraSystem);
    mf2->setImage(cameraIdx,image2);


    // view descriptor
  //  std::vector<uint64_t>* landmarkIds = frame.getLandmarkIds();
  //  uint64_t landmarkId = landmarksIds->at(0);
  //  const unsigned char * descriptor = frame.keypointDescriptor(landmarkId);
  //  double keypointSize;
  //  frame.getKeypointSize()
  //  LOG(WARNING) << "descriptor: ";
  //  for (size_t i=0;i<frame.numKeypoints();i++)
  //    LOG(WARNING) << int(frame.keypointDescriptor(frame.getLandmarkIds()->at(0))[i]);


// 2D3D matching setup
  // create matcher object
  size_t numCameras = 1;
  uint64_t keyframeID = 0;
  okvis::MatchToReference matcher(numCameras);

// 2D2D matching
  // prior
  okvis::kinematics::Transformation T_WS_prior;

  // based on prior get nearest keyframe from reference map
  keyframeID = 0;
  matcher.setNearestKeyframe(mf2);

  // detect and describe
  okvis::kinematics::Transformation T_SC = *mf->T_SC(cameraIdx);
  okvis::kinematics::Transformation T_WC = T_WS_prior*T_SC;
  matcher.setFrame(mf);
  matcher.detectAndDescribe(cameraIdx, T_WC, nullptr);

  // hack, as reference map is not available yet, add image2 as only referenceFrame
  matcher.setNearestKeyframe(mf2);
  matcher.detectAndDescribeKeyframe(cameraIdx, T_WC, nullptr);
  
  // match 2D keypoints to 3D landmarks --> get correspondences
  matcher.match2D2D(cameraIdx);

  // run P3P ransac --> get pose measurement
  matcher.solvePnPRansac();

  // save pose as a measurement
  okvis::kinematics::Transformation T_WC_estimated;
  matcher.get_current_T_WC(T_WC_estimated);
  LOG(WARNING) << "T_WC_estimated = ";
  LOG(WARNING) << T_WC_estimated.T();

  OKVIS_ASSERT_TRUE(Exception, false, "MatchToReference not yet finished");

}

