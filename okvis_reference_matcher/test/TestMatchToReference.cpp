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
 *  Created on: Mar 31, 2015
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *********************************************************************************/

#include <iostream>
#include <stdio.h>
#include <gtest/gtest.h>
#include <okvis/MatchToReference.hpp>
#include <opencv2/opencv.hpp>
#include <brisk/brisk.h>
#include "okvis/cameras/PinholeCamera.hpp"
#include "okvis/cameras/NoDistortion.hpp"
#include "okvis/cameras/RadialTangentialDistortion.hpp"
#include "okvis/cameras/EquidistantDistortion.hpp"
#include "okvis/Frame.hpp"

TEST(MatchToReference, unispital)
{
  OKVIS_DEFINE_EXCEPTION(Exception, std::runtime_error);

// image callback setup
      // create camera
    std::vector<std::shared_ptr<okvis::cameras::CameraBase> > cameras;
    cameras.push_back(
        okvis::cameras::PinholeCamera<okvis::cameras::EquidistantDistortion>::createUnispitalObject());

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

    // create okvis::Frame container which holds image keypoints and descriptors
    okvis::Frame frame(image, cameras.at(0), detector, extractor);
    okvis::Frame frame2(image2, cameras.at(0), detector, extractor);

    // detect corners
    frame.detect();
    frame2.detect();

    // describe BRISK features
    frame.describe();
    frame2.describe();

    // view descriptor
  //  std::vector<uint64_t>* landmarkIds = frame.getLandmarkIds();
  //  uint64_t landmarkId = landmarksIds->at(0);
  //  const unsigned char * descriptor = frame.keypointDescriptor(landmarkId);
  //  double keypointSize;
  //  frame.getKeypointSize()
  //  LOG(WARNING) << "descriptor: ";
  //  for (size_t i=0;i<frame.numKeypoints();i++)
  //    LOG(WARNING) << int(frame.keypointDescriptor(frame.getLandmarkIds()->at(0))[i]);


// 2D2D matching setup
  // create 2D2D matcher ()
  cv::FlannBasedMatcher matcher2D2D(new cv::flann::LshIndexParams(20,10,2));

// 2D2D matching
  // get descriptors
  cv::Mat descriptors = frame.descriptors();
  cv::Mat descriptors2 = frame2.descriptors();

  std::vector<cv::DMatch> matches;
  matcher2D2D.match(descriptors, descriptors2, matches);





// inspect and check
  // get all keypoints
  std::vector<cv::KeyPoint> keypoints;
  for (size_t i=0;i<frame.numKeypoints();i++) {
    cv::KeyPoint kp;
    frame.getCvKeypoint(i, kp);
    keypoints.push_back(kp);
  }
    
  std::vector<cv::KeyPoint> keypoints2;
  for (size_t i=0;i<frame2.numKeypoints();i++) {
    cv::KeyPoint kp;
    frame2.getCvKeypoint(i, kp);
    keypoints2.push_back(kp);
  }

  cv::Mat all_matches;
  cv::drawMatches( image, keypoints, image2, keypoints2,
                       matches, all_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
                       std::vector<char>(),cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
  cv::imshow( "BRISK All Matches", all_matches );
  cv::waitKey(0);



// 2D3D matching setup
  // create matcher object
  okvis::MatchToReference matcher;

// 2D3D matching
  // based on prior get landmark candidates from 3D map


  // match 2D keypoints to 3D landmarks --> get correspondences
  matcher.match();

  // run P3P ransac --> get pose measurement
  matcher.runP3P();

  // save pose as a measurement
  okvis::kinematics::Transformation T_WC_estimated;
  matcher.get_current_T_WC(T_WC_estimated);
  LOG(WARNING) << "T_WC_estimated = ";
  LOG(WARNING) << T_WC_estimated.T();

  OKVIS_ASSERT_TRUE(Exception, false, "MatchToReference not yet finished");

/*  // instantiate all possible versions of test cameras
  std::vector<std::shared_ptr<okvis::cameras::CameraBase> > cameras;
  cameras.push_back(
      okvis::cameras::PinholeCamera<okvis::cameras::NoDistortion>::createTestObject());
  cameras.push_back(
      okvis::cameras::PinholeCamera<okvis::cameras::RadialTangentialDistortion>::createTestObject());
  cameras.push_back(
      okvis::cameras::PinholeCamera<okvis::cameras::EquidistantDistortion>::createTestObject());

  for (size_t c = 0; c < cameras.size(); ++c) {

#ifdef __ARM_NEON__
   std::shared_ptr<cv::FeatureDetector> detector(
        new brisk::BriskFeatureDetector(34, 2));
#else
   std::shared_ptr<cv::FeatureDetector> detector(
        new brisk::ScaleSpaceFeatureDetector<brisk::HarrisScoreCalculator>(
            34, 2, 800, 450));
#endif
 
    std::shared_ptr<cv::DescriptorExtractor> extractor(
        new cv::BriskDescriptorExtractor(true, false));

    // create a stupid random image
    Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic> eigenImage(752,480);
    eigenImage.setRandom();
    cv::Mat image(480, 752, CV_8UC1, eigenImage.data());
    okvis::Frame frame(image, cameras.at(c), detector, extractor);

    // run
    frame.detect();
    frame.describe();
  }*/
}

