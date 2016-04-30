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
#define CERES_FOUND true

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

void convertIntrinsics(Eigen::VectorXd intrinsics, cv::Mat & cameraMatrix, cv::Mat & distCoeffs) {
  cameraMatrix.at<double>(0,0) = intrinsics(0);
  cameraMatrix.at<double>(1,1) = intrinsics(1);
  cameraMatrix.at<double>(0,2) = intrinsics(2);
  cameraMatrix.at<double>(1,2) = intrinsics(3);
  distCoeffs.at<double>(0) = intrinsics(4);
  distCoeffs.at<double>(1) = intrinsics(5);
  distCoeffs.at<double>(2) = intrinsics(6);
  distCoeffs.at<double>(3) = intrinsics(7);
} 

bool CheckCoherentRotation(cv::Mat_<double>& R) {
  if (fabsf(determinant(R)) - 1.0 > 1e-07) {
    LOG(WARNING) << "det(R) != +-1.0, this is not a rotation matrix";
    return false;
  }
  return true;
}

void projectionsFromEssential(cv::Mat E, cv::Matx34d & P1, cv::Matx34d & P2) {
  // decompose E to P' , HZ (9.19)
  cv::SVD svd(E,cv::SVD::MODIFY_A);
  cv::Mat svd_u = svd.u;
  cv::Mat svd_vt = svd.vt;
  cv::Mat svd_w = svd.w;
  cv::Matx33d W(0,-1,0,1,0,0,0,0,1);//HZ 9.13
  cv::Mat_<double> R = svd_u * cv::Mat(W) * svd_vt; //HZ 9.19
  cv::Mat_<double> t = svd_u.col(2); //u3
  if (!CheckCoherentRotation (R)) {
    std::cout<<"resulting rotation is not coherent\n";
    P1 = 0;
    P2 = 0;
  } else {
    P1 = cv::Matx34d(1.0,0.0,0.0,0.0,
                0.0,1.0,0.0,0.0,
                0.0,0.0,1.0,0.0);
    P2 = cv::Matx34d(R(0,0),R(0,1),R(0,2),t(0),
                R(1,0),R(1,1),R(1,2),t(1),
                R(2,0),R(2,1),R(2,2),t(2));
  }
}

TEST(ReferenceMatcher, OpencvMatcher)
{
  OKVIS_DEFINE_EXCEPTION(Exception, std::runtime_error);

// image callback setup
    // create camera
    // for frame
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
            "/media/julian/45068791-e8b2-46ee-b57c-b8ba15e0ddfb/20160324_unispital/003/cam0/data/1458825891243364500.png",
            cv::IMREAD_GRAYSCALE); // image N=11

    // load images for reference map
    cv::Mat refimage1 = cv::imread(
            "/media/julian/45068791-e8b2-46ee-b57c-b8ba15e0ddfb/20160324_unispital/003/cam0/data/1458825890743364500.png",
            cv::IMREAD_GRAYSCALE); // image N=1
    cv::Mat refimage2 = cv::imread(
            "/media/julian/45068791-e8b2-46ee-b57c-b8ba15e0ddfb/20160324_unispital/003/cam0/data/1458825890993364500.png",
            cv::IMREAD_GRAYSCALE); // image N=6

    // view image
  //  cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE );
  //  cv::imshow("Display Image", image);
  //  cv::waitKey(0);

    // create okvis::Frame container which holds image keypoints and descriptors
    okvis::Frame frame(image, cameras.at(0), detector, extractor);
    okvis::Frame refframe1(refimage1, cameras.at(0), detector, extractor);
    okvis::Frame refframe2(refimage2, cameras.at(0), detector, extractor);

    // detect and describe frame
    // detect corners
    frame.detect();
    refframe1.detect();
    refframe2.detect();
    // describe BRISK features
    frame.describe();
    refframe1.describe();
    refframe2.describe();


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
  cv::Mat refdescriptors1 = refframe1.descriptors();
  cv::Mat refdescriptors2 = refframe2.descriptors();

  std::vector<cv::DMatch> refmatches;
  matcher2D2D.match(refdescriptors1, refdescriptors2, refmatches);

// check quality
  //-- Quick calculation of max and min distances between keypoints
  double max_dist = 0; double min_dist = 100;
  for( int i = 0; i < refdescriptors1.rows; i++ ) {
    double dist = refmatches[i].distance;
    if( dist < min_dist ) min_dist = dist;
    if( dist > max_dist ) max_dist = dist;
  }

  printf("-- Max dist : %f \n", max_dist );
  printf("-- Min dist : %f \n", min_dist );

  std::vector<cv::DMatch> good_refmatches;

  for( int i = 0; i < refdescriptors1.rows; i++ ) {
    if( refmatches[i].distance <= std::max(5*min_dist, 0.02) ) {
      good_refmatches.push_back( refmatches[i]); 
    }
  }

// inspect and check
  // get all keypoints
  std::vector<cv::KeyPoint> refkeypoints1;
  for (size_t i=0;i<refframe1.numKeypoints();i++) {
    cv::KeyPoint kp;
    refframe1.getCvKeypoint(i, kp);
    refkeypoints1.push_back(kp);
  }
    
  std::vector<cv::KeyPoint> refkeypoints2;
  for (size_t i=0;i<refframe2.numKeypoints();i++) {
    cv::KeyPoint kp;
    refframe2.getCvKeypoint(i, kp);
    refkeypoints2.push_back(kp);
  }

  cv::Mat all_refmatches, all_good_refmatches;
  cv::drawMatches( refimage1, refkeypoints1, refimage2, refkeypoints2,
                       refmatches, all_refmatches, cv::Scalar::all(-1), cv::Scalar::all(-1),
                       std::vector<char>(),cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
  cv::imshow( "BRISK All Matches for Reference Map", all_refmatches );
  cv::waitKey(0);

  cv::drawMatches( refimage1, refkeypoints1, refimage2, refkeypoints2,
                       good_refmatches, all_good_refmatches, cv::Scalar::all(-1), cv::Scalar::all(-1),
                       std::vector<char>(),cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
  cv::imshow( "BRISK GOOD Matches for Reference Map", all_good_refmatches );
  cv::waitKey(0);

// find fundamental matrix
  cv::Mat F;
  std::vector<cv::KeyPoint> kp1, kp2;
  std::vector<cv::Point2f> k1, k2;
  // loop through DMatch and assign points to k1 and k2
  for(std::vector<cv::DMatch>::iterator it = refmatches.begin(); it != refmatches.end(); ++it) {
    cv::KeyPoint kp;
    refframe1.getCvKeypoint(it->queryIdx, kp);
    kp1.push_back(kp);
    refframe2.getCvKeypoint(it->trainIdx, kp);
    kp2.push_back(kp);
  }
  cv::KeyPoint::convert(kp1, k1);
  cv::KeyPoint::convert(kp2, k2);
  F = cv::findFundamentalMat(k1, k2, CV_FM_RANSAC, 3.0, 0.99, cv::noArray());

// create 3D points out of matches
  // get projection matrices from Fundamental (not unique!)

  // 1) convert intrinsics from Eigen::VectorXd to cv::Mat cameraMatrix and cv::Mat distCoeffs
  Eigen::VectorXd intrinsics;
  refframe1.geometry()->getIntrinsics(intrinsics);
  LOG(WARNING) << "intrinsics: ";
  LOG(WARNING) << intrinsics;
  cv::Mat cameraMatrix, distCoeffs;
  cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
  distCoeffs = cv::Mat::zeros(8, 1, CV_64F);
  convertIntrinsics(intrinsics, cameraMatrix, distCoeffs);
  // Set prior of second camera wrt first camera
  //cv::Matx33d R(1.0,0.0,0.0,
  //              0.0,1.0,0.0,
  //              0.0,0.0,1.0);
  //cv::Matx31d T(2.0,0.0,0.0);

  // 2) Get essential matrix
  cv::Mat E = cameraMatrix.t() * F * cameraMatrix;

  // 3) Get Projection matrices from Essential
  cv::Matx34d P1, P2;
  projectionsFromEssential(E,P1,P2);
  //cv::Mat R1, R2, Q;
  //stereoRectify(cameraMatrix, distCoeffs, cameraMatrix, distCoeffs, 
  //              refimage1.size(), R, T, R1, R2, P1, P2, Q);
  LOG(WARNING) << "P1: ";
  LOG(WARNING) << P1;
  LOG(WARNING) << "P2: ";
  LOG(WARNING) << P2;
  // 4) Triangulate Points
  cv::Mat points4D, points3D;
  cv::triangulatePoints(P1, P2, k1, k2, points4D);
  //cv::convertPointsFromHomogeneous(points4D, points3D);


// setup PnP ransac (for pose estimation of frame wrt referenceframe)
  cv::Mat tvec;
  cv::Mat rvec(1,3,CV_64F);
  cv::Mat Rot(3,3,CV_64F);

/*  cv::solvePnPRansac(points3D, kp1,
                        cameraMatrix, distCoeffs, rvec, tvec,
                        false, 100, 8.0, 0.99, cv::noArray()); */

  okvis::kinematics::Transformation T_WC_estimated/*(translation, quaternion)*/;
  cv::Mat T_WC_estimated_cv(T_WC_estimated.T().rows(), T_WC_estimated.T().cols(), CV_64F, T_WC_estimated.T().data());
  //cv::Affine3 Affe(T_WC_estimated_cv);
  cv::Rodrigues(rvec, Rot);
// somehow the Rodrigues() is not working, debug it!
  //Eigen::Map<Eigen::Matrix3d> rotation(Rot.ptr<double>());
  //Eigen::Quaterniond quaternion(rotation);
  //Eigen::Map<Eigen::Vector3d> translation(tvec.ptr<double>());
// somehow the conversion from cv::Mat to Eigen::Matrix is not working, debug it!


  // save pose as a measurement
  LOG(WARNING) << "T_WC_estimated = ";
  LOG(WARNING) << T_WC_estimated.T();

  OKVIS_ASSERT_TRUE(Exception, true, "OpencvMatcher failed.");

}

