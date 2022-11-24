//
// Created by tianyi on 08.11.22.
//

#pragma once


#include <cstdio>      /* printf, NULL */
#include <cstdlib>     /* srand, rand */
#include <ctime>       /* time */
#include<opencv2/opencv.hpp>

#include "hyper/messages/visual.hpp"
#include "hyper/system/components/frontends/abstract.hpp"
//#include "hyper/system/components/frontends/visual/MonocularUtilizer.h"

namespace hyper
{

class Frame;

// THIS IS THE INITIALIZER FOR MONOCULAR SLAM. NOT USED IN THE STEREO OR RGBD CASE.
class Initializer
{
  typedef std::pair<int,int> Match;
  using Frame = hyper::Frame;

 public:

  // Fix the reference frame
//  Initializer(const Frame &ReferenceFrame, float sigma = 1.0, int iterations = 200);
  Initializer(cv::Mat K, float sigma, int iterations);

  // Computes in parallel a fundamental matrix and a homography
  // Selects a model and tries to recover the motion and the structure from motion
//  bool Initialize(const Frame &CurrentFrame, const std::vector<int> &vMatches12, cv::Mat &R21, cv::Mat &t21,
//      std::vector<cv::Point3f> &vP3D, std::vector<bool> &vbTriangulated);
  bool Initialize(
      std::vector<cv::Point2f> &Points1, std::vector<cv::Point2f> &Points2,
      cv::Mat &R21, cv::Mat &t21,
      std::vector<cv::Point3f> &vP3D, std::vector<bool> &vbTriangulated);


 private:

  void FindHomography(std::vector<bool> &vbMatchesInliers, float &score, cv::Mat &H21);
  void FindFundamental(std::vector<bool> &vbInliers, float &score, cv::Mat &F21);

  cv::Mat ComputeH21(const std::vector<cv::Point2f> &vP1, const std::vector<cv::Point2f> &vP2);
  cv::Mat ComputeF21(const std::vector<cv::Point2f> &vP1, const std::vector<cv::Point2f> &vP2);

  float CheckHomography(const cv::Mat &H21, const cv::Mat &H12, std::vector<bool> &vbMatchesInliers, float sigma);

  float CheckFundamental(const cv::Mat &F21, std::vector<bool> &vbMatchesInliers, float sigma);

  bool ReconstructF(std::vector<bool> &vbMatchesInliers, cv::Mat &F21, cv::Mat &K,
      cv::Mat &R21, cv::Mat &t21, std::vector<cv::Point3f> &vP3D, std::vector<bool> &vbTriangulated, float minParallax, int minTriangulated);

  bool ReconstructH(std::vector<bool> &vbMatchesInliers, cv::Mat &H21, cv::Mat &K,
      cv::Mat &R21, cv::Mat &t21, std::vector<cv::Point3f> &vP3D, std::vector<bool> &vbTriangulated, float minParallax, int minTriangulated);

  void Triangulate(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const cv::Mat &P1, const cv::Mat &P2, cv::Mat &x3D);

  void Normalize(const std::vector<cv::KeyPoint> &vKeys, std::vector<cv::Point2f> &vNormalizedPoints, cv::Mat &T);

  int CheckRT(const cv::Mat &R, const cv::Mat &t, const std::vector<cv::KeyPoint> &vKeys1, const std::vector<cv::KeyPoint> &vKeys2,
      const std::vector<Match> &vMatches12, std::vector<bool> &vbInliers,
      const cv::Mat &K, std::vector<cv::Point3f> &vP3D, float th2, std::vector<bool> &vbGood, float &parallax);

  void DecomposeE(const cv::Mat &E, cv::Mat &R1, cv::Mat &R2, cv::Mat &t);


  // Keypoints from Reference Frame (Frame 1)
  std::vector<cv::KeyPoint> mvKeys1;

  // Keypoints from Current Frame (Frame 2)
  std::vector<cv::KeyPoint> mvKeys2;

  // Current Matches from Reference to Current
  std::vector<Match> mvMatches12;
//  std::vector<bool> mvbMatched1;

  // Calibration
  cv::Mat mK;

  // Standard Deviation and Variance
  float mSigma, mSigma2;

  // Ransac max iterations
  int mMaxIterations;

  // Ransac sets
  std::vector<std::vector<size_t> > mvSets;

};

}



