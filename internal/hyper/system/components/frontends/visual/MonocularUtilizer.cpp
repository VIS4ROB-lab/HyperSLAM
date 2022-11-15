#include <numeric>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>
#include <opencv2/features2d.hpp>


#include "hyper/composite.hpp"
#include "hyper/sensors/camera.hpp"
#include "hyper/system/components/backend.hpp"
// #include "hyper/system/components/frontends/visual/tracking.hpp"
#include "hyper/system/components/frontends/visual/Initializer.hpp"
#include "hyper/system/components/frontends/visual/MonocularUtilizer.hpp"
#include "hyper/yaml/yaml.hpp"

namespace hyper {

VisualTracksInit::VisualTracksInit(const Stamp& initStamp, const Camera& initCamera, const Stamp& currStamp, const Camera& currCamera):AbstractMessage{initStamp, initCamera},
initial_track{std::make_unique<VisualTracks>(initStamp, initCamera)},current_track{std::make_unique<VisualTracks>(currStamp, currCamera)},
initial_pose{}, current_pose{}{}


MapPoint::MapPoint(const cv::Mat &Pos, Frame* pFrame, const int &idxF){
  Pos.copyTo(mWorldPos);
  cv::Mat Ow = pFrame->GetCameraCenter();
  mNormalVector = mWorldPos - Ow;
  mNormalVector = mNormalVector/cv::norm(mNormalVector);

  cv::Mat PC = Pos - Ow;
  const float dist = cv::norm(PC);
  const int level = pFrame->mvKeys[idxF].octave;

  pFrame->mDescriptors.row(idxF).copyTo(mDescriptor);
  mnId=nNextId++;

}

void MapPoint::AddObservation(Frame* pF,size_t idx){
  if(mObservations.count(pF))
    return;
  mObservations[pF]=idx;

  if(pF->mvuRight[idx]>=0)
    nObs+=2;
  else
    nObs++;
}

void MapPoint::SetWorldPos(const cv::Mat &Pos)
{
  Pos.copyTo(mWorldPos);
}

long unsigned int Frame::nNextId=0;
bool Frame::mbInitialComputations=true;
Frame::Frame(){};
Frame::Frame(const Frame &frame):
        mTimeStamp(frame.mTimeStamp), mK(frame.mK.clone()), mDistCoef(frame.mDistCoef.clone()),
        mbf(frame.mbf), mb(frame.mb), mThDepth(frame.mThDepth), N(frame.N), mvKeys(frame.mvKeys),
                                   mvuRight(frame.mvuRight),
        mvDepth(frame.mvDepth),
        mDescriptors(frame.mDescriptors.clone()),
        mvpMapPoints(frame.mvpMapPoints), mvbOutlier(frame.mvbOutlier), mnId(frame.mnId)
  {
  for(int i=0;i<FRAME_GRID_COLS;i++)
    for(int j=0; j<FRAME_GRID_ROWS; j++)
      mGrid[i][j]=frame.mGrid[i][j];

  if(!frame.mTcw.empty())
    SetPose(frame.mTcw);
}

Frame::Frame(const double& stamp, const cv::Mat& im, const cv::Mat& K)
    :mTimeStamp{stamp}, I0{im}, mK{K}
{
  // Frame ID
  mnId=nNextId++;
  // ORB extraction
  Frame::ExtractORB(I0);
  N = mvKeys.size();
  if(mvKeys.empty())
    return;

  // This is done only for the first Frame (or after a change in the calibration)
  if(mbInitialComputations)
  {

    mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);
    mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);

    fx = K.at<float>(0,0);
    fy = K.at<float>(1,1);
    cx = K.at<float>(0,2);
    cy = K.at<float>(1,2);
    invfx = 1.0f/fx;
    invfy = 1.0f/fy;

    mbInitialComputations=false;
  }

  mb = mbf/fx;

}

void Frame::ExtractORB(const cv::Mat& im) {
  // use ORB detector and descriptor
  cv::Mat I0 = im;
  cv::Ptr<cv::ORB> descriptor = cv::ORB::create();
  descriptor->compute (I0, mvKeys, mDescriptors);

}

void Frame::AddMapPoint(MapPoint *pMP, const size_t &idx){
  mvpMapPoints[idx]=pMP;
}

void Frame::SetPose(cv::Mat Tcw)
{
  mTcw = Tcw.clone();
  UpdatePoseMatrices();
}

void Frame::UpdatePoseMatrices()
{
  mRcw = mTcw.rowRange(0,3).colRange(0,3);
  mRwc = mRcw.t();
  mtcw = mTcw.rowRange(0,3).col(3);
  mOw = -mRcw.t()*mtcw;
}

float Frame::ComputeSceneMedianDepth(const int q)
{
  std::vector<MapPoint*> vpMapPoints;
  cv::Mat Tcw_;
  {
    vpMapPoints = mvpMapPoints;
    Tcw_ = mTcw.clone();
  }

  std::vector<float> vDepths;
  vDepths.reserve(N);
  cv::Mat Rcw2 = Tcw_.row(2).colRange(0,3);
  Rcw2 = Rcw2.t();
  float zcw = Tcw_.at<float>(2,3);
  for(int i=0; i<N; i++)
  {
    if(mvpMapPoints[i])
    {
      MapPoint* pMP = mvpMapPoints[i];
      cv::Mat x3Dw = pMP->GetWorldPos();
      float z = Rcw2.dot(x3Dw)+zcw;
      vDepths.push_back(z);
    }
  }

  sort(vDepths.begin(),vDepths.end());

  return vDepths[(vDepths.size()-1)/q];
}

cv::Mat MapPoint::GetWorldPos()
{
  return mWorldPos.clone();
}

cv::Mat MapPoint::GetNormal()
{
  return mNormalVector.clone();
}
}