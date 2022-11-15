//
// Created by tianyi on 08.11.22.
//



#pragma once

#include <map>

#include "hyper/messages/visual.hpp"
//#include "hyper/system/components/frontends/abstract.hpp"
#include "hyper/system/components/frontends/visual/Initializer.hpp"
#include "hyper/messages/visual.hpp"

namespace hyper {

/// Visual Tracker for init 2 images
class VisualTracksInit
    : public  AbstractMessage{
 public:
  using Pose = Eigen::Map<SE3<Scalar>>;
//  using Message = std::tuple<std::unique_ptr<VisualTracks>, Pose>;
//  using Messages = std::map<int, Message>;
//
  std::unique_ptr<VisualTracks> initial_track;
  std::unique_ptr<VisualTracks> current_track;
  std::unique_ptr<Pose> initial_pose;
  std::unique_ptr<Pose> current_pose;

  VisualTracksInit(const Stamp& initStamp, const Camera& initCamera, const Stamp& currStamp, const Camera& currCamera);
//  void addMessage(const int idx, const Stamp& stamp, const Camera& camera, const Pose& pose);
//  Messages messages;


  // Definitions.
//  using Image = cv_bridge::CvImageConstPtr;
//  using Points = std::vector<cv::Point2f>;
//  using Entry = std::tuple<Image, Points>;
//  using Tracks = std::map<const Camera*, Entry>;
//  using Identifiers = std::vector<Identifier>;
//  using Positions = std::vector<Position<Scalar>>;
//  using Lengths = std::vector<std::size_t>;

//  /// Constructor from time and sensor.
//  /// \param stamp Stamp.
//  /// \param camera Camera.
//  VisualTracks(const Stamp& stamp, const Camera& camera);
//
//  /// Sensor accessor.
//  /// \return Sensor.
//  [[nodiscard]] auto sensor() const -> const Camera& final;
//
//  /// Sets the associated sensor.
//  /// \param camera Sensor to set.
//  auto setSensor(const Camera& camera) -> void;
//
//  /// Adds a new track.
//  /// \param camera Associated camera.
//  auto addTrack(const Camera& camera) -> Entry&;
//
//  /// Track accessor.
//  /// \param camera Associated camera.
//  /// \return Entry containing the associated image and tracked points.
//  [[nodiscard]] auto getTrack(const Camera& camera) const -> const Entry&;
//
//  /// Track modifier.
//  /// \param camera Associated camera.
//  /// \return Entry containing the associated image and tracked points.
//  auto getTrack(const Camera& camera) -> Entry&;

//  Tracks track;           ///< Tracks.
//  Identifiers identifier; ///< Track identifiers.
//  Positions position;     ///< Track positions (optional).
//  Lengths length;         ///< Track lengths.
//  using Message = std::tuple<VisualTracks, Pose>;
//  using Messages = std::map<int, Message>;


};


class Frame;
class MapPoint;

class MapPoint{
 public:
  explicit MapPoint(const cv::Mat &Pos, Frame* pFrame, const int &idxF);
  void SetWorldPos(const cv::Mat &Pos);
  cv::Mat GetWorldPos();
  cv::Mat GetNormal();
  void UpdateNormalAndDepth();
//  std::map<Frame*, size_t> GetObservations();
//  int Observations();

  void AddObservation(Frame* pF,size_t idx);
 public:
  long unsigned int mnId;
  static long unsigned int nNextId;
  long int mnFirstKFid;
  long int mnFirstFrame;
  int nObs;

  // Variables used by the tracking
  float mTrackProjX;
  float mTrackProjY;
  float mTrackProjXR;
  bool mbTrackInView;
  int mnTrackScaleLevel;
  float mTrackViewCos;
  long unsigned int mnTrackReferenceForFrame;
  long unsigned int mnLastFrameSeen;

  // Variables used by local mapping
  long unsigned int mnBALocalForKF;
  long unsigned int mnFuseCandidateForKF;

  // Variables used by loop closing
  long unsigned int mnLoopPointForKF;
  long unsigned int mnCorrectedByKF;
  long unsigned int mnCorrectedReference;
  cv::Mat mPosGBA;
  long unsigned int mnBAGlobalForKF;


  static std::mutex mGlobalMutex;

 protected:

  // Position in absolute coordinates
  cv::Mat mWorldPos;

  // Keyframes observing the point and associated index in keyframe
//  std::map<KeyFrame*,size_t> mObservations;
  std::map<Frame*,size_t> mObservations;

  // Mean viewing direction
  cv::Mat mNormalVector;

  // Best descriptor to fast matching
  cv::Mat mDescriptor;

//  // Reference KeyFrame
//  KeyFrame* mpRefKF;

  // Tracking counters
  int mnVisible;
  int mnFound;

  // Bad flag (we do not currently erase MapPoint from memory)
  bool mbBad;
  MapPoint* mpReplaced;

  // Scale invariance distances
  float mfMinDistance;
  float mfMaxDistance;

//  Map* mpMap;

  std::mutex mMutexPos;
  std::mutex mMutexFeatures;
};

#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64
class Frame{

 public:
  explicit Frame();
  explicit Frame(const double& stamp, const cv::Mat& im, const cv::Mat& K);
  //Copy Constructor
  Frame(const Frame &frame);

  void ExtractORB(const cv::Mat& im);

  void AddMapPoint(MapPoint *pMP, const size_t &idx);

  float ComputeSceneMedianDepth(const int q);
  // Set the camera pose.
  void SetPose(cv::Mat Tcw);

  // Computes rotation, translation and camera center matrices from the camera pose.
  void UpdatePoseMatrices();

  // Returns the camera center.
  inline cv::Mat GetCameraCenter(){
    return mOw.clone();
  }

  // Returns inverse of rotation
  inline cv::Mat GetRotationInverse(){
    return mRwc.clone();
  }

  // Check if a MapPoint is in the frustum of the camera
  // and fill variables of the MapPoint to be used by the tracking
//  bool isInFrustum(MapPoint* pMP, float viewingCosLimit);

  // Compute the cell of a keypoint (return false if outside the grid)
//  bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);

//  std::vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel=-1, const int maxLevel=-1) const;

 public:
  // Frame timestamp.
  double mTimeStamp;
  cv::Mat I0;


  // Calibration matrix and OpenCV distortion parameters.
  cv::Mat mK;
  static float fx;
  static float fy;
  static float cx;
  static float cy;
  static float invfx;
  static float invfy;
  cv::Mat mDistCoef;

  // Stereo baseline multiplied by fx.
  float mbf;

  // Stereo baseline in meters.
  float mb;

  // Threshold close/far points. Close points are inserted from 1 view.
  // Far points are inserted as in the monocular case from 2 views.
  float mThDepth;

  // Number of KeyPoints.
  int N;

  // Vector of keypoints (original for visualization) and undistorted (actually used by the system).
  // In the RGB-D case, RGB images can be distorted.
  std::vector<cv::KeyPoint> mvKeys;
  std::vector<cv::KeyPoint> mvKeysUn;

  // Corresponding stereo coordinate and depth for each keypoint.
  // "Monocular" keypoints have a negative value.
  std::vector<float> mvuRight;
  std::vector<float> mvDepth;

  // ORB descriptor, each row associated to a keypoint.
  cv::Mat mDescriptors;

  // MapPoints associated to keypoints, NULL pointer if no association.
  std::vector<MapPoint*> mvpMapPoints;

  // Flag to identify outlier associations.
  std::vector<bool> mvbOutlier;

  // Keypoints are assigned to cells in a grid to reduce matching complexity when projecting MapPoints.
  static float mfGridElementWidthInv;
  static float mfGridElementHeightInv;
  std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];

  // Camera pose.
  cv::Mat mTcw;

  // Current and Next Frame id.
  static long unsigned int nNextId;
  long unsigned int mnId;

  //  // Reference Keyframe.
  //  KeyFrame* mpReferenceKF;


  // Undistorted Image Bounds (computed once).
  static float mnMinX;
  static float mnMaxX;
  static float mnMinY;
  static float mnMaxY;

  static bool mbInitialComputations;


 private:

  //  // Undistort keypoints given OpenCV distortion parameters.
  //  // Only for the RGB-D case. Stereo must be already rectified!
  //  // (called in the constructor).
  //  void UndistortKeyPoints();
  //
  // Computes image bounds for the undistorted image (called in the constructor).
//  void ComputeImageBounds(const cv::Mat &imLeft);
  //
  // Assign keypoints to the grid for speed up feature matching (called in the constructor).
//  void AssignFeaturesToGrid();

  // Rotation, translation and camera center
  cv::Mat mRcw;
  cv::Mat mtcw;
  cv::Mat mRwc;
  cv::Mat mOw; //==mtwc
};
}