//
// Created by tianyi on 08.11.22.
//

#pragma once

#include <map>

#include "hyper/messages/visual.hpp"
#include "hyper/system/components/frontends/abstract.hpp"
#include "hyper/system/components/frontends/visual/Initializer.hpp"
#include "hyper/system/components/frontends/visual/MonocularUtilizer.hpp"

namespace hyper {

/// \class The discrete stereo KLT frontend implements the callback for conventional stereo camera
/// messages (e.g. const boost::shared_ptr<const sensor_msgs::Image>). It offers the functionality of
/// taking a sequence of input images and returning labeled feature tracks.
class VisualFrontend final : public AbstractFrontend {
 public:
  /// Constructor from YAML node.
  /// \param node Input YAML node.
  explicit VisualFrontend(const Node& node);

  /// Message callback.
  /// \param sensor Sensor submitting data to the frontend.
  /// \param message Message being submitted to the frontend.
  void callback(const Sensor& sensor, const Message& message) final;


 private:
  using Queue = std::map<Stamp, std::unique_ptr<VisualTracks>>;
  using IDs = VisualTracks::Identifiers;
  using Lengths = VisualTracks::Lengths;
  using Points = VisualTracks::Points;
  using Image = cv::Mat;
  using Mask = cv::Mat;
  using Patch = cv::Size;
  using Status = std::vector<uchar>;
  using Errors = std::vector<float>;
  using Indices = std::vector<std::size_t>;
  using Criterion = cv::TermCriteria;
  using Size = std::size_t;

  /// Tracks features across images with cross-check.
  /// \param image_0 Image to track from.
  /// \param image_1 Image to track to.
  /// \param points_0 Points to track.
  /// \return Tracked points and their tracking status.
  [[nodiscard]] auto trackPoints(const Image& image_0, const Image& image_1, const Points& points_0) const -> std::tuple<Points, Status>;

  /// Tracks points across images.
  /// \param prev_I0 Previous image to track from.
  /// \param I0 Current image to track to.
  /// \param I1 Corresponding (current) other image (i.e. stereo setup).
  /// \param prev_P0 Points to track in I0.
  /// \param prev_P1 Points to track in I1.
  /// \param ids Associated IDs of prev_P0.
  /// \param lengths Associated lengths of prev_P0.
  /// \return Tuple containing tracked points P0 and P1.
  auto trackForward(const Image& prev_I0, const Image& I0, const Image* I1, Points& prev_P0, Points* prev_P1, IDs* ids, Lengths* lengths) const -> std::tuple<Points, Points>;

  /// Tracks features across views.
  /// \param previous_view Previous view.
  /// \param current_view Current view.
  /// \return Mask of tracked features.
  auto trackFeatures(VisualTracks& previous_view, VisualTracks& current_view) -> Mask;

  /// Select new point in the image.
  /// \param previous_view Previous view.
  /// \param current_view Current view.
  /// \param max_num_points Maximum number of new points.
  /// \param mask Mask to be applied (i.e. where no point can be initialized).
  auto selectFeatures(VisualTracks& previous_view, VisualTracks& current_view, const Size& max_num_points, const Mask& mask) const -> void;

  /// Initializes new points.
  /// \param previous_view Previous view.
  /// \param current_view Current view.
  /// \param new_previous_points_0 Candidate points to initialize.
  auto circularInitialization(VisualTracks& previous_view, VisualTracks& current_view, Points& new_previous_points_0) const -> void;

  void MonocularInitialization();

  void CreateInitialMapMonocular();

  int SearchForInitialization(Frame &F1, Frame &F2, std::vector<cv::Point2f> &vbPrevMatched, std::vector<int> &vnMatches12);
  void Reset();

  Queue queue_;               ///< Queue.
  Size max_num_tracks_;      ///< Maximum number of feature tracks.
  int min_track_separation_; ///< Minimum distance in pixel between tracks.
  int patch_size_;           ///< Patch size (in pixel) to be used in KLT feature tracker.
  Patch patch_;              ///< Patch to be used in KLT feature tracker.
  int num_pyramid_levels_;   ///< Number of pyramid levels to use in tracker.
  Criterion criterion_;      ///< Termination criterion.
  Scalar min_track_quality_; ///< Minimum feature quality to allow tracking.
  Scalar max_track_error_;   ///< Maximum allowed pixel error between forward and backward tracking.
  bool show_tracks_;         ///< Enables visual output of feature tracks.


  // =============== For Initialization =============================

  // Tracking states
  enum eTrackingState{
    SYSTEM_NOT_READY=-1,
    NO_IMAGES_YET=0,
    NOT_INITIALIZED=1,
    OK=2,
    LOST=3
  };

  // Input sensor
  enum eSensor{
    MONOCULAR=0,
    STEREO=1,
    RGBD=2
  };

  // initializer
  eTrackingState mState;
  eTrackingState mLastProcessedState;
  eSensor mSensor;
  Initializer* mpInitializer;
  cv::Mat mK;
  // Matching
  Frame mCurrentFrame;
  Frame mInitialFrame;
  Frame mLastFrame;
  Camera mCurrentCamera;
  Camera mInitialCamera;
  const int TH_HIGH = 10000;
  const int TH_LOW = 0;
  // Save 2D points and 3D points
  std::vector<cv::Point2f> mvbPrevMatched;
  std::vector<int> mvIniMatches;
  std::vector<cv::Point3f> mvIniP3D;
  std::unique_ptr<VisualTracksInit> mInitialTracks;

};

} // namespace hyper
