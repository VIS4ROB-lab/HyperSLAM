/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include <map>

#include "hyper/messages/visual.hpp"
#include "hyper/system/components/frontends/abstract.hpp"

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

  Queue queue_; ///< Queue.

  Size max_num_tracks_;      ///< Maximum number of feature tracks.
  int min_track_separation_; ///< Minimum distance in pixel between tracks.
  int patch_size_;           ///< Patch size (in pixel) to be used in KLT feature tracker.
  Patch patch_;              ///< Patch to be used in KLT feature tracker.
  int num_pyramid_levels_;   ///< Number of pyramid levels to use in tracker.
  Criterion criterion_;      ///< Termination criterion.
  Scalar min_track_quality_; ///< Minimum feature quality to allow tracking.
  Scalar max_track_error_;   ///< Maximum allowed pixel error between forward and backward tracking.
  bool show_tracks_;         ///< Enables visual output of feature tracks.
};

} // namespace hyper
