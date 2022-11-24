///// This file is subject to the terms and conditions defined in
///// the 'LICENSE' file, which is part of this repository.
//
//#include <numeric>
//
//#include <opencv2/highgui.hpp>
//#include <opencv2/video.hpp>
//
//#include "hyper/composite.hpp"
//#include "hyper/sensors/camera.hpp"
//#include "hyper/system/components/backend.hpp"
//#include "hyper/system/components/frontends/visual/klt.hpp"
//#include "hyper/yaml/yaml.hpp"
//
//namespace hyper {
//
//namespace {
//
///// Appends a vector to another.
///// \tparam TElement Element type.
///// \tparam OtherElementType Other element type.
///// \param v0 Vector to append to.
///// \param v1 Vector being appended.
//template <typename TElement, typename TOtherElement = TElement>
//auto append(std::vector<TElement>& v0, const std::vector<TOtherElement>& v1) -> void {
//  v0.reserve(v0.size() + v1.size());
//  v0.insert(v0.end(), v1.begin(), v1.end());
//}
//
///// Select elements by indices.
///// \tparam TElement Element type.
///// \tparam TIndex Index type.
///// \param elements Vector of elements.
///// \param indices Vector of indices.
///// \return Selection of elements.
//template <typename TElement, typename TIndex>
//auto select(std::vector<TElement>& elements, const std::vector<TIndex>& indices) -> void {
//  std::vector<TElement> selected_elements;
//  selected_elements.reserve(indices.size());
//  for (const auto& index : indices) {
//    selected_elements.emplace_back(elements[index]);
//  }
//  elements = std::move(selected_elements);
//}
//
///// Prunes elements by condition.
///// \tparam TElement Element type.
///// \tparam TCondition Conditions type.
///// \param elements Vector of elements.
///// \param conditions Vector of conditions.
//template <typename TElement, typename TCondition>
//auto prune(std::vector<TElement>& elements, const std::vector<TCondition>& conditions) -> void {
//  using TSize = typename std::vector<TElement>::size_type;
//  DCHECK_EQ(elements.size(), conditions.size());
//  auto j = TSize{0};
//  for (auto i = TSize{0}; i < elements.size(); ++i) {
//    if (conditions[i]) elements[j++] = std::move(elements[i]);
//  }
//  elements.erase(elements.cbegin() + j, elements.cend());
//}
//
///// Sorts element indices according to element order.
///// \tparam TElement Element type.
///// \tparam TComparator Comparator type.
///// \param elements Vector of elements.
///// \param comparator Comparator.
///// \return Sorted indices.
//template <typename TElement, typename TComparator = std::greater<TElement>>
//auto indexSort(const std::vector<TElement>& elements, TComparator comparator = TComparator{}) -> std::vector<typename std::vector<TElement>::size_type> {
//  using TIndex = typename std::vector<TElement>::size_type;
//  std::vector<TIndex> indices(elements.size());
//  std::iota(indices.begin(), indices.end(), TIndex{0});
//  std::sort(indices.begin(), indices.end(), [&elements, &comparator](const auto i, const auto j) -> bool { return comparator(elements[i], elements[j]); });
//  return indices;
//}
//
///// Computes the distance between points.
///// \tparam ScalarType Scalar type.
///// \param px First point.
///// \param py Second point.
///// \return Distance between points.
//template <typename ScalarType>
//auto distance(const cv::Point_<ScalarType>& px, const cv::Point_<ScalarType>& py) -> ScalarType {
//  const auto dx = px.x - py.x;
//  const auto dy = px.y - py.y;
//  return std::sqrt(dx * dx + dy * dy);
//}
//
///// Checks whether a point lies inside the image.
///// \tparam ImageType Image type.
///// \tparam ScalarType Scalar type.
///// \param image Image to be used.
///// \param point Point to be used.
///// \return True if point lies inside the image.
//template <typename ImageType, typename ScalarType>
//auto contains(const ImageType& image, const cv::Point_<ScalarType>& point) -> bool {
//  constexpr auto kBorder = 1;
//  const auto x = cvRound(point.x);
//  const auto y = cvRound(point.y);
//  return kBorder <= x && kBorder <= y && x <= image.cols - kBorder && y < image.rows - kBorder;
//}
//
//auto showTracks(const std::string& window_name, const VisualTracks& previous_view, const VisualTracks& current_view) -> void {
//  // Colors and constants.
//  constexpr auto kPointSize = 2;
//  const auto color_0 = cv::Scalar(255, 0, 255);
//  const auto color_1 = cv::Scalar(0, 255, 255);
//
//  const auto& [current_I0, current_P0] = current_view.tracks.cbegin()->second;
//  const auto& [current_I1, current_P1] = current_view.tracks.crbegin()->second;
//  const auto& [previous_I0, previous_P0] = previous_view.tracks.cbegin()->second;
//  const auto& [previous_I1, previous_P1] = previous_view.tracks.crbegin()->second;
//
//  // Create image.
//  cv::Mat image;
//  cv::hconcat(current_I0->image, current_I1->image, image);
//
//  // Color lambda.
//  auto color_lambda = [&color_0, &color_1](const float length) {
//    const auto factor = std::min(1.f, length / 20.f);
//    return (1.f - factor) * color_0 + factor * color_1; };
//
//  // Convert to color image and plot points.
//  cv::cvtColor(image, image, cv::COLOR_GRAY2RGB);
//  for (const auto& [previous_point_0, current_point_0, length] : makeComposite(previous_P0, current_P0, current_view.lengths)) {
//    cv::circle(image, current_point_0, kPointSize, color_lambda(static_cast<float>(length)), kPointSize);
//    cv::arrowedLine(image, previous_point_0, current_point_0, color_1);
//  }
//
//  const auto offset = cv::Point2f{static_cast<float>(current_I0->image.cols), 0.f};
//  for (const auto& [previous_point_1, current_point_1, length] : makeComposite(previous_P1, current_P1, current_view.lengths)) {
//    const auto offset_previous_point_1 = previous_point_1 + offset;
//    const auto offset_current_point_1 = current_point_1 + offset;
//    cv::circle(image, offset_current_point_1, kPointSize, color_lambda(static_cast<float>(length)), kPointSize);
//    cv::arrowedLine(image, offset_previous_point_1, offset_current_point_1, color_1);
//  }
//
//  // Show the image.
//  cv::imshow(window_name, image);
//  cv::waitKey(1);
//}
//
//} // namespace
//
//VisualFrontend::VisualFrontend(const Node& node)
//    : AbstractFrontend{node},
//      max_num_tracks_{yaml::ReadAs<decltype(max_num_tracks_)>(node, "max_num_tracks")},
//      min_track_separation_{yaml::ReadAs<decltype(min_track_separation_)>(node, "min_track_separation")},
//      patch_size_{yaml::ReadAs<decltype(patch_size_)>(node, "patch_size")},
//      patch_{patch_size_, patch_size_},
//      num_pyramid_levels_{yaml::ReadAs<decltype(num_pyramid_levels_)>(node, "num_pyramid_levels")},
//      criterion_{cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01},
//      min_track_quality_{yaml::ReadAs<decltype(min_track_quality_)>(node, "min_track_quality")},
//      max_track_error_{yaml::ReadAs<decltype(max_track_error_)>(node, "max_track_error")},
//      show_tracks_{yaml::ReadAs<decltype(show_tracks_)>(node, "show_tracks")} {}
//
//void VisualFrontend::callback(const Sensor& sensor, const Message& message) {
//  // Convert message.
//  auto image = boost::static_pointer_cast<const sensor_msgs::Image>(message);
//  auto cv_image = cv_bridge::toCvShare(image, "mono8");
//
//  // Try emplace stamp.
//  const auto stamp = image->header.stamp.toSec();
//  auto [queue_itr, inserted] = queue_.try_emplace(stamp, nullptr);
//
//  const auto& camera = sensor.as<Camera>();
//  if (!inserted) { // Add new track.
//    auto& [I, _] = queue_itr->second->addTrack(camera);
//    I = std::move(cv_image);
//
//  } else { // Create view and add track.
//    queue_itr->second = std::make_unique<VisualTracks>(stamp, camera);
//    auto& [I, _] = queue_itr->second->getTrack(camera);
//    I = std::move(cv_image);
//  }
//
//  // Check for non-empty message queue.
//  if (queue_.empty()) return;
//
//  // Growing message queue warning.
//  DLOG_IF(WARNING, 5 <= queue_.size()) << "Growing message queue.";
//
//  // Search next message (or return if none exists).
//  const auto previous_itr = queue_.cbegin();
//  const auto current_itr = std::find_if(std::next(previous_itr), queue_.cend(), [](const auto& itr) -> bool { return itr.second->tracks.size() == 2; });
//  if (current_itr == queue_.cend()) return;
//
//  // Unpack iterators.
//  const auto& [previous_stamp, previous_view] = *previous_itr;
//  const auto& [current_stamp, current_view] = *current_itr;
//
//  // Detect incomplete views.
//  DLOG_IF(WARNING, 1 < std::distance(previous_itr, current_itr)) << "Incomplete stereo frame(s) detected.";
//  if (previous_view->tracks.size() < 2) {
//    DLOG(WARNING) << "Dropping incomplete message at beginning of queue.";
//    queue_.erase(previous_itr);
//    return;
//  }
//
//  // Track points.
//  const auto mask = trackFeatures(*previous_view, *current_view);
//  selectFeatures(*previous_view, *current_view, max_num_tracks_, mask);
//
//  // Show tracks.
//  if (show_tracks_) {
//    showTracks("Feature Tracks", *previous_view, *current_view);
//  }
//
//  // Extract and submit message.
//  queue_.erase(std::next(previous_itr), current_itr);
//  backend().submit(std::move(queue_.extract(previous_itr).mapped()));
//}
//
//auto VisualFrontend::trackPoints(const Image& image_0, const Image& image_1, const Points& points_0) const -> std::tuple<Points, Status> {
//  // Allocate memory.
//  Points points_1;
//  Status status_0, status_1;
//  Errors errors;
//
//  // Forward track points (with cross-check).
//  cv::calcOpticalFlowPyrLK(image_0, image_1, points_0, points_1, status_0, errors, patch_, num_pyramid_levels_, criterion_);
//  Points points_10 = points_0;
//  cv::calcOpticalFlowPyrLK(image_1, image_0, points_1, points_10, status_1, errors, patch_, num_pyramid_levels_, criterion_, cv::OPTFLOW_USE_INITIAL_FLOW);
//
//  // Discard tracks.
//  for (auto i = Size{0}; i < status_1.size(); ++i) {
//    status_1[i] = status_0[i] && status_1[i] && contains(image_1, points_1[i]) && distance(points_0[i], points_10[i]) < max_track_error_;
//  }
//
//  // Return tracked points and status.
//  return {std::move(points_1), std::move(status_1)};
//}
//
//auto VisualFrontend::trackForward(const Image& prev_I0, const Image& I0, const Image* I1, Points& prev_P0, Points* prev_P1, IDs* ids, Lengths* lengths) const -> std::tuple<Points, Points> {
//  // Track forward.
//  if (!prev_P0.empty()) {
//    auto [P0, S0] = trackPoints(prev_I0, I0, prev_P0);
//    prune(prev_P0, S0);
//    if (prev_P1) prune(*prev_P1, S0);
//    prune(P0, S0);
//    if (ids) prune(*ids, S0);
//    if (lengths) prune(*lengths, S0);
//
//    // Track sideways.
//    if (I1 && !P0.empty()) {
//      auto [P1, S1] = trackPoints(I0, *I1, P0);
//      prune(prev_P0, S1);
//      if (prev_P1) prune(*prev_P1, S1);
//      prune(P0, S1);
//      prune(P1, S1);
//      if (ids) prune(*ids, S1);
//      if (lengths) prune(*lengths, S1);
//      return {std::move(P0), std::move(P1)};
//    }
//
//    return {std::move(P0), Points{}};
//  }
//
//  // Nothing to track.
//  DLOG(WARNING) << "Previous feature set is empty. Forward tracking failed.";
//  return {};
//}
//
//auto VisualFrontend::trackFeatures(VisualTracks& previous_view, VisualTracks& current_view) -> Mask {
//  // Constants.
//  constexpr auto kMaskValue = 0;
//  constexpr auto kFreeValue = 255;
//
//  // Unpack view.
//  auto& previous_ids = previous_view.identifiers;
//  auto& previous_lengths = previous_view.lengths;
//
//  auto& [current_I0, current_P0] = current_view.tracks.begin()->second;
//  auto& [current_I1, current_P1] = current_view.tracks.rbegin()->second;
//  auto& [previous_I0, previous_P0] = previous_view.tracks.begin()->second;
//  auto& [previous_I1, previous_P1] = previous_view.tracks.rbegin()->second;
//
//  // Track points.
//  std::tie(current_P0, current_P1) = trackForward(previous_I0->image, current_I0->image, &current_I1->image, previous_P0, &previous_P1, &previous_ids, &previous_lengths);
//
//  // Initialize mask.
//  Mask mask{previous_I0->image.rows, previous_I0->image.cols, CV_8UC1, cv::Scalar(kFreeValue)};
//
//  // Select best points.
//  Indices selected_indices;
//  selected_indices.reserve(max_num_tracks_);
//  for (const auto& index : indexSort(previous_lengths)) {
//    const auto& point = previous_P0[index];
//    if (mask.at<uchar>(point) == kFreeValue) {
//      cv::circle(mask, point, min_track_separation_, kMaskValue, cv::FILLED);
//      selected_indices.emplace_back(index);
//    }
//  }
//
//  // Select best points.
//  select(previous_P0, selected_indices);
//  select(current_P0, selected_indices);
//  if (!previous_P1.empty()) select(previous_P1, selected_indices);
//  if (!current_P1.empty()) select(current_P1, selected_indices);
//  select(previous_ids, selected_indices);
//  select(previous_lengths, selected_indices);
//
//  // Update points.
//  current_view.identifiers = previous_ids;
//  current_view.lengths = previous_lengths;
//  for (auto& length : current_view.lengths) {
//    ++length;
//  }
//
//  // Returns mask.
//  return mask;
//}
//
//auto VisualFrontend::selectFeatures(VisualTracks& previous_view, VisualTracks& current_view, const Size& max_num_points, const Mask& mask) const -> void {
//  // Unpack view.
//  auto& [current_I0, current_P0] = current_view.tracks.begin()->second;
//  auto& [previous_I0, previous_P0] = previous_view.tracks.begin()->second;
//
//  // Compute maximum number of new points.
//  const auto num_current_points = current_P0.size();
//  const auto max_num_new_points = max_num_points - num_current_points;
//
//  if (0 < max_num_new_points) {
//    // Initialize new points.
//    Points new_previous_P0;
//    new_previous_P0.reserve(max_num_new_points);
//    cv::goodFeaturesToTrack(previous_I0->image, new_previous_P0, static_cast<int>(max_num_new_points), min_track_quality_, min_track_separation_, mask);
//    circularInitialization(previous_view, current_view, new_previous_P0);
//  }
//}
//
//auto VisualFrontend::circularInitialization(VisualTracks& previous_view, VisualTracks& current_view, Points& new_previous_P0) const -> void {
//  // Unpack view.
//  auto& [current_I0, current_P0] = current_view.tracks.begin()->second;
//  auto& [current_I1, current_P1] = current_view.tracks.rbegin()->second;
//  auto& [previous_I0, previous_P0] = previous_view.tracks.begin()->second;
//  auto& [previous_I1, previous_P1] = previous_view.tracks.rbegin()->second;
//
//  // Filter backwards.
//  auto [new_current_P0, new_current_P1] = trackForward(previous_I0->image, current_I0->image, &current_I1->image, new_previous_P0, nullptr, nullptr, nullptr);
//
//  Points new_previous_P1;
//  if (!new_previous_P0.empty() && previous_view.tracks.size() > 1 && current_view.tracks.size() > 1) {
//    Status status_0, status_1;
//    Errors errors;
//
//    // Circular tracking.
//    cv::calcOpticalFlowPyrLK(previous_I0->image, previous_I1->image, new_previous_P0, new_previous_P1, status_0, errors, patch_, num_pyramid_levels_, criterion_);
//    Points circular_new_current_P1 = new_current_P1;
//    cv::calcOpticalFlowPyrLK(previous_I1->image, current_I1->image, new_previous_P1, circular_new_current_P1, status_1, errors, patch_, num_pyramid_levels_, criterion_, cv::OPTFLOW_USE_INITIAL_FLOW);
//
//    // Discard invalid points.
//    for (auto i = Size{0}; i < status_0.size(); ++i) {
//      status_0[i] = status_0[i] && status_1[i] && contains(current_I1->image, new_current_P1[i]) && distance(new_current_P1[i], circular_new_current_P1[i]) < max_track_error_;
//    }
//
//    // Prune invalid points.
//    prune(new_previous_P0, status_0);
//    prune(new_previous_P1, status_0);
//    prune(new_current_P0, status_0);
//    prune(new_current_P1, status_0);
//
//    DCHECK(new_previous_P0.size() == new_current_P0.size());
//    DCHECK(new_previous_P0.size() == new_previous_P1.size());
//    DCHECK(new_current_P0.size() == new_current_P1.size());
//  }
//
//  // Append new points.
//  append(previous_P0, new_previous_P0);
//  append(previous_P1, new_previous_P1);
//  append(current_P0, new_current_P0);
//  append(current_P1, new_current_P1);
//
//  // Calculate old and new size.
//  const auto old_size = current_view.identifiers.size();
//  const auto diff_size = new_current_P0.size();
//  const auto new_size = old_size + diff_size;
//
//  // ID generation.
//  static Size id_generator = 0;
//  auto current_id = id_generator;
//  id_generator += diff_size;
//
//  // Initialize new IDs.
//  previous_view.identifiers.reserve(new_size);
//  current_view.identifiers.reserve(new_size);
//  for (auto i = Size{0}; i < diff_size; ++i) {
//    previous_view.identifiers.emplace_back(current_id);
//    current_view.identifiers.emplace_back(current_id);
//    ++current_id;
//  }
//
//  // Initialize new lengths.
//  previous_view.lengths.resize(new_size, 0);
//  current_view.lengths.resize(new_size, 1);
//
//  DCHECK(previous_view.identifiers.size() == current_view.identifiers.size());
//  DCHECK(previous_view.lengths.size() == current_view.lengths.size());
//}
//
//} // namespace hyper
