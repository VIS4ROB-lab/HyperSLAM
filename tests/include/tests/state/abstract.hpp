/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include "hyper/state/abstract.hpp"

#include "tests/random.hpp"

namespace hyper::tests {

template <>
class Mock<AbstractState> {
 public:
  using Rate = Stamp;
  using InclusiveRange = Range<Stamp, BoundaryPolicy::INCLUSIVE>;

  static constexpr auto kDefaultRate = Rate{5};
  static constexpr auto kDefaultRange = InclusiveRange{Scalar{0}, Scalar{10}};

  /// Deleted default constructor.
  Mock() = delete;

  /// Creates a random state.
  /// \tparam TAmbientSpace_ Ambient space type.
  /// \param inclusive_range Inclusive input range.
  /// \param rate Input rate.
  /// \return Random state.
  template <typename TAmbientSpace_>
  static auto Random(const InclusiveRange& inclusive_range = kDefaultRange, const Rate& rate = kDefaultRate) -> std::unique_ptr<AbstractState> {
    auto state = std::make_unique<AbstractState>();
    for (const auto& sample : inclusive_range.sample(rate)) {
      auto stamped_element = std::make_unique<Stamped<TAmbientSpace_>>();
      stamped_element->stamp() = sample;
      stamped_element->variable() = Mock<TAmbientSpace_>::Random();
      state->elements().insert(std::move(stamped_element));
    }
    return state;
  }
};

} // namespace hyper::tests
