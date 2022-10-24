/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#include <boost/uuid/uuid_generators.hpp>

#include "hyper/system/components/abstract.hpp"

namespace hyper {

namespace {

using Generator = boost::uuids::random_generator;

auto generator() -> Generator& {
  static Generator generator_;
  return generator_;
}

} // namespace

auto AbstractComponent::uuid() const -> const UUID& {
  return uuid_;
}

AbstractComponent::AbstractComponent()
    : uuid_{generator()()} {}

} // namespace hyper
