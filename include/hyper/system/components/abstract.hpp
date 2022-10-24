/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include <utility>

#include <boost/uuid/uuid.hpp>

#include "hyper/system/components/forward.hpp"
#include "hyper/yaml/forward.hpp"

namespace hyper {

class AbstractComponent {
 public:
  using Node = YAML::Node;
  using UUID = boost::uuids::uuid;

  /// Virtual destructor.
  virtual ~AbstractComponent() = default;

  /// UUID accessor.
  /// \return UUID.
  [[nodiscard]] auto uuid() const -> const UUID&;

  /// Downcasts this instance.
  /// \tparam TDerived_ Target type.
  /// \return Cast instance.
  template <typename TDerived_> inline auto as() const -> const TDerived_& {
    return static_cast<const TDerived_&>(*this);
  }

  /// Downcasts this instance.
  /// \tparam TDerived_ Target type.
  /// \return Cast instance.
  template <typename TDerived_> inline auto as() -> TDerived_& {
    return const_cast<TDerived_&>(std::as_const(*this).template as<TDerived_>());
  }

 protected:
  /// Default constructor.
  AbstractComponent();

 private:
  UUID uuid_; ///< UUID.
};

} // namespace hyper
