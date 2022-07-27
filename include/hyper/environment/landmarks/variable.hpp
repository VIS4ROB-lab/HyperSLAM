/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.#pragma once

#pragma once

#include "hyper/variables/forward.hpp"

#include "hyper/environment/landmarks/abstract.hpp"

namespace hyper {

template <typename TVariable>
class VariableLandmark final
    : public AbstractLandmark {
 public:
  /// Default constructor.
  VariableLandmark() = default;

  /// Constructor from variable.
  /// \param derived Input variable.
  explicit VariableLandmark(const Eigen::Ref<const TVariable>& variable)
      : variable_{variable} {}

  /// Parameters accessor.
  /// \return Parameters.
  [[nodiscard]] auto parameters() const -> ConstParameters final {
    return std::as_const(variable_).memory();
  }

  /// Parameters modifier.
  /// \return Parameters.
  [[nodiscard]] auto parameters() -> Parameters final {
    return variable_.memory();
  }

  /// Variable accessor.
  /// \return Variable.
  [[nodiscard]] auto variable() const -> const TVariable& {
    return variable_;
  }

  /// Variable modifier.
  /// \return Variable.
  auto variable() -> TVariable& {
    return const_cast<TVariable&>(std::as_const(*this).variable());
  }

 private:
  TVariable variable_;
};

using PositionLandmark = VariableLandmark<Position<Scalar>>;

} // namespace hyper
