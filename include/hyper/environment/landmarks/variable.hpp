/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.#pragma once

#pragma once

#include "hyper/variables/forward.hpp"

#include "hyper/environment/landmarks/abstract.hpp"

namespace hyper {

template <typename TVariable>
class Landmark final
    : public AbstractLandmark {
 public:
  // Definitions.
  using Variable = TVariable;

  /// Default constructor.
  Landmark() = default;

  /// Constructor from variable.
  /// \param derived Input variable.
  explicit Landmark(const Eigen::Ref<const Variable>& variable)
      : variable_{variable} {}

  /// Variables accessor.
  /// \return Variables.
  [[nodiscard]] auto variables() const -> Pointers<const AbstractVariable<Scalar>> final {
    return {&variable_};
  }

  /// Variables modifier.
  /// \return Variables.
  [[nodiscard]] auto variables() -> Pointers<AbstractVariable<Scalar>> final {
    return {&variable_};
  }

  /// Variables accessor.
  /// \param stamp Input stamp.
  /// \return Variables.
  [[nodiscard]] auto variables(const Stamp& /* stamp */) const -> Pointers<const AbstractVariable<Scalar>> final {
    return variables();
  }

  /// Variables modifier.
  /// \param stamp Input stamp.
  /// \return Variables.
  [[nodiscard]] auto variables(const Stamp& /* stamp */) -> Pointers<AbstractVariable<Scalar>> final {
    return variables();
  }

  /// Variable accessor.
  /// \return Variable.
  [[nodiscard]] auto variable() const -> const Variable& {
    return variable_;
  }

  /// Variable modifier.
  /// \return Variable.
  auto variable() -> Variable& {
    return const_cast<Variable&>(std::as_const(*this).variable());
  }

 private:
  Variable variable_;
};

} // namespace hyper
