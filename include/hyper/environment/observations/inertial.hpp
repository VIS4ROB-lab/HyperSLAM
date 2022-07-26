/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include <glog/logging.h>

#include "hyper/environment/observations/abstract.hpp"
#include "hyper/variables/gravity.hpp"

namespace hyper {

template <typename TManifold>
class InertialObservation final
    : public AbstractObservation {
 public:
  // Definitions.
  using Measurement = InertialMeasurement<TManifold>;

  /// Constructor from measurement and gravity.
  /// \param measurement Measurement to use.
  /// \param gravity Gravity data.
  InertialObservation(std::unique_ptr<Measurement>&& measurement, const Scalar* gravity)
      : AbstractObservation{std::move(measurement)}, gravity_{gravity} {
    DCHECK(gravity_ != nullptr);
  }

  /// Default destructor.
  ~InertialObservation() final = default;

  /// Measurement accessor.
  /// \return Measurement.
  [[nodiscard]] auto measurement() const -> const Measurement& {
    return static_cast<const Measurement&>(AbstractObservation::measurement()); // NOLINT
  }

  /// Measurement modifier.
  /// \return Measurement.
  [[nodiscard]] auto measurement() -> Measurement& {
    return const_cast<Measurement&>(std::as_const(*this).measurement());
  }

  /// Collects the memory blocks.
  /// \return Memory blocks.
  [[nodiscard]] auto memoryBlocks() const -> MemoryBlocks<Scalar> final {
    return {{const_cast<Scalar*>(gravity_), Traits<Gravity<Scalar>>::kNumParameters}};
  }

  /// Fetch the gravity vector.
  /// \return Pointer to gravity vector.
  [[nodiscard]] auto gravity() const -> Eigen::Map<const Gravity<Scalar>> {
    return Eigen::Map<const Gravity<Scalar>>{gravity_};
  }

 private:
  const Scalar* gravity_; ///< Gravity data.
};

} // namespace hyper
