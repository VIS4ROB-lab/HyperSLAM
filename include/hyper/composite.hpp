/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include <tuple>
#include <type_traits>

#include <glog/logging.h>

namespace hyper {

/// Creates an iterable composite of multiple containers.
/// This is useful for container types containing the
/// elements which should be iterated through simultaneously.
/// Example usage: for(const auto& [var0,...,varn] : composite) { ... }.
/// \tparam TArg Any number of container types.
template <typename... TArg>
struct Composite {
  static auto constexpr Begin{[](auto&&... args) { return std::make_tuple(begin(args)...); }};
  static auto constexpr End{[](auto&&... args) { return std::make_tuple(end(args)...); }};

  /// Check whether the containers are compatible.
  /// \param args References to containers.
  /// \return True if containers are compatible in size.
  static auto CheckCompatibility(TArg&&... args) -> bool {
    const auto [compatible, size] = CompatibilityCheck(std::forward<TArg>(args)...);
    return compatible;
  }

  struct Iterator {
    template <typename T, std::size_t i = 0, std::size_t j = std::tuple_size_v<T>>
    struct Compare {
      static auto equal(T const& lhs, T const& rhs) -> bool {
        if constexpr (i == j)
          return false;
        else {
          return (std::get<i>(lhs) == std::get<i>(rhs) || Compare<T, i + 1, j>::equal(lhs, rhs));
        }
      }
    };

    decltype(auto) operator++() {
      std::apply([](auto&... args) { ((++args), ...); }, iterators_);
      return (*this);
    }

    auto operator!=(const Iterator& rhs) const {
      return !Compare<decltype(iterators_)>::equal(iterators_, rhs.iterators_);
    }

    auto operator*() const {
      return std::apply([](auto&... args) { return std::forward_as_tuple(*args...); }, iterators_);
    }

    std::invoke_result_t<decltype(Begin), TArg&&...> iterators_;
  };

  auto begin() const -> Iterator {
    return {std::apply(Begin, references_)};
  }

  auto end() const -> Iterator {
    return {std::apply(End, references_)};
  }

  std::invoke_result_t<decltype(&std::forward_as_tuple<TArg...>), TArg&&...> references_;

 private:
  /// Innermost template recursion.
  /// \tparam TArg_ Container with size() function.
  /// \param arg Reference to container.
  /// \return Validity and size pair.
  template <typename TArg_>
  static constexpr auto CompatibilityCheck(const TArg_& arg_) -> std::pair<bool, std::size_t> {
    return {true, arg_.size()};
  }

  /// Template recursion to check for same-sized containers.
  /// \tparam TArg_ Outermost container with size() function.
  /// \tparam Args_ Remaining containers with size() function.
  /// \param arg Reference to outer-most container.
  /// \param args References to remaining containers.
  /// \return Validity and size pair.
  template <typename TArg_, typename... Args_>
  static constexpr auto CompatibilityCheck(const TArg_& arg_, Args_&&... args_) -> std::pair<bool, std::size_t> {
    const auto [compatible, size] = CompatibilityCheck(std::forward<Args_>(args_)...);
    return {compatible && (arg_.size() == size), size};
  }
};

/// Creates a composite from a number of containers
/// and checks that the containers have the same number
/// of elements.
/// \tparam TArg Any number of container types.
/// \param args Input arguments to build composite from.
/// \return The created composite.
template <typename... TArg>
auto makeComposite(TArg&&... args) -> Composite<TArg...> {
  DCHECK(Composite<TArg...>::CheckCompatibility(std::forward<TArg>(args)...)) << "Containers must be compatible to create composite.";
  return {std::forward_as_tuple(args...)};
}

} // namespace hyper
