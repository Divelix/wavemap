#ifndef WAVEMAP_CORE_CONFIG_STRING_LIST_H_
#define WAVEMAP_CORE_CONFIG_STRING_LIST_H_

#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "wavemap/core/config/param.h"

namespace wavemap {
struct StringList {
  using ValueType = std::vector<std::string>;
  ValueType value{};

  // Constructors
  StringList() = default;
  StringList(ValueType value) : value(std::move(value)) {}  // NOLINT

  // Assignment operator
  StringList& operator=(ValueType rhs) {
    value = std::move(rhs);
    return *this;
  }

  // Allow implicit conversions to the underlying type
  operator ValueType&() { return value; }
  operator const ValueType&() const { return value; }

  // Method to load from configs
  static std::optional<StringList> from(const param::Value& param);

  // Method to facilitate printing
  std::string toStr() const;
};
}  // namespace wavemap

#endif  // WAVEMAP_CORE_CONFIG_STRING_LIST_H_
