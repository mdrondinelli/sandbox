#ifndef MARLON_UTIL_CAPACITY_ERROR_H
#define MARLON_UTIL_CAPACITY_ERROR_H

#include <exception>
#include <memory>
#include <string>
#include <string_view>

namespace marlon {
namespace util {
class Capacity_error : public std::exception {
public:
  explicit Capacity_error(std::string_view what_arg)
      : _what{std::make_shared<std::string>(what_arg)} {}

  char const *what() const noexcept {
    return _what->c_str();
  }

private:
  std::shared_ptr<std::string> _what;
};
} // namespace util
} // namespace marlon

#endif