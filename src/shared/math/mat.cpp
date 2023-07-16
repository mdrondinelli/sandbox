#include "mat.h"

namespace marlon {
namespace math {
  static_assert(sizeof(Mat2x2i) == 16);
  static_assert(sizeof(Mat2x2f) == 16);
  static_assert(sizeof(Mat2x2d) == 32);
  static_assert(sizeof(Mat2x3i) == 24);
  static_assert(sizeof(Mat2x3f) == 24);
  static_assert(sizeof(Mat2x3d) == 48);
  static_assert(sizeof(Mat2x4i) == 32);
  static_assert(sizeof(Mat2x4f) == 32);
  static_assert(sizeof(Mat2x4d) == 64);
  static_assert(sizeof(Mat3x2i) == 24);
  static_assert(sizeof(Mat3x2f) == 24);
  static_assert(sizeof(Mat3x2d) == 48);
  static_assert(sizeof(Mat3x3i) == 36);
  static_assert(sizeof(Mat3x3f) == 36);
  static_assert(sizeof(Mat3x3d) == 72);
  static_assert(sizeof(Mat3x4i) == 48);
  static_assert(sizeof(Mat3x4f) == 48);
  static_assert(sizeof(Mat3x4d) == 96);
  static_assert(sizeof(Mat4x2i) == 32);
  static_assert(sizeof(Mat4x2f) == 32);
  static_assert(sizeof(Mat4x2d) == 64);
  static_assert(sizeof(Mat4x3i) == 48);
  static_assert(sizeof(Mat4x3f) == 48);
  static_assert(sizeof(Mat4x3d) == 96);
  static_assert(sizeof(Mat4x4i) == 64);
  static_assert(sizeof(Mat4x4f) == 64);
  static_assert(sizeof(Mat4x4d) == 128);
}
} // namespace marlon