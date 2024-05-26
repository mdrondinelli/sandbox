#ifndef MARLONR_GRAPHICS_RGB_SPECTRUM_H
#define MARLONR_GRAPHICS_RGB_SPECTRUM_H

namespace marlon {
namespace graphics {
class Rgb_spectrum {
public:
  float r;
  float g;
  float b;

  constexpr Rgb_spectrum(float rgb) noexcept : r{rgb}, g{rgb}, b{rgb} {}

  constexpr Rgb_spectrum(float r, float g, float b) noexcept
      : r{r}, g{g}, b{b} {}

  static constexpr Rgb_spectrum black() noexcept {
    return Rgb_spectrum{0.0f};
  }

  static constexpr Rgb_spectrum white() noexcept {
    return Rgb_spectrum{1.0f};
  }
};

constexpr Rgb_spectrum mix(Rgb_spectrum a, Rgb_spectrum b, float t) noexcept {
  auto const wa = 1.0f - t;
  auto const wb = t;
  return {wa * a.r + wb * b.r, wa * a.g + wb * b.g, wa * a.b + wb * b.b};
}
} // namespace graphics
} // namespace marlon

#endif