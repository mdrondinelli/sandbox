#ifndef MARLONR_GRAPHICS_RGB_SPECTRUM_H
#define MARLONR_GRAPHICS_RGB_SPECTRUM_H

namespace marlon {
namespace graphics {
class Rgb_spectrum {
public:
  float r;
  float g;
  float b;

  constexpr Rgb_spectrum(float r, float g, float b) noexcept
      : r{r}, g{g}, b{b} {}
};
} // namespace graphics
} // namespace marlon

#endif