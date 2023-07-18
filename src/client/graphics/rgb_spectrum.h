#ifndef MARLONR_RGB_SPECTRUM
#define MARLONR_RGB_SPECTRUM

namespace marlon {
namespace rendering {
class Rgb_spectrum {
public:
  float r;
  float g;
  float b;

  constexpr Rgb_spectrum(float r, float g, float b) noexcept
      : r{r}, g{g}, b{b} {}
};
} // namespace rendering
} // namespace marlon

#endif