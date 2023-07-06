#ifndef MARLONR_RENDERING_UNIQUE_PTR
#define MARLONR_RENDERING_UNIQUE_PTR

namespace marlon {
namespace rendering {
class Renderer;

template <typename T, typename T_create_info> class Unique_ptr {
  Renderer *_renderer;
  T *_object;

public:
  explicit Unique_ptr(Renderer *renderer, T_create_info const &create_info)
      : _renderer{renderer}, _p{renderer->create(create_info)} {}

  ~Unique_ptr() {
    if (_object) {
      _renderer->destroy(_object);
    }
  }
};
} // namespace rendering
} // namespace marlon

#endif