// #ifndef MARLON_GRAPHICS_GL_SURFACE_H
// #define MARLON_GRAPHICS_GL_SURFACE_H

// #include "../surface.h"
// #include "mesh.h"

// namespace marlon {
// namespace graphics {
// class Gl_surface : public Surface {
//   friend class Gl_scene;

// public:
//   class Impl {
//   public:
//     explicit Impl(Surface_create_info const &create_info) noexcept;

//     Material const &get_material() const noexcept { return _material; }

//     Gl_mesh *get_mesh() const noexcept { return _mesh; }

//   private:
//     Material _material;
//     Gl_mesh *_mesh;
//   };

//   explicit Gl_surface(Surface_create_info const &create_info) noexcept;

// private:
//   Impl _impl;
// };
// } // namespace graphics
// } // namespace marlon

// #endif