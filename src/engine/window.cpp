#include "window.h"

#include <GLFW/glfw3.h>

using namespace marlon::math;

namespace marlon {
namespace engine {
void Window::set_current_context_swap_interval(int interval) {
  glfwSwapInterval(interval);
}

Window::Window(Window_create_info const &create_info)
    : _glfw_window{[&]() {
        glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
        glfwWindowHint(GLFW_SRGB_CAPABLE, GLFW_TRUE);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
        return make_unique_glfw_window(create_info.extents.x,
                                       create_info.extents.y,
                                       create_info.title,
                                       create_info.full_screen ? glfwGetPrimaryMonitor() : nullptr);
      }()} {
  if (glfwRawMouseMotionSupported()) {
    glfwSetInputMode(_glfw_window.get(), GLFW_RAW_MOUSE_MOTION, GLFW_TRUE);
  }
  glfwSetWindowUserPointer(_glfw_window.get(), this);
  glfwSetCursorPosCallback(_glfw_window.get(), [](GLFWwindow *window, double x, double y) {
    auto const self = static_cast<Window *>(glfwGetWindowUserPointer(window));
    if (self->_cursor_position) {
      self->_delta_cursor_position += math::Vec2d{x - self->_cursor_position->x, y - self->_cursor_position->y};
    }
    self->_cursor_position = math::Vec2d{x, y};
  });
}

void Window::make_context_current() noexcept {
  glfwMakeContextCurrent(_glfw_window.get());
}

void Window::swap_buffers() noexcept {
  glfwSwapBuffers(_glfw_window.get());
}

Vec2i Window::get_framebuffer_extents() const noexcept {
  auto result = Vec2i{};
  glfwGetFramebufferSize(_glfw_window.get(), &result[0], &result[1]);
  return result;
}

bool Window::should_close() const noexcept {
  return glfwWindowShouldClose(_glfw_window.get());
}

bool Window::is_key_pressed(Key key) const noexcept {
  return glfwGetKey(_glfw_window.get(), static_cast<int>(key)) == GLFW_PRESS;
}

bool Window::is_mouse_button_pressed(Mouse_button mouse_button) const noexcept {
  return glfwGetMouseButton(_glfw_window.get(), static_cast<int>(mouse_button)) == GLFW_PRESS;
}

math::Vec2d Window::get_cursor_position() const noexcept {
  auto result = math::Vec2d{};
  glfwGetCursorPos(_glfw_window.get(), &result[0], &result[1]);
  return result;
}

math::Vec2d Window::get_delta_cursor_position() const noexcept {
  return _delta_cursor_position;
}

Cursor_mode Window::get_cursor_mode() const noexcept {
  auto const value = glfwGetInputMode(_glfw_window.get(), GLFW_CURSOR);
  switch (value) {
  case GLFW_CURSOR_NORMAL:
    return Cursor_mode::normal;
  case GLFW_CURSOR_HIDDEN:
    return Cursor_mode::hidden;
  case GLFW_CURSOR_DISABLED:
    return Cursor_mode::disabled;
  default:
    math::unreachable();
  }
}

void Window::set_cursor_mode(Cursor_mode mode) noexcept {
  int value;
  switch (mode) {
  case Cursor_mode::normal:
    value = GLFW_CURSOR_NORMAL;
    break;
  case Cursor_mode::hidden:
    value = GLFW_CURSOR_HIDDEN;
    break;
  case Cursor_mode::disabled:
    value = GLFW_CURSOR_DISABLED;
    break;
  default:
    math::unreachable();
  }
  glfwSetInputMode(_glfw_window.get(), GLFW_CURSOR, value);
}

void Window::pre_input() noexcept {
  _delta_cursor_position = math::Vec2d::zero();
}
} // namespace engine
} // namespace marlon
