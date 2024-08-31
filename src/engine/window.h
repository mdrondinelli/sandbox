#ifndef MARLON_ENGINE_WINDOW_H
#define MARLON_ENGINE_WINDOW_H

#include <optional>

#include <graphics/gl/window.h>
#include <math/vec.h>

#include "glfw_window.h"

namespace marlon {
namespace engine {
enum class Key {
  k_unknown = -1,
  k_space = 32,
  k_apostrophe = 39,
  k_comma = 44,
  k_minus = 45,
  k_period = 46,
  k_slash = 47,
  k_0 = 48,
  k_1 = 49,
  k_2 = 50,
  k_3 = 51,
  k_4 = 52,
  k_5 = 53,
  k_6 = 54,
  k_7 = 55,
  k_8 = 56,
  k_9 = 57,
  k_semicolon = 59,
  k_equal = 61,
  k_a = 65,
  k_b = 66,
  k_c = 67,
  k_d = 68,
  k_e = 69,
  k_f = 70,
  k_g = 71,
  k_h = 72,
  k_i = 73,
  k_j = 74,
  k_k = 75,
  k_l = 76,
  k_m = 77,
  k_n = 78,
  k_o = 79,
  k_p = 80,
  k_q = 81,
  k_r = 82,
  k_s = 83,
  k_t = 84,
  k_u = 85,
  k_v = 86,
  k_w = 87,
  k_x = 88,
  k_y = 89,
  k_z = 90,
  k_left_bracket = 91,
  k_backslash = 92,
  k_right_bracket = 93,
  k_grave_accent = 96,
  k_world_1 = 161,
  k_world_2 = 162,
  k_escape = 256,
  k_enter = 257,
  k_tab = 258,
  k_backspace = 259,
  k_insert = 260,
  k_delete = 261,
  k_right = 262,
  k_left = 263,
  k_down = 264,
  k_up = 265,
  k_page_up = 266,
  k_page_down = 267,
  k_home = 268,
  k_end = 269,
  k_caps_lock = 280,
  k_scroll_lock = 281,
  k_num_lock = 282,
  k_print_screen = 283,
  k_pause = 284,
  k_f1 = 290,
  k_f2 = 291,
  k_f3 = 292,
  k_f4 = 293,
  k_f5 = 294,
  k_f6 = 295,
  k_f7 = 296,
  k_f8 = 297,
  k_f9 = 298,
  k_f10 = 299,
  k_f11 = 300,
  k_f12 = 301,
  k_f13 = 302,
  k_f14 = 303,
  k_f15 = 304,
  k_f16 = 305,
  k_f17 = 306,
  k_f18 = 307,
  k_f19 = 308,
  k_f20 = 309,
  k_f21 = 310,
  k_f22 = 311,
  k_f23 = 312,
  k_f24 = 313,
  k_f25 = 314,
  k_numpad_0 = 320,
  k_numpad_1 = 321,
  k_numpad_2 = 322,
  k_numpad_3 = 323,
  k_numpad_4 = 324,
  k_numpad_5 = 325,
  k_numpad_6 = 326,
  k_numpad_7 = 327,
  k_numpad_8 = 328,
  k_numpad_9 = 329,
  k_numpad_decimal = 330,
  k_numpad_divide = 331,
  k_numpad_multiply = 332,
  k_numpad_subtract = 333,
  k_numpad_add = 334,
  k_numpad_enter = 335,
  k_numpad_equal = 336,
  k_left_shift = 340,
  k_left_control = 341,
  k_left_alt = 342,
  k_left_super = 343,
  k_right_shift = 344,
  k_right_control = 345,
  k_right_alt = 346,
  k_right_super = 347,
  k_menu = 348,
  k_last = k_menu,
};

enum class Mouse_button {
  mb_1 = 0,
  mb_2 = 1,
  mb_3 = 2,
  mb_4 = 3,
  mb_5 = 4,
  mb_6 = 5,
  mb_7 = 6,
  mb_8 = 7,
  mb_last = mb_8,
  mb_left = mb_1,
  mb_right = mb_2,
  mb_middle = mb_3,
};

enum class Cursor_mode { normal, hidden, disabled };

struct Window_create_info {
  const char *title;
  math::Vec2i extents;
  bool full_screen;
};

// Member functions, including constructor, must be called from the main thread
class Window : public graphics::gl::Window {
public:
  explicit Window(Window_create_info const &create_info);

  Window(Window const &other) = delete;

  Window &operator=(Window const &other) = delete;

  bool should_close() const noexcept;

  math::Vec2i get_framebuffer_extents() const noexcept final;

  bool is_key_pressed(Key key) const noexcept;

  bool is_mouse_button_pressed(Mouse_button mouse_button) const noexcept;

  math::Vec2d get_cursor_position() const noexcept;

  math::Vec2d get_delta_cursor_position() const noexcept;

  Cursor_mode get_cursor_mode() const noexcept;

  void set_cursor_mode(Cursor_mode mode) noexcept;

private:
  friend class App;

  GLFWwindow *get_glfw_window() noexcept;

  void pre_input() noexcept;

  Unique_glfw_window_ptr _glfw_window;
  std::optional<math::Vec2d> _cursor_position;
  math::Vec2d _delta_cursor_position{math::Vec2d::zero()};
};
} // namespace engine
} // namespace marlon

#endif
