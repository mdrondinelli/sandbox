#include <iostream>
#include <vector>

#include <glad/glad.h>

#include <GLFW/glfw3.h>

#include "../graphics/gl/graphics.h"
#include "glfw_instance.h"
#include "glfw_window.h"

namespace client = marlon::client;
namespace graphics = marlon::graphics;

client::Glfw_unique_window_ptr create_window() {
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
  return client::make_glfw_unique_window(1024, 768, "title");
}

std::unique_ptr<graphics::Gl_graphics> create_graphics(GLFWwindow *window) {
  glfwMakeContextCurrent(window);
  if (!gladLoadGLLoader([](char const *procname) {
        return static_cast<void *>(glfwGetProcAddress(procname));
      })) {
    throw std::runtime_error{"Failed to initialize OpenGL"};
  }
  return std::make_unique<graphics::Gl_graphics>();
}

void tick(float dt) {

}

void run_game_loop(GLFWwindow *window) {
  auto const tick_rate = 1.0;
  auto const tick_duration = 1.0 / tick_rate;
  auto previous_time = glfwGetTime();
  auto accumulator = 0.0;
  for (;;) {
    glfwPollEvents();
    if (glfwWindowShouldClose(window)) {
      break;
    }
    auto const current_time = glfwGetTime();
    auto const elapsed_time = current_time - previous_time;
    previous_time = current_time;
    accumulator += elapsed_time;
    while (accumulator >= tick_duration) {
      accumulator -= tick_duration;
      tick(tick_duration);
    }
    glfwSwapBuffers(window);
  }
}

int main() {
  client::Glfw_shared_instance const glfw;
  auto const window = create_window();
  auto const graphics = create_graphics(window.get());
  run_game_loop(window.get());
}