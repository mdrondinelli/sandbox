#include <iostream>
#include <vector>

#include <glad/glad.h>

#include <GLFW/glfw3.h>

#include "glfw/instance.h"
#include "glfw/window.h"

int main() {
  marlon::glfw::Instance glfw;
  // glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  const auto window = marlon::glfw::make_unique_window(800, 600, "title");
  glfwMakeContextCurrent(window.get());
  const auto loadproc = [](const char *procname) -> void * {
    return glfwGetProcAddress(procname);
  };
  if (!gladLoadGLLoader(loadproc)) {
    std::cerr << "Failed to initialize OpenGL" << std::endl;
    return -1;
  }
  glfwSwapInterval(1);
  while (!glfwWindowShouldClose(window.get())) {
    glClearColor(1.0, 0.0, 1.0, 1.0);
    glClear(GL_COLOR_BUFFER_BIT);
    glfwSwapBuffers(window.get());
    glfwWaitEvents();
  }
  return 0;
}