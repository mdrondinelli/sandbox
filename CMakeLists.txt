cmake_minimum_required(VERSION 3.24 FATAL_ERROR)
project(Sandbox CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(/W4 /WX)
find_package(glfw3 3.3.8 REQUIRED)
find_package(Catch2 3 REQUIRED)
add_library(
  graphics
  "src/graphics/gl/camera.cpp"
  "src/graphics/gl/camera_instance.cpp"
  "src/graphics/gl/default_render_target.cpp"
  "src/graphics/gl/graphics.cpp"
  "src/graphics/gl/material.cpp"
  "src/graphics/gl/mesh.cpp"
  "src/graphics/gl/scene.cpp"
  "src/graphics/gl/scene_diff.cpp"
  "src/graphics/gl/scene_node.cpp"
  "src/graphics/gl/surface.cpp"
  "src/graphics/gl/surface_instance.cpp"
  "src/graphics/gl/unique_buffer.cpp"
  "src/graphics/gl/unique_shader.cpp"
  "src/graphics/gl/unique_shader_program.cpp"
  "src/graphics/gl/unique_vertex_array.cpp")
set(
  CLIENT_SOURCES
  "src/client/glad/glad.cpp" 
  "src/client/glfw/instance.cpp"
  "src/client/glfw/window.cpp"
  "src/shared/math/mat.cpp"
  "src/shared/math/quat.cpp"
  "src/shared/math/vec.cpp")
add_executable(
  client ${CLIENT_SOURCES} "src/client/main.cpp")
add_executable(client_tests ${CLIENT_SOURCES})
target_compile_definitions(client PRIVATE CATCH_CONFIG_DISABLE)
target_include_directories(graphics PRIVATE include)
target_include_directories(client PRIVATE include)
target_include_directories(client_tests PRIVATE include)
target_link_libraries(client glfw graphics)
target_link_libraries(client_tests glfw Catch2::Catch2WithMain)