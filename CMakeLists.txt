cmake_minimum_required(VERSION 3.24 FATAL_ERROR)
project(Sandbox CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if (MSVC)
  add_compile_options(/W4 /WX)
else()
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()
find_package(glfw3 3.3.8 REQUIRED)
find_package(Catch2 3 REQUIRED)
add_executable(
  math_tests
  "src/math/mat.cpp"
  "src/math/quat.cpp"
  "src/math/vec.cpp")
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
  "src/graphics/gl/texture.cpp"
  "src/graphics/gl/unique_buffer_handle.cpp"
  "src/graphics/gl/unique_shader_handle.cpp"
  "src/graphics/gl/unique_shader_program_handle.cpp"
  "src/graphics/gl/unique_texture_handle.cpp"
  "src/graphics/gl/unique_vertex_array_handle.cpp"
  "src/graphics/glad/glad.cpp")
add_library(
  physics
  "src/physics/ball.cpp"
  "src/physics/box.cpp"
  "src/physics/half_space.cpp"
  "src/physics/particle.cpp"
  "src/physics/space.cpp")
set(
  CLIENT_SOURCES
  "src/client/entity.cpp"
  "src/client/glfw_instance.cpp"
  "src/client/glfw_window.cpp"
  "src/client/static_prop_entity.cpp"
  "src/client/test_entity.cpp")
add_executable(
  client ${CLIENT_SOURCES} "src/client/main.cpp")
add_executable(client_tests ${CLIENT_SOURCES})
set_property(TARGET client PROPERTY MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
set_property(TARGET graphics PROPERTY MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
set_property(TARGET physics PROPERTY MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
target_compile_definitions(client PRIVATE CATCH_CONFIG_DISABLE)
# target_compile_definitions(graphics PRIVATE)
# target_compile_definitions(physics PRIVATE)
target_include_directories(client PRIVATE include)
target_include_directories(client_tests PRIVATE include)
target_include_directories(graphics PRIVATE include)
target_include_directories(physics PRIVATE include)
target_link_libraries(client glfw graphics physics)
target_link_libraries(client_tests glfw Catch2::Catch2WithMain)
target_link_libraries(math_tests Catch2::Catch2WithMain)
target_link_libraries(graphics ${CMAKE_SOURCE_DIR}/lib/ktx.lib)