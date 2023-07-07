cmake_minimum_required(VERSION 3.24 FATAL_ERROR)
project(Sandbox CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(/W4 /WX)
find_package(glfw3 3.3.8 REQUIRED)
find_package(Catch2 3 REQUIRED)
set(
  CLIENT_SOURCES
  "src/client/glad/glad.cpp" 
  "src/client/glfw/instance.cpp"
  "src/client/glfw/window.cpp"
  "src/client/rendering/gl/camera.cpp"
  "src/client/rendering/gl/camera_instance.cpp"
  "src/client/rendering/gl/default_render_destination.cpp"
  "src/client/rendering/gl/material.cpp"
  "src/client/rendering/gl/mesh.cpp"
  "src/client/rendering/gl/render.cpp"
  "src/client/rendering/gl/renderer.cpp"
  "src/client/rendering/gl/scene.cpp"
  "src/client/rendering/gl/scene_node.cpp"
  "src/client/rendering/gl/surface.cpp"
  "src/client/rendering/gl/surface_instance.cpp"
  "src/client/rendering/gl/unique_buffer.cpp"
  "src/client/rendering/gl/unique_shader.cpp"
  "src/client/rendering/gl/unique_vertex_array.cpp"
  "src/shared/math/quat.cpp"
  "src/shared/math/vec.cpp")
add_executable(
  client ${CLIENT_SOURCES} "src/client/main.cpp")
add_executable(client_tests ${CLIENT_SOURCES})
target_compile_definitions(client PRIVATE CATCH_CONFIG_DISABLE=1)
target_include_directories(client PRIVATE include)
target_include_directories(client_tests PRIVATE include)
target_link_libraries(client glfw)
target_link_libraries(client_tests glfw Catch2::Catch2WithMain)