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
  "src/math/vec.cpp"
)
add_library(
  util
  "src/util/memory.cpp"
  "src/util/thread_pool.cpp"
)
add_executable(
  util_tests
  "src/util/hash_tests.cpp"
  "src/util/memory_tests.cpp"
  "src/util/bit_list_tests.cpp"
  "src/util/list_tests.cpp"
  "src/util/queue_tests.cpp"
  "src/util/set_tests.cpp"
  "src/util/map_tests.cpp"
)
add_library(
  physics
  "src/physics/world.cpp"
)
add_library(
  graphics
  "src/graphics/gl/default_render_target.cpp"
  "src/graphics/gl/graphics.cpp"
  "src/graphics/gl/render_stream.cpp"
  "src/graphics/gl/scene.cpp"
  "src/graphics/gl/surface_material.cpp"
  "src/graphics/gl/surface_mesh.cpp"
  "src/graphics/gl/texture.cpp"
  "src/graphics/gl/unique_buffer_handle.cpp"
  "src/graphics/gl/unique_shader_handle.cpp"
  "src/graphics/gl/unique_shader_program_handle.cpp"
  "src/graphics/gl/unique_texture_handle.cpp"
  "src/graphics/gl/unique_vertex_array_handle.cpp"
  "src/graphics/gl/wireframe_mesh.cpp"
  "src/graphics/glad/gl.cpp"
)
add_library(
  engine
  "src/engine/app.cpp"
  "src/engine/glfw_init_guard.cpp"
  "src/engine/glfw_window.cpp"
  "src/engine/window.cpp"
)
add_executable(
  client
  "src/client/dynamic_prop.cpp"
  "src/client/static_prop.cpp"
  "src/client/main.cpp"
)
set_target_properties(math_tests util util_tests graphics physics engine client PROPERTIES MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
target_include_directories(graphics PRIVATE include src)
target_include_directories(engine PRIVATE include)
target_include_directories(client PRIVATE include)
target_compile_definitions(client PRIVATE CATCH_CONFIG_DISABLE)
target_link_libraries(math_tests Catch2::Catch2WithMain)
target_link_libraries(util_tests util Catch2::Catch2WithMain)
target_link_libraries(physics util)
target_link_libraries(graphics util ${CMAKE_SOURCE_DIR}/lib/ktx.lib)
target_link_libraries(engine physics graphics glfw)
target_link_libraries(client engine)