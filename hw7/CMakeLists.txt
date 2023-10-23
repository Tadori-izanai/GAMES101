cmake_minimum_required(VERSION 3.10)
project(RayTracing)

set(CMAKE_CXX_STANDARD 17)

add_executable(RayTracing main.cpp Object.hpp Vector.cpp Vector.hpp Sphere.hpp global.hpp Triangle.hpp Scene.cpp
        Scene.hpp Light.hpp AreaLight.hpp BVH.cpp BVH.hpp Bounds3.hpp Ray.hpp Material.hpp Intersection.hpp
        Renderer.cpp Renderer.hpp)

target_compile_options(RayTracing PRIVATE -O2)

find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    message("OPENMP FOUND")
    target_link_libraries(RayTracing PUBLIC OpenMP::OpenMP_CXX)
endif()
