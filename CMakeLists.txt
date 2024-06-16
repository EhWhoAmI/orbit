cmake_minimum_required(VERSION 3.13)

project(Orbit)

find_package(glm REQUIRED)
find_package(Entt REQUIRED)

add_library(orbit OBJECT src/orbit.cpp include/orbit.h include/units.h include/coordinates.h include/body.h)
target_include_directories(orbit PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(orbit glm::glm EnTT::EnTT)
