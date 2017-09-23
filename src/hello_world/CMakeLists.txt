cmake_minimum_required(VERSION 2.4.3)

PROJECT(HELLO_WORLD)
set(TARGET hello_world)

add_executable(${TARGET} hello_world.cpp)

target_include_directories(${TARGET} PRIVATE ${BULLET_INCLUDE_DIRS})
target_link_libraries(${TARGET} BulletDynamics BulletCollision LinearMath)