cmake_minimum_required(VERSION 2.4.3)

project(BULLET_PLAYGROUND)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# configure Bullet
set(BUILD_SHARED_LIBS ON CACHE BOOL "Build shared libraries")
set(BUILD_UNIT_TESTS OFF CACHE BOOL "Build unit tests")
set(BUILD_BULLET2_DEMOS OFF CACHE BOOL "Build Bullet 2 demos")
set(BUILD_CPU_DEMOS OFF CACHE BOOL "Build CPU demos")
set(BUILD_OPENGL3_DEMOS OFF CACHE BOOL "Build OpenGL 3 demos")
set(BUILD_PYBULLET OFF CACHE BOOL "Build pyBullet")
set(BULLET_INCLUDE_DIRS ${BULLET_PHYSICS_SOURCE_DIR}/src)
add_subdirectory(third_party/bullet3)

# Bullet GUI app
set(GUI_APP_INCLUDE_DIRS
	${BULLET_PHYSICS_SOURCE_DIR}/examples/Utils
	${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs/Glew
	${BULLET_PHYSICS_SOURCE_DIR}/examples/OpenGLWindow
	${BULLET_PHYSICS_SOURCE_DIR}/examples/ExampleBrowser)
set(GUI_APP_SRCS
	${BULLET_PHYSICS_SOURCE_DIR}/examples/Utils/b3Clock.cpp
	${BULLET_PHYSICS_SOURCE_DIR}/examples/ExampleBrowser/OpenGLGuiHelper.cpp
	${BULLET_PHYSICS_SOURCE_DIR}/examples/ExampleBrowser/GL_ShapeDrawer.cpp
	${BULLET_PHYSICS_SOURCE_DIR}/examples/ExampleBrowser/CollisionShape2TriangleMesh.cpp)
set(GUI_APP_LIBRARIES OpenGLWindow)
if (COMMAND cmake_policy)
   cmake_policy(SET CMP0003 NEW)
   if (POLICY CMP0042)
      # Enable MACOSX_RPATH by default.
      cmake_policy(SET CMP0042 NEW)
   endif()
endif()
ADD_DEFINITIONS(-DGLEW_STATIC)
add_subdirectory(${BULLET_PHYSICS_SOURCE_DIR}/examples/OpenGLWindow)

subdirs(src/hello_world src/gui_app)