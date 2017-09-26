PROJECT(GUI_APP)
set(TARGET gui_app)

add_executable(${TARGET} gui_app.cpp ${GUI_APP_SRCS})

target_include_directories(${TARGET} PRIVATE ${BULLET_INCLUDE_DIRS} ${GUI_APP_INCLUDE_DIRS})
target_link_libraries(${TARGET} ${GUI_APP_LIBRARIES} BulletDynamics BulletCollision LinearMath)