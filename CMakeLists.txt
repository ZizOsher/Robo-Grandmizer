cmake_minimum_required(VERSION 3.5)
project(client)

set(PLAYERC++_INCLUDE_DIRS "/usr/local/include/player-3.0")
set(PLAYERC++_LIBRARIES "-lplayerc++")
set(TIRPC_INCLUDE_DIRS "/usr/include/tirpc")

find_package(OpenCV REQUIRED)

file(GLOB SOURCES "*.cpp")

add_executable(client ${SOURCES})

# Add the OpenCV and PLAYERC++ header directories to the compiler's header search path.
target_include_directories(client PRIVATE ${OpenCV_INCLUDE_DIRS} ${PLAYERC++_INCLUDE_DIRS} ${TIRPC_INCLUDE_DIRS})

# Link the simple_client target against the OpenCV and PLAYERC++ libraries.
target_link_libraries(client ${OpenCV_LIBS} ${PLAYERC++_LIBRARIES})

