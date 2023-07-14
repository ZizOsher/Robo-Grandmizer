cmake_minimum_required(VERSION 3.5)
project(simple_client)

set(PLAYERC++_INCLUDE_DIRS "/usr/local/include/player-3.0")
set(PLAYERC++_LIBRARIES "-lplayerc++")
set(TIRPC_INCLUDE_DIRS "/usr/include/tirpc")

find_package(OpenCV REQUIRED)

file(GLOB SOURCES "*.cpp")

add_executable(simple_client ${SOURCES})
#add_executable(simple_client simple_client.cpp)

# Add the OpenCV and PLAYERC++ header directories to the compiler's header search path.
target_include_directories(simple_client PRIVATE ${OpenCV_INCLUDE_DIRS} ${PLAYERC++_INCLUDE_DIRS} ${TIRPC_INCLUDE_DIRS})

# Link the simple_client target against the OpenCV and PLAYERC++ libraries.
target_link_libraries(simple_client ${OpenCV_LIBS} ${PLAYERC++_LIBRARIES})

