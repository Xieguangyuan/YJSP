cmake_minimum_required(VERSION 3.0.0)
project(YJSP VERSION 0.1.0)
add_subdirectory(thirdParty/RaspberryPiRC/RPiIBus)
add_executable(YJSP main.cpp)
target_link_libraries(YJSP Ibus)

