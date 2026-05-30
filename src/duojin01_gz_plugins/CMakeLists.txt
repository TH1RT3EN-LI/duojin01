cmake_minimum_required(VERSION 3.8)
project(duojin01_gz_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-sim8 REQUIRED)
find_package(gz-transport13 REQUIRED)

add_library(duojin01_suction_carry_system SHARED
  src/suction_carry_system.cpp
)

target_compile_features(duojin01_suction_carry_system PUBLIC cxx_std_17)

target_link_libraries(duojin01_suction_carry_system
  gz-plugin2::gz-plugin2
  gz-plugin2::register
  gz-sim8::gz-sim8
  gz-transport13::gz-transport13
)

install(
  TARGETS duojin01_suction_carry_system
  LIBRARY DESTINATION lib
)

ament_package()
