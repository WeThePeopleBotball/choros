cmake_minimum_required(VERSION 3.14)
project(choros_lib LANGUAGES CXX)

# Create the static library from sources
add_library(choros STATIC
    lifecycle.cpp
    navigation.cpp
)

# Include headers for the library
target_include_directories(choros PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# Optionally define warnings or extra flags
target_compile_options(choros PRIVATE -Wall -Wextra -pedantic)

# Link pthread for std::thread and async support
find_package(Threads REQUIRED)
target_link_libraries(choros PUBLIC Threads::Threads)
