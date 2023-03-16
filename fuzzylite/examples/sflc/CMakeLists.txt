cmake_minimum_required(VERSION 2.8.8)

project(FuzzyLiteDemo CXX)

#we add the sources
set(sources src/main.cpp)

#we create the binary
add_executable(binary ${sources})

target_include_directories(binary PUBLIC "/home/pgarcia/fuzzylite/fuzzylite")

target_link_directories(binary PUBLIC "/home/pgarcia/fuzzylite/fuzzylite/release/bin")

target_link_libraries(binary fuzzylite)