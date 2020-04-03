cmake_minimum_required(VERSION 3.5.1)

set (LIBCARLA /home/praveen/workspace/libcarla)

set(CMAKE_C_COMPILER /usr/bin/clang-8)
set(CMAKE_CXX_COMPILER /usr/bin/clang++-8)
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -std=c++14 -O3 -g -fsanitize=address -fno-omit-frame-pointer -fPIC")

include_directories(${LIBCARLA}/include)
include_directories(${LIBCARLA}/include/system)

link_directories(${LIBCARLA}/lib)

file(GLOB SOURCES "spawn_npc.cpp")

add_executable(tm_test ${SOURCES})

find_package(Threads)
target_link_libraries(tm_test -lcarla_client -lrpc -lDetour -lDetourCrowd -fsanitize=address)
