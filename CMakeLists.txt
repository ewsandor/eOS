cmake_minimum_required(VERSION 3.22)

project(eOS 
        DESCRIPTION "Ed's Operating System"
        LANGUAGES ASM CXX
        HOMEPAGE_URL "https://git.sandorlaboratories.com/edward/eos")

set(PROJECT_VERSION 0.0.1-dev)

set(CMAKE_C_COMPILER   arm-none-eabi-gcc)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_ arm-none-eabi-gcc)
add_compile_options(-mcpu=arm1176jzf-s -fpic -ffreestanding -Wall -Wextra -Werror -O2)
add_link_options(-T linker.ld)

add_library(boot OBJECT boot.S)

ADD_EXECUTABLE(kernel kernel.c)

target_link_libraries(kernel PRIVATE boot)