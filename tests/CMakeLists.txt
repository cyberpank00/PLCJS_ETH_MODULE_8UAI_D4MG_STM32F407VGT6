cmake_minimum_required(VERSION 3.22)
project(plcjs_8uai_host_tests C)
set(CMAKE_C_STANDARD 11)
enable_testing()

add_executable(test_ai test_ai.c
    ../Application/ai/ai_math.c
    ../Application/keys/keys.c)
target_include_directories(test_ai PRIVATE
    ../Application/ai ../Application/keys ../Application/board)
target_compile_definitions(test_ai PRIVATE AI_HOST_BUILD)
target_compile_options(test_ai PRIVATE -Wall -Wextra -UNDEBUG)
target_link_libraries(test_ai PRIVATE m)
add_test(NAME ai COMMAND test_ai)
