cmake_minimum_required(VERSION 3.16)
project(tyr_bambu_cloud_helper LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(Threads REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(Boost REQUIRED)

set(VENDOR "${CMAKE_CURRENT_SOURCE_DIR}/vendor")
set(HOST_SOURCES
    "${VENDOR}/tools/pjarczak_bambu_linux_host/LinuxPluginHost.cpp"
    "${VENDOR}/src/slic3r/Utils/PJarczakLinuxBridge/PJarczakLinuxBridgeConfig.cpp"
    "${VENDOR}/src/slic3r/Utils/PJarczakLinuxBridge/PJarczakLinuxSoBridgeRpcProtocol.cpp")
add_executable(tyr_bambu_cloud_host
    "${VENDOR}/tools/pjarczak_bambu_linux_host/main.cpp" ${HOST_SOURCES})
target_compile_definitions(tyr_bambu_cloud_host PRIVATE
    PJARCZAK_LINUX_BRIDGE_STANDALONE_HOST=1
    PJARCZAK_LINUX_BRIDGE_LIGHTWEIGHT_TASKS=1 _GLIBCXX_USE_CXX11_ABI=1)
target_include_directories(tyr_bambu_cloud_host PRIVATE "${VENDOR}" ${Boost_INCLUDE_DIRS})
target_link_libraries(tyr_bambu_cloud_host PRIVATE
    Threads::Threads OpenSSL::Crypto nlohmann_json::nlohmann_json ${CMAKE_DL_LIBS})
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_link_options(tyr_bambu_cloud_host PRIVATE -static-libstdc++ -static-libgcc)
endif()

include(CTest)
if(BUILD_TESTING)
    find_package(Python3 REQUIRED COMPONENTS Interpreter)
    add_library(tyr_fake_network SHARED tests/fake_network.cpp)
    target_include_directories(tyr_fake_network PRIVATE "${VENDOR}")
    target_compile_definitions(tyr_fake_network PRIVATE _GLIBCXX_USE_CXX11_ABI=1)
    target_link_libraries(tyr_fake_network PRIVATE Threads::Threads)
    set_target_properties(tyr_fake_network PROPERTIES OUTPUT_NAME bambu_networking)
    add_test(NAME tyr_cloud_helper_protocol COMMAND "${Python3_EXECUTABLE}"
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_host.py"
        "$<TARGET_FILE:tyr_bambu_cloud_host>" "$<TARGET_FILE:tyr_fake_network>")
    set_tests_properties(tyr_cloud_helper_protocol PROPERTIES TIMEOUT 60)
    add_executable(tyr_cloud_helper_frame_tests tests/test_frame.cpp
        "${VENDOR}/src/slic3r/Utils/PJarczakLinuxBridge/PJarczakLinuxSoBridgeRpcProtocol.cpp")
    target_include_directories(tyr_cloud_helper_frame_tests PRIVATE "${VENDOR}")
    target_link_libraries(tyr_cloud_helper_frame_tests PRIVATE nlohmann_json::nlohmann_json)
    add_test(NAME tyr_cloud_helper_frames COMMAND tyr_cloud_helper_frame_tests)
endif()
