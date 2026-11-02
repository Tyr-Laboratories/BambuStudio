# TODO Add individual tests as executables in separate directories
# add_subirectory(<testcase>)

set(TEST_DATA_DIR ${CMAKE_CURRENT_SOURCE_DIR}/data)
# Test runners are command-line executables even when the application is a bundle.
set(CMAKE_MACOSX_BUNDLE OFF)
file(TO_NATIVE_PATH "${TEST_DATA_DIR}" TEST_DATA_DIR)

add_library(Catch2 INTERFACE)
list (APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/modules/Catch2)
target_include_directories(Catch2 INTERFACE ${CMAKE_CURRENT_LIST_DIR})
add_library(Catch2::Catch2 ALIAS Catch2)
if (APPLE)
	# OSX builds targeting OSX 10.9 do not support new std::uncought_exception()
	# see https://github.com/catchorg/Catch2/issues/1218
	target_compile_definitions(Catch2 INTERFACE -DCATCH_CONFIG_NO_CPP17_UNCAUGHT_EXCEPTIONS)
endif()
include(Catch)

set(CATCH_EXTRA_ARGS "" CACHE STRING "Extra arguments for catch2 test suites.")

add_library(test_common INTERFACE)
target_compile_definitions(test_common INTERFACE TEST_DATA_DIR=R"\(${TEST_DATA_DIR}\)" CATCH_CONFIG_FAST_COMPILE)
target_link_libraries(test_common INTERFACE Catch2::Catch2)

if (APPLE)
    target_link_libraries(test_common INTERFACE "-liconv -framework IOKit" "-framework CoreFoundation" -lc++)
endif()

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# libslic3r's project loader and logging backend use these utility functions.
# Link their production implementations into headless slicing tests as well.
add_library(slicer_test_support STATIC
    slicer_test_support.cpp
    ${PROJECT_SOURCE_DIR}/src/slic3r/Utils/Http.cpp
    ${PROJECT_SOURCE_DIR}/src/slic3r/Utils/BBLUtil.cpp
)
target_link_libraries(slicer_test_support PUBLIC libslic3r libcurl OpenSSL::SSL OpenSSL::Crypto)
set_property(TARGET slicer_test_support PROPERTY FOLDER "tests")

add_subdirectory(libnest2d)
add_subdirectory(libslic3r)
add_subdirectory(slic3rutils)
add_subdirectory(fff_print)
add_subdirectory(sla_print)
add_subdirectory(fila_manager)
add_subdirectory(filament_group)
if (SLIC3R_GUI)
    add_subdirectory(gui_jobs)
endif ()
add_subdirectory(cpp17 EXCLUDE_FROM_ALL)    # does not have to be built all the time
# add_subdirectory(example)
