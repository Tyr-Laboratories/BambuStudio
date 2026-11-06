get_filename_component(_TEST_NAME ${CMAKE_CURRENT_LIST_DIR} NAME)
add_executable(${_TEST_NAME}_tests
    ${_TEST_NAME}_tests_main.cpp
    helio_retry_policy_tests.cpp
    helio_support_data_tests.cpp
    helio_recent_runs_tests.cpp
    )

target_link_libraries(${_TEST_NAME}_tests test_common libslic3r_gui libslic3r)
# These tests include GUI utility headers, so they need the same wxWidgets
# header configuration as their production dependency.
get_directory_property(_utils_wx_includes DIRECTORY ${PROJECT_SOURCE_DIR}/src DEFINITION wxWidgets_INCLUDE_DIRS)
get_directory_property(_utils_wx_definitions DIRECTORY ${PROJECT_SOURCE_DIR}/src DEFINITION wxWidgets_DEFINITIONS)
target_include_directories(${_TEST_NAME}_tests PRIVATE ${_utils_wx_includes})
target_compile_definitions(${_TEST_NAME}_tests PRIVATE ${_utils_wx_definitions})
if (APPLE)
    target_link_libraries(${_TEST_NAME}_tests
        "-framework AVFoundation" "-framework AVKit" "-framework CoreMedia" "-framework VideoToolbox")
endif()
if (MSVC)
    target_link_libraries(${_TEST_NAME}_tests Setupapi.lib)
endif ()
set_property(TARGET ${_TEST_NAME}_tests PROPERTY FOLDER "tests")

if (WIN32)
    # See tests/libslic3r/CMakeLists.txt for the full rationale; the legacy
    # 1-arg invocation no longer matches bambustudio_copy_dlls's 4-arg signature.
    # bambustudio_copy_dlls(${_TEST_NAME}_tests <config> <postfix> <out>)
endif()

# catch_discover_tests(${_TEST_NAME}_tests TEST_PREFIX "${_TEST_NAME}: ")
add_test(${_TEST_NAME}_tests ${_TEST_NAME}_tests "${CATCH_EXTRA_ARGS} exclude:[NotWorking]")
