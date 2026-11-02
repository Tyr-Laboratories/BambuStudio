# Exercise the production legacy job dispatcher without the full GUI application,
# network agent, account configuration, or a printer.
add_executable(gui_job_tests
    test_job_dispatch.cpp
    ${PROJECT_SOURCE_DIR}/src/slic3r/GUI/Jobs/Job.cpp
)
get_directory_property(_job_wx_libraries DIRECTORY ${PROJECT_SOURCE_DIR}/src DEFINITION wxWidgets_LIBRARIES)
get_directory_property(_job_wx_includes DIRECTORY ${PROJECT_SOURCE_DIR}/src DEFINITION wxWidgets_INCLUDE_DIRS)
get_directory_property(_job_wx_definitions DIRECTORY ${PROJECT_SOURCE_DIR}/src DEFINITION wxWidgets_DEFINITIONS)
target_include_directories(gui_job_tests PRIVATE
    ${PROJECT_SOURCE_DIR}/src
    ${PROJECT_BINARY_DIR}/src/libslic3r
    ${_job_wx_includes}
)
target_compile_definitions(gui_job_tests PRIVATE ${_job_wx_definitions})
target_link_libraries(gui_job_tests PRIVATE test_common boost_libs ${_job_wx_libraries})
set_property(TARGET gui_job_tests PROPERTY FOLDER "tests")
add_test(NAME gui_job_tests COMMAND gui_job_tests ${CATCH_EXTRA_ARGS})

add_executable(official_sender_tests
    test_official_sender.cpp
    ${PROJECT_SOURCE_DIR}/src/slic3r/GUI/OfficialSender.cpp
)
target_include_directories(official_sender_tests PRIVATE ${PROJECT_SOURCE_DIR}/src)
target_link_libraries(official_sender_tests PRIVATE test_common)
set_property(TARGET official_sender_tests PROPERTY FOLDER "tests")
add_test(NAME official_sender_tests COMMAND official_sender_tests ${CATCH_EXTRA_ARGS})
