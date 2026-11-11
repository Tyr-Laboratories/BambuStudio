get_filename_component(_TEST_NAME ${CMAKE_CURRENT_LIST_DIR} NAME)
add_executable(${_TEST_NAME}_tests 
	${_TEST_NAME}_tests.cpp
	test_data.cpp
	test_data.hpp
	test_extrusion_entity.cpp
	test_fill.cpp
	test_flow.cpp
	test_gcode.cpp
	test_gcodewriter.cpp
	test_gcodewriter_travel.cpp
	test_model.cpp
	test_print.cpp
	test_printgcode.cpp
	test_printobject.cpp
	test_skirt_brim.cpp
	test_support_material.cpp
	test_support_material_on_wipe_tower.cpp
	test_trianglemesh.cpp
	)
target_link_libraries(${_TEST_NAME}_tests test_common libslic3r slicer_test_support)
set_property(TARGET ${_TEST_NAME}_tests PROPERTY FOLDER "tests")

if (WIN32)
    # See tests/libslic3r/CMakeLists.txt for the full rationale; the legacy
    # 1-arg invocation no longer matches bambustudio_copy_dlls's 4-arg signature.
    # bambustudio_copy_dlls(${_TEST_NAME}_tests <config> <postfix> <out>)
endif()

# catch_discover_tests(${_TEST_NAME}_tests TEST_PREFIX "${_TEST_NAME}: ")
add_test(${_TEST_NAME}_tests ${_TEST_NAME}_tests ${CATCH_EXTRA_ARGS})

add_executable(support_material_on_wipe_tower_tests
	fff_print_tests.cpp
	test_data.cpp
	test_data.hpp
	test_support_material_on_wipe_tower.cpp
	)
target_link_libraries(support_material_on_wipe_tower_tests test_common libslic3r
	libcurl
	OpenSSL::SSL OpenSSL::Crypto
)
if (MSVC)
	target_link_options(support_material_on_wipe_tower_tests PRIVATE /FORCE:UNRESOLVED)
endif()
set_property(TARGET support_material_on_wipe_tower_tests PROPERTY FOLDER "tests")
add_test(support_material_on_wipe_tower_tests support_material_on_wipe_tower_tests ${CATCH_EXTRA_ARGS})

add_executable(periodic_modifier_tests
	fff_print_tests.cpp
	test_data.cpp
	test_data.hpp
	test_periodic_modifier.cpp
	)
target_link_libraries(periodic_modifier_tests test_common libslic3r
	libcurl
	OpenSSL::SSL OpenSSL::Crypto
)
if (MSVC)
	target_link_options(periodic_modifier_tests PRIVATE /FORCE:UNRESOLVED)
endif()
set_property(TARGET periodic_modifier_tests PROPERTY FOLDER "tests")
add_test(periodic_modifier_tests periodic_modifier_tests ${CATCH_EXTRA_ARGS})


add_executable(wall_painting_tests fff_print_tests.cpp test_wall_painting.cpp test_round_hole_detection.cpp)
target_link_libraries(wall_painting_tests test_common libslic3r slicer_test_support)
set_property(TARGET wall_painting_tests PROPERTY FOLDER "tests")
add_test(wall_painting_tests wall_painting_tests ${CATCH_EXTRA_ARGS})

add_executable(bridge_acceleration_tests fff_print_tests.cpp test_bridge_acceleration.cpp)
target_link_libraries(bridge_acceleration_tests test_common libslic3r slicer_test_support)
set_property(TARGET bridge_acceleration_tests PROPERTY FOLDER "tests")
add_test(bridge_acceleration_tests bridge_acceleration_tests ${CATCH_EXTRA_ARGS})

add_executable(overhang_wall_tests fff_print_tests.cpp test_overhang_walls.cpp test_slope_wall_config.cpp)
target_link_libraries(overhang_wall_tests test_common libslic3r slicer_test_support)
set_property(TARGET overhang_wall_tests PROPERTY FOLDER "tests")
add_test(overhang_wall_tests overhang_wall_tests ${CATCH_EXTRA_ARGS})

# Keep the relevant pre-existing model/3MF/slicing checks runnable even when
# unrelated legacy G-code tests target APIs that have since been removed.
add_executable(wall_painting_regression_tests
    fff_print_tests.cpp test_data.cpp test_model.cpp test_trianglemesh.cpp
    test_extrusion_entity.cpp test_flow.cpp test_printobject.cpp
    ../libslic3r/test_3mf.cpp
)
target_link_libraries(wall_painting_regression_tests test_common libslic3r slicer_test_support)
set_property(TARGET wall_painting_regression_tests PROPERTY FOLDER "tests")
add_test(wall_painting_regression_tests wall_painting_regression_tests ${CATCH_EXTRA_ARGS})

add_executable(bridge_controls_tests fff_print_tests.cpp test_bridge_controls.cpp test_bridge_controls_config.cpp test_bridge_controls_reslice.cpp test_bridge_stack_patterns.cpp test_bridge_controls_cache.cpp)
target_link_libraries(bridge_controls_tests test_common libslic3r slicer_test_support)
target_include_directories(bridge_controls_tests PRIVATE ${CMAKE_SOURCE_DIR}/src/libslic3r)
set_property(TARGET bridge_controls_tests PROPERTY FOLDER "tests")
add_test(bridge_controls_tests bridge_controls_tests ${CATCH_EXTRA_ARGS})

add_executable(bridge_aware_cubic_tests fff_print_tests.cpp test_bridge_aware_cubic.cpp)
target_link_libraries(bridge_aware_cubic_tests test_common libslic3r slicer_test_support)
set_property(TARGET bridge_aware_cubic_tests PROPERTY FOLDER "tests")
add_test(bridge_aware_cubic_tests bridge_aware_cubic_tests ${CATCH_EXTRA_ARGS})

add_executable(support_base_tests fff_print_tests.cpp test_support_base_painting.cpp test_support_base_slicing.cpp test_support_base_routes.cpp)
target_link_libraries(support_base_tests test_common libslic3r slicer_test_support)
set_property(TARGET support_base_tests PROPERTY FOLDER "tests")
add_test(support_base_tests support_base_tests ${CATCH_EXTRA_ARGS})

add_executable(organic_foundation_tests fff_print_tests.cpp test_organic_foundations.cpp test_organic_foundation_volumes.cpp test_organic_interface_phase.cpp test_organic_foundation_chord.cpp)
target_link_libraries(organic_foundation_tests test_common libslic3r slicer_test_support)
set_property(TARGET organic_foundation_tests PROPERTY FOLDER "tests")
add_test(organic_foundation_tests organic_foundation_tests ${CATCH_EXTRA_ARGS})

add_executable(benchmark_organic_support benchmark_organic_support.cpp)
target_link_libraries(benchmark_organic_support test_common libslic3r slicer_test_support)
set_property(TARGET benchmark_organic_support PROPERTY FOLDER "tests")

add_executable(parallel_plate_tests fff_print_tests.cpp test_parallel_plate_slicing.cpp)
target_link_libraries(parallel_plate_tests test_common libslic3r slicer_test_support)
target_include_directories(parallel_plate_tests PRIVATE ${CMAKE_SOURCE_DIR}/src/libslic3r)
set_property(TARGET parallel_plate_tests PROPERTY FOLDER "tests")
add_test(parallel_plate_tests parallel_plate_tests ${CATCH_EXTRA_ARGS})

add_executable(slicing_spatial_tests fff_print_tests.cpp test_slicing_spatial.cpp)
target_link_libraries(slicing_spatial_tests test_common libslic3r slicer_test_support)
set_property(TARGET slicing_spatial_tests PROPERTY FOLDER "tests")
add_test(slicing_spatial_tests slicing_spatial_tests ${CATCH_EXTRA_ARGS})
