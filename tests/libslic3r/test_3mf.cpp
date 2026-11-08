#include <catch2/catch.hpp>

#include "libslic3r/Model.hpp"
#include "libslic3r/Format/3mf.hpp"
#include "libslic3r/Format/STL.hpp"

#include <boost/filesystem/operations.hpp>

using namespace Slic3r;

SCENARIO("Reading 3mf file", "[3mf]") {
    GIVEN("umlauts in the path of the file") {
        Model model;
        WHEN("3mf model is read") {
        	std::string path = std::string(TEST_DATA_DIR) + "/test_3mf/Geräte/Büchse.3mf";
        	DynamicPrintConfig config;
            ConfigSubstitutionContext ctxt{ ForwardCompatibilitySubstitutionRule::Disable };
            bool ret = load_3mf(path.c_str(), config, ctxt, &model, false);
            THEN("load should succeed") {
                REQUIRE(ret);
            }
        }
    }
}

SCENARIO("Export+Import geometry to/from 3mf file cycle", "[3mf]") {
    GIVEN("world vertices coordinates before save") {
        // load a model from stl file
        Model src_model;
        std::string src_file = std::string(TEST_DATA_DIR) + "/test_3mf/Prusa.stl";
        load_stl(src_file.c_str(), &src_model);
        src_model.add_default_instances();

        ModelObject* src_object = src_model.objects.front();

        // apply generic transformation to the 1st volume
        Geometry::Transformation src_volume_transform;
        src_volume_transform.set_offset({ 10.0, 20.0, 0.0 });
        src_volume_transform.set_rotation({ Geometry::deg2rad(25.0), Geometry::deg2rad(35.0), Geometry::deg2rad(45.0) });
        src_volume_transform.set_scaling_factor({ 1.1, 1.2, 1.3 });
        src_volume_transform.set_mirror({ -1.0, 1.0, -1.0 });
        src_object->volumes.front()->set_transformation(src_volume_transform);

        // apply generic transformation to the 1st instance
        Geometry::Transformation src_instance_transform;
        src_instance_transform.set_offset({ 5.0, 10.0, 0.0 });
        src_instance_transform.set_rotation({ Geometry::deg2rad(12.0), Geometry::deg2rad(13.0), Geometry::deg2rad(14.0) });
        src_instance_transform.set_scaling_factor({ 0.9, 0.8, 0.7 });
        src_instance_transform.set_mirror({ 1.0, -1.0, -1.0 });
        src_object->instances.front()->set_transformation(src_instance_transform);

        WHEN("model is saved+loaded to/from 3mf file") {
            // save the model to 3mf file
            std::string test_file = std::string(TEST_DATA_DIR) + "/test_3mf/prusa.3mf";
            store_3mf(test_file.c_str(), &src_model, nullptr, false);

            // load back the model from the 3mf file
            Model dst_model;
            DynamicPrintConfig dst_config;
            {
                ConfigSubstitutionContext ctxt{ ForwardCompatibilitySubstitutionRule::Disable };
                load_3mf(test_file.c_str(), dst_config, ctxt, &dst_model, false);
            }
            boost::filesystem::remove(test_file);

            // compare meshes
            TriangleMesh src_mesh = src_model.mesh();
            TriangleMesh dst_mesh = dst_model.mesh();

            bool res = src_mesh.its.vertices.size() == dst_mesh.its.vertices.size();
            if (res) {
                for (size_t i = 0; i < dst_mesh.its.vertices.size(); ++i) {
                    res &= dst_mesh.its.vertices[i].isApprox(src_mesh.its.vertices[i]);
                }
            }
            THEN("world vertices coordinates after load match") {
                REQUIRE(res);
            }
        }
    }
}

SCENARIO("2D convex hull of sinking object", "[3mf]") {
    GIVEN("model") {
        // load a model
        Model model;
        std::string src_file = std::string(TEST_DATA_DIR) + "/test_3mf/Prusa.stl";
        REQUIRE(load_stl(src_file.c_str(), &model));
        REQUIRE(model.objects.size() == 1);
        model.add_default_instances();

        WHEN("model is rotated, scaled and set as sinking") {
            ModelObject* object = model.objects.front();
            object->center_around_origin(false);

            // set instance's attitude so that it is rotated, scaled and sinking
            ModelInstance* instance = object->instances.front();
            instance->set_rotation(Vec3d(-M_PI / 4.0, 0., 0.));
            instance->set_offset(Vec3d::Zero());
            instance->set_scaling_factor({ 2.0, 2.0, 2.0 });

            const BoundingBoxf3 bounds = object->instance_bounding_box(*instance);
            REQUIRE(bounds.min.z() < 0.0);
            REQUIRE(bounds.max.z() > 0.0);
            REQUIRE(object->volumes.size() == 1);
            const ModelVolume* volume = object->volumes.front();
            const Transform3d transform = instance->get_transformation().get_matrix();

            // Keep reference coordinates in millimeters so changes to the internal
            // fixed-point scale do not change this geometry regression.
            const auto require_hull_matches = [](const Polygon& hull, const std::vector<Vec2d>& expected) {
                REQUIRE(hull.points.size() == expected.size());
                for (size_t i = 0; i < expected.size(); ++i) {
                    CAPTURE(i);
                    // Allow 1um error due to floating point rounding.
                    CHECK(unscale<double>(hull.points[i].x()) == Approx(expected[i].x()).epsilon(0.0).margin(0.001));
                    CHECK(unscale<double>(hull.points[i].y()) == Approx(expected[i].y()).epsilon(0.0).margin(0.001));
                }
            };

            THEN("the above-bed mesh hull matches the sinking reference") {
                // Arrangement now projects the complete 3D hull. Exercise the
                // explicit clipping helper to retain the original sinking check.
                const Transform3f volume_transform = (transform * volume->get_matrix()).cast<float>();
                const Polygon hull = its_convex_hull_2d_above(volume->mesh().its, volume_transform, 0.0f);
                require_hull_matches(hull, {
                    { -91.501496, -15.914144 },
                    { 91.501496, -15.914144 },
                    { 91.501496, 0.004243 },
                    { 78.229680, 4.246883 },
                    { 56.898100, 4.246883 },
                    { -85.501496, 4.242641 },
                    { -91.501496, 0.004243 }
                });
            }

            THEN("the arrangement hull includes the complete transformed object") {
                require_hull_matches(object->convex_hull_2d(transform), {
                    { -91.501495, -15.914144 },
                    { 91.501495, -15.914144 },
                    { 91.501495, 13.792824 },
                    { 34.846497, 14.717717 },
                    { -85.501495, 13.917982 },
                    { -91.501495, 13.792824 }
                });
            }
        }
    }
}

