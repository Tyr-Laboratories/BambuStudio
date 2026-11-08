#ifndef SLIC3R_TEST_UTILS
#define SLIC3R_TEST_UTILS

#include <libslic3r/TriangleMesh.hpp>
#include <libslic3r/Format/OBJ.hpp>
#include <stdexcept>

#if defined(WIN32) || defined(_WIN32)
#define PATH_SEPARATOR R"(\)"
#else
#define PATH_SEPARATOR R"(/)"
#endif

inline Slic3r::TriangleMesh load_model(const std::string &obj_filename)
{
    Slic3r::TriangleMesh mesh;
    auto fpath = TEST_DATA_DIR PATH_SEPARATOR + obj_filename;
    Slic3r::ObjInfo obj_info;
    std::string message;
    if (!Slic3r::load_obj(fpath.c_str(), &mesh, obj_info, message))
        throw std::runtime_error("Could not load test model " + fpath + ": " + message);
    return mesh;
}

#endif // SLIC3R_TEST_UTILS
