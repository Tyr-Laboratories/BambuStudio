#include <catch2/catch.hpp>
#include "libslic3r/RoundHoleDetection.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/TriangleSelector.hpp"
#include "libslic3r/Model.hpp"
#include <boost/filesystem.hpp>
#include <numeric>
#include <random>

using namespace Slic3r;
using namespace Slic3r::WallPainting;

namespace {
// Closed annular part. Its first 2 * sides faces are the inward-facing bore;
// the remaining faces are its exterior, top and bottom surfaces.
indexed_triangle_set bore(unsigned sides = 64, double sweep = 2. * M_PI,
                          double top_radius = 2., bool blind = false)
{
    indexed_triangle_set mesh;
    const bool closed = std::abs(sweep - 2. * M_PI) < 1e-8;
    const unsigned count = closed ? sides : sides + 1;
    for (unsigned ring = 0; ring < 4; ++ring)
        for (unsigned i = 0; i < count; ++i) {
            const double a = sweep * i / sides;
            const double r = ring < 2 ? (ring == 0 ? 2. : top_radius) : 7.;
            mesh.vertices.emplace_back(float(r * cos(a)), float(r * sin(a)), ring % 2 ? 10.f : 0.f);
        }
    auto quad = [&](int a, int b, int c, int d) {
        mesh.indices.emplace_back(a,b,c);
        mesh.indices.emplace_back(a,c,d);
    };
    for (unsigned i = 0; i < sides; ++i) {
        const int j = (i + 1) % count;
        quad(i, count + i, count + j, j);
    }
    for (unsigned i = 0; i < sides; ++i) {
        const int j = (i + 1) % count;
        quad(2*count+i, 2*count+j, 3*count+j, 3*count+i);
        quad(count+i, 3*count+i, 3*count+j, count+j);
        if (!blind)
            quad(i, j, 2*count+j, 2*count+i);
    }
    if (blind) {
        // Close the bore at z=0, then close the outer body's lower face at z=-1.
        const int center = int(mesh.vertices.size());
        mesh.vertices.emplace_back(0.f, 0.f, 0.f);
        for (unsigned i=0; i<sides; ++i)
            mesh.indices.emplace_back(center, i, (i+1)%count);
        const int lower = int(mesh.vertices.size());
        mesh.vertices.emplace_back(0.f,0.f,-1.f);
        for (unsigned i=0; i<count; ++i)
            mesh.vertices.emplace_back(mesh.vertices[2*count+i].x(),mesh.vertices[2*count+i].y(),-1.f);
        for (unsigned i=0; i<sides; ++i) {
            const int j=(i+1)%count;
            quad(2*count+i,lower+1+i,lower+1+j,2*count+j);
            mesh.indices.emplace_back(lower,lower+1+j,lower+1+i);
        }
    }
    if (!closed) {
        quad(0, 2*count, 3*count, count);
        quad(count-1, 2*count-1, 4*count-1, 3*count-1);
    }
    return mesh;
}

indexed_triangle_set chamfered_bore()
{
    indexed_triangle_set mesh;
    constexpr int count=64;
    const std::array<double,5> radii{{2.,2.,1.8,7.,7.}};
    const std::array<double,5> heights{{0.,9.,10.,0.,10.}};
    for(int ring=0;ring<5;++ring)
        for(int i=0;i<count;++i) {
            const double a=2.*M_PI*i/count;
            mesh.vertices.emplace_back(float(radii[ring]*cos(a)),float(radii[ring]*sin(a)),float(heights[ring]));
        }
    auto quad=[&](int a,int b,int c,int d) {
        mesh.indices.emplace_back(a,b,c); mesh.indices.emplace_back(a,c,d);
    };
    for(int i=0;i<count;++i) {
        const int j=(i+1)%count;
        quad(i,count+i,count+j,j);
    }
    for(int i=0;i<count;++i) {
        const int j=(i+1)%count;
        quad(count+i,2*count+i,2*count+j,count+j);
        quad(3*count+i,3*count+j,4*count+j,4*count+i);
        quad(2*count+i,4*count+i,4*count+j,2*count+j);
        quad(i,j,3*count+j,3*count+i);
    }
    return mesh;
}

void require_bore(const std::vector<RoundHole> &holes, unsigned sides=64, double radius=2.)
{
    REQUIRE(holes.size()==1);
    REQUIRE(holes.front().radius == Approx(radius).margin(0.002));
    REQUIRE(holes.front().facets.size()==2*sides);
    for (int i=0;i<int(2*sides);++i)
        CHECK(std::binary_search(holes.front().facets.begin(),holes.front().facets.end(),i));
}
}

TEST_CASE("Round-hole fill selects the bore without the outer cylinder or end faces", "[WallPainting][RoundHoles]")
{
    require_bore(detect_round_internal_holes(bore()));
}

TEST_CASE("Round-hole fill accepts blind and arbitrarily oriented mirrored bores", "[WallPainting][RoundHoles]")
{
    const bool blind=GENERATE(false,true);
    const double mirror=GENERATE(-1.,1.);
    Transform3d transform=Transform3d::Identity();
    transform.translate(Vec3d(800.,-270.,120.));
    transform.rotate(Eigen::AngleAxisd(1.231,Vec3d(2.,-3.,1.).normalized()));
    transform.scale(Vec3d(mirror,1.,1.));
    const auto holes=detect_round_internal_holes(bore(64,2.*M_PI,2.,blind),transform);
    require_bore(holes);
    CHECK(std::abs(holes.front().axis.dot(transform.linear()*Vec3d::UnitZ()))==Approx(1.).margin(1e-5));
}

TEST_CASE("Round-hole fill recognizes complete bores with oblique rims", "[WallPainting][RoundHoles]")
{
    auto mesh=bore();
    // A steeply tilted thin plate: no horizontal section contains the whole
    // bore, but its wall still wraps around the cylinder exactly once.
    for(auto &vertex:mesh.vertices)
        vertex.z()=0.1f*vertex.z()+2.f*vertex.x();
    require_bore(detect_round_internal_holes(mesh));
}

TEST_CASE("Round-hole fill survives shallow chamfer seed ordering", "[WallPainting][RoundHoles]")
{
    auto mesh=chamfered_bore();
    const int ordering=GENERATE(0,1,2,3);
    std::vector<int> original_faces(mesh.indices.size());
    std::iota(original_faces.begin(),original_faces.end(),0);
    if(ordering==1) {
        // Same winding and geometry; put the chamfer edge first.
        for(auto &face:mesh.indices) face=Vec3i(face[1],face[2],face[0]);
    } else if(ordering==2) {
        std::reverse(mesh.indices.begin(),mesh.indices.end());
        std::reverse(original_faces.begin(),original_faces.end());
    } else if(ordering==3) {
        std::mt19937 rng(1729);
        std::shuffle(original_faces.begin(),original_faces.end(),rng);
        const auto old_faces=mesh.indices;
        for(size_t i=0;i<original_faces.size();++i) {
            const auto &face=old_faces[original_faces[i]];
            mesh.indices[i]=Vec3i(face[2],face[0],face[1]);
        }
    }
    const auto holes=detect_round_internal_holes(mesh);
    REQUIRE(holes.size()==1);
    REQUIRE(holes.front().facets.size()==128);
    for(int face:holes.front().facets) CHECK(original_faces[face]<128);
}

TEST_CASE("Round-hole fill recognizes densely tessellated cylinders", "[WallPainting][RoundHoles]")
{
    require_bore(detect_round_internal_holes(bore(8192)),8192);
}

TEST_CASE("Round-hole fill rejects a highly tessellated spherical cavity", "[WallPainting][RoundHoles]")
{
    auto mesh=its_make_sphere(2.,M_PI/180.);
    its_flip_triangles(mesh);
    REQUIRE(mesh.indices.size()>100000);
    CHECK(detect_round_internal_holes(mesh).empty());
}

TEST_CASE("Round-hole fill rejects equatorial sphere belts independently of face order", "[WallPainting][RoundHoles]")
{
    auto mesh=its_make_sphere(2.,M_PI/(63.-0.000001));
    its_flip_triangles(mesh);
    const int ordering=GENERATE(0,1,2);
    if(ordering==1) {
        // An odd latitude count creates a genuinely cylindrical narrow belt
        // between the two equal-radius rings on either side of the equator.
        // It is still part of a spherical cavity, not a bore to reinforce.
        std::stable_sort(mesh.indices.begin(),mesh.indices.end(),[&](const Vec3i &a,const Vec3i &b) {
            auto tilt=[&](const Vec3i &t) {
                return std::abs((mesh.vertices[t[1]]-mesh.vertices[t[0]]).cross(mesh.vertices[t[2]]-mesh.vertices[t[0]]).normalized().z());
            };
            return tilt(a)<tilt(b);
        });
    } else if(ordering==2) {
        std::mt19937 rng(1729); std::shuffle(mesh.indices.begin(),mesh.indices.end(),rng);
    }
    CHECK(detect_round_internal_holes(mesh).empty());
}

TEST_CASE("Round-hole fill keeps very shallow bores with flat mouths", "[WallPainting][RoundHoles]")
{
    auto mesh=bore();
    for(auto &vertex:mesh.vertices) vertex.z()*=0.005f;
    require_bore(detect_round_internal_holes(mesh));
}

TEST_CASE("Round-hole fill rejects channels polygons taper and elliptical sections", "[WallPainting][RoundHoles]")
{
    SECTION("open 330 degree channel") { CHECK(detect_round_internal_holes(bore(64,11.*M_PI/6.)).empty()); }
    SECTION("hexagon") { CHECK(detect_round_internal_holes(bore(6)).empty()); }
    SECTION("octagon") { CHECK(detect_round_internal_holes(bore(8)).empty()); }
    SECTION("taper") { CHECK(detect_round_internal_holes(bore(64,2.*M_PI,2.4)).empty()); }
    SECTION("ellipse") {
        Transform3d transform=Transform3d::Identity(); transform.scale(Vec3d(1.2,1.,1.));
        CHECK(detect_round_internal_holes(bore(),transform).empty());
    }
    SECTION("external cylinder") { CHECK(detect_round_internal_holes(its_make_cylinder(2.,10.,M_PI/32.)).empty()); }
    SECTION("spherical internal cavity") {
        auto mesh=its_make_sphere(2.,M_PI/64.);
        its_flip_triangles(mesh);
        CHECK(detect_round_internal_holes(mesh).empty());
    }
    SECTION("helical strip with complete angular coverage") {
        auto mesh=bore(96,3.*M_PI);
        constexpr int count=97;
        for(size_t i=0;i<mesh.vertices.size();++i)
            mesh.vertices[i].z()=.04f*mesh.vertices[i].z()+float(8.*(i%count)/96.);
        CHECK(detect_round_internal_holes(mesh).empty());
    }
}

TEST_CASE("Round-hole fill keeps close disconnected bores separate and permits circular scaling", "[WallPainting][RoundHoles]")
{
    auto mesh=bore();
    auto other=bore();
    for(auto &vertex:other.vertices) vertex.x()+=15.f;
    const int vertex_offset=int(mesh.vertices.size()), face_offset=int(mesh.indices.size());
    mesh.vertices.insert(mesh.vertices.end(),other.vertices.begin(),other.vertices.end());
    for(auto face:other.indices) mesh.indices.emplace_back(face.array()+vertex_offset);
    Transform3d transform=Transform3d::Identity(); transform.scale(Vec3d(2.,2.,3.));
    const auto holes=detect_round_internal_holes(mesh,transform);
    REQUIRE(holes.size()==2);
    for(const auto &hole:holes) {
        CHECK(hole.radius==Approx(4.).margin(0.002));
        CHECK(hole.facets.size()==128);
        for(int face:hole.facets)
            CHECK(((face>=0 && face<128) || (face>=face_offset && face<face_offset+128)));
    }
}

TEST_CASE("Round-hole fill handles degenerate and nonmanifold candidates conservatively", "[WallPainting][RoundHoles]")
{
    SECTION("empty") { CHECK(detect_round_internal_holes(indexed_triangle_set{}).empty()); }
    SECTION("zero area") {
        indexed_triangle_set mesh; mesh.vertices.emplace_back(0,0,0); mesh.indices.emplace_back(0,0,0);
        CHECK(detect_round_internal_holes(mesh).empty());
    }
    SECTION("duplicate cylinder face") {
        auto mesh=bore(); mesh.indices.push_back(mesh.indices.front());
        CHECK(detect_round_internal_holes(mesh).empty());
    }
}

TEST_CASE("Round-hole fill preserves unrelated subdivided manual paint", "[WallPainting][RoundHoles]")
{
    TriangleMesh mesh(bore());
    TriangleSelector selector(mesh);
    const int outer_face=128;
    selector.set_facet(outer_face,EnforcerBlockerType::ENFORCER);
    const auto &face=mesh.its.indices[outer_face];
    Vec3f center=(mesh.its.vertices[face[0]]+mesh.its.vertices[face[1]]+mesh.its.vertices[face[2]])/3.f;
    selector.select_patch(outer_face,TriangleSelector::SinglePointCursor::cursor_factory(center,center+Vec3f(0,0,20),0.5f,
        TriangleSelector::CursorType::SPHERE,Transform3d::Identity(),TriangleSelector::ClippingPlane()),
        EnforcerBlockerType::NONE,Transform3d::Identity(),true);
    Model model;
    auto *object=model.add_object();
    auto &before=object->add_volume(mesh)->wall_painting_facets; before.set(selector);
    REQUIRE(before.get_triangle_as_string(outer_face).size()>1);
    for (const auto &hole:detect_round_internal_holes(mesh.its))
        for (int face_id:hole.facets) selector.set_facet(face_id,EnforcerBlockerType::ENFORCER);
    auto &after=object->add_volume(mesh)->wall_painting_facets; after.set(selector);
    CHECK(after.get_triangle_as_string(outer_face)==before.get_triangle_as_string(outer_face));
    CHECK(after.get_triangle_as_string(0)=="4");
    TriangleSelector reopened(mesh); reopened.deserialize(after.get_data());
    auto &roundtrip=object->add_volume(mesh)->wall_painting_facets; roundtrip.set(reopened);
    CHECK(roundtrip.equals(after));

    object->add_instance();
    const auto workspace=boost::filesystem::temp_directory_path()/boost::filesystem::unique_path("round-hole-%%%%-%%%%");
    boost::filesystem::create_directories(workspace);
    const std::string original_data=data_dir(), original_temp=temporary_dir();
    set_data_dir(workspace.string()); set_temporary_dir(workspace.string());
    struct Cleanup {
        boost::filesystem::path path; std::string data,temp;
        ~Cleanup() { set_data_dir(data); set_temporary_dir(temp); boost::system::error_code ec; boost::filesystem::remove_all(path,ec); }
    } cleanup{workspace,original_data,original_temp};
    const std::string filename=(workspace/"filled.3mf").string();
    DynamicPrintConfig config;
    StoreParams params; params.path=filename.c_str(); params.model=&model; params.config=&config;
    REQUIRE(store_bbs_3mf(params));
    Model loaded; DynamicPrintConfig loaded_config;
    ConfigSubstitutionContext substitutions(ForwardCompatibilitySubstitutionRule::Disable);
    bool is_bambu=false; Semver version; PlateDataPtrs plates; std::vector<Preset*> presets;
    REQUIRE(load_bbs_3mf(filename.c_str(),&loaded_config,&substitutions,&loaded,&plates,&presets,&is_bambu,&version,
        nullptr,LoadStrategy::LoadModel|LoadStrategy::LoadConfig));
    REQUIRE(loaded.objects.size()==1);
    REQUIRE(loaded.objects.front()->volumes.size()==3);
    CHECK(loaded.objects.front()->volumes[1]->wall_painting_facets.equals(after));
    CHECK(loaded.objects.front()->volumes[1]->wall_painting_facets.get_triangle_as_string(outer_face)==before.get_triangle_as_string(outer_face));
    for(auto *plate:plates) delete plate;
}

TEST_CASE("Round-hole fill reports progress and propagates cancellation", "[WallPainting][RoundHoles]")
{
    float previous=-1.f;
    const auto holes=detect_round_internal_holes(bore(),Transform3d::Identity(),[&](float p) {
        CHECK(p>=previous); CHECK(p>=0.f); CHECK(p<=1.f); previous=p;
    });
    REQUIRE(previous==1.f);
    REQUIRE_FALSE(holes.empty());
    REQUIRE_THROWS_AS(detect_round_internal_holes(bore(),Transform3d::Identity(),[](float) {
        throw std::runtime_error("canceled");
    }),std::runtime_error);
    REQUIRE_THROWS_AS(detect_round_internal_holes(bore(),Transform3d::Identity(),[](float progress) {
        if(progress>=0.4f) throw std::runtime_error("canceled during detection");
    }),std::runtime_error);
}
