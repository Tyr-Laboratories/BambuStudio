#include "WallPainting.hpp"
#include "Print.hpp"
#include "Layer.hpp"
#include "Model.hpp"
#include "TriangleMeshSlicer.hpp"
#include "ClipperUtils.hpp"

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

namespace Slic3r::WallPainting {
std::vector<LayerPaint> slice_paint(const PrintObject &object, const std::function<void()> &cancel)
{
    const auto layers=object.layers();
    std::vector<LayerPaint> result(layers.size());
    bool slope_walls=false;
    for (const auto &region:object.all_regions())
        slope_walls |= region.get().config().extra_wall_slope_angle.value>0.;
    if ((!object.model_object()->is_wall_painting_painted() && !slope_walls) || object.print()->config().spiral_mode) return result;
    std::vector<float> heights;
    heights.reserve(layers.size());
    for (const Layer *layer : layers) heights.push_back(float(layer->slice_z));
    std::vector<Polygons> material(layers.size()), removed(layers.size());
    for (const ModelVolume *volume : object.model_object()->volumes) {
        if (!volume->is_model_part() && !volume->is_negative_volume()) continue;
        cancel();
        const Transform3d transform=object.trafo()*volume->get_matrix();
        const bool mirrored=transform.linear().determinant()<0.;
        // Resolve source ownership before XY compensation moves the boundary.
        // A removed face can be closer to a compensated cut than the face that
        // actually survives. Distance matching against raw facets alone would
        // incorrectly transfer that removed face's painting to the cut.
        MeshSlicingParamsEx params;
        params.trafo=transform;
        switch (object.config().slicing_mode.value) {
        case SlicingMode::Regular: params.mode=MeshSlicingParams::SlicingMode::Regular; break;
        case SlicingMode::EvenOdd: params.mode=MeshSlicingParams::SlicingMode::EvenOdd; break;
        case SlicingMode::CloseHoles: params.mode=MeshSlicingParams::SlicingMode::Positive; break;
        }
        indexed_triangle_set source_mesh=volume->mesh().its;
        if (mirrored) its_flip_triangles(source_mesh);
        auto slices=slice_mesh_ex(source_mesh,heights,params,cancel);
        auto &destination=volume->is_negative_volume() ? removed : material;
        for (size_t i=0;i<slices.size();++i) {
            auto polygons=to_polygons(std::move(slices[i]));
            for (auto &polygon:polygons) polygon.translate(-object.center_offset());
            polygons_append(destination[i],std::move(polygons));
        }
        for (const auto state : {EnforcerBlockerType::NONE, EnforcerBlockerType::ENFORCER, EnforcerBlockerType::BLOCKER}) {
            if (volume->is_negative_volume() && state != EnforcerBlockerType::NONE) continue;
            const auto facets=volume->is_negative_volume() ? volume->mesh().its : volume->wall_painting_facets.get_facets(*volume,state);
            // Facets are cut in parallel, in fixed blocks whose lines are appended block by block, so every
            // layer receives its lines in facet order, as a serial loop would give them.
            constexpr size_t block_size=4096;
            std::vector<std::vector<std::pair<size_t,PaintedLine>>> blocks((facets.indices.size()+block_size-1)/block_size);
            tbb::parallel_for(tbb::blocked_range<size_t>(0,blocks.size()),[&](const tbb::blocked_range<size_t> &range) {
                for (size_t block=range.begin();block<range.end();++block) {
                    std::vector<std::pair<size_t,PaintedLine>> &out=blocks[block];
                    for (size_t f=block*block_size;f<std::min(facets.indices.size(),(block+1)*block_size);++f) {
                        if ((f&1023)==0) cancel();
                        const auto &face=facets.indices[f];
                        std::array<Vec3d,3> v;
                        for (int i=0;i<3;++i) v[i]=transform*facets.vertices[face[i]].cast<double>();
                        Vec3d normal=(v[1]-v[0]).cross(v[2]-v[0]);
                        if (mirrored) normal=-normal;
                        if (volume->is_negative_volume()) normal=-normal;
                        const double slope_angle=std::atan2(std::abs(normal.z()),normal.head<2>().norm())*180./PI;
                        const double low=std::min({v[0].z(),v[1].z(),v[2].z()});
                        const double high=std::max({v[0].z(),v[1].z(),v[2].z()});
                        if (high-low<EPSILON) continue; // Horizontal faces don't define perimeter boundaries.
                        auto first=std::lower_bound(layers.begin(),layers.end(),low,[](const Layer *l,double z){return l->slice_z<z;});
                        for (auto layer=first;layer!=layers.end() && (*layer)->slice_z<high;++layer) {
                            const double z=(*layer)->slice_z;
                            Vec2d hits[3];
                            int   num_hits=0;
                            for (int e=0;e<3;++e) {
                                const Vec3d &a=v[e], &b=v[(e+1)%3];
                                if ((a.z()<=z && b.z()>z)||(b.z()<=z && a.z()>z)) {
                                    const Vec3d p=a+(b-a)*((z-a.z())/(b.z()-a.z()));
                                    hits[num_hits++]=p.head<2>();
                                }
                            }
                            if (num_hits!=2) continue;
                            Point a=Point::new_scale(hits[0].x(),hits[0].y())-object.center_offset();
                            Point b=Point::new_scale(hits[1].x(),hits[1].y())-object.center_offset();
                            if (a==b) continue;
                            const Vec2d tangent(-normal.y(),normal.x());
                            if ((b-a).cast<double>().dot(tangent)<0.) std::swap(a,b);
                            out.emplace_back(size_t(layer-layers.begin()),
                                PaintedLine{Line(a,b),size_t(volume->id().id),state==EnforcerBlockerType::ENFORCER,-1,slope_angle});
                        }
                    }
                }
            });
            for (const auto &block:blocks)
                for (const auto &[layer,line]:block)
                    result[layer].push_back(line);
        }
    }
    // Each layer's labels depend only on that layer's slices and lines.
    tbb::parallel_for(tbb::blocked_range<size_t>(0,result.size()),[&](const tbb::blocked_range<size_t> &range) {
        for (size_t i=range.begin();i<range.end();++i) {
            cancel();
            const Polygons surviving=diff(union_(material[i]),removed[i]);
            const auto policy=BoundaryPolicy::bind(surviving,result[i],0,1,scaled<double>(0.0002));
            LayerPaint labels;
            labels.reserve(policy.spans().size());
            for (const auto &span:policy.spans())
                labels.push_back({span.line,span.source,span.painted,-1,span.slope_angle});
            result[i]=std::move(labels);
        }
    });
    return result;
}
} // namespace Slic3r::WallPainting
