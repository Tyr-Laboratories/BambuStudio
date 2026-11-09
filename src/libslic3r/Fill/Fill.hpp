#ifndef slic3r_Fill_hpp_
#define slic3r_Fill_hpp_

#include <memory.h>
#include <float.h>
#include <stdint.h>

#include "../libslic3r.h"
#include "../ExPolygon.hpp"
#include "../PrintConfig.hpp"

#include "FillBase.hpp"

namespace Slic3r {

class ExtrusionEntityCollection;
class Layer;
class LayerRegion;

// The islands of the sparse fills of one pattern on a layer, as Layer::make_fills hands them to the filler, each with the regions
// whose sparse fill it holds (the fill groups' region_id_group).
struct SparseIslandOfPattern
{
    std::vector<size_t> region_ids;
    ExPolygon           island;
};
std::vector<SparseIslandOfPattern> sparse_islands_of_pattern(const Layer &layer, InfillPattern pattern);

// An interface class to Perl, aggregating an instance of a Fill and a FillData.
class Filler
{
public:
    Filler() : fill(nullptr) {}
    ~Filler() { 
        delete fill; 
        fill = nullptr;
    }
    Fill        *fill;
    FillParams   params;
};

} // namespace Slic3r

#endif // slic3r_Fill_hpp_
