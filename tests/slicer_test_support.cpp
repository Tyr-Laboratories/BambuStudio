// The GUI normally provides NanoSVG's implementation through BitmapCache.cpp.
// Headless slicer tests need the same parser without the GUI bitmap cache.
#define NANOSVG_IMPLEMENTATION
#include <nanosvg/nanosvg.h>
