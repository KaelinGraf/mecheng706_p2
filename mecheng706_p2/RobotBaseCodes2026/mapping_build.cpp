// ---------------------------------------------------------------------------
// mapping_build.cpp — unity shim
//
// The Arduino build only compiles .cpp/.ino in the sketch ROOT (and src/), NOT
// arbitrary subfolders. The mapping pipeline lives in mapping/, so without this
// shim its translation units never get compiled or linked. This file lives in
// the root, so Arduino DOES compile it; it #includes the mapping sources to pull
// them into the build.
//
// The quote-includes inside the mapping sources (e.g. "mapping/occupancy_grid.h",
// "../firefighter.h", "utils.h") resolve via the sketch-root -I path that the
// Arduino build adds, so the original file layout and includes are untouched.
//
// Guarded by ENABLE_MAPPING so that with mapping OFF this compiles to nothing and
// the firmware builds exactly as before.
// ---------------------------------------------------------------------------
#include "mappings.h"

#if ENABLE_MAPPING
#include "mapping/pose.cpp"
#include "mapping/robot_model.cpp"
#include "mapping/occupancy_grid.cpp"
#include "mapping/fire_localiser.cpp"
#include "mapping/world_model.cpp"
// navigator.cpp is intentionally omitted: it is empty, and Navigator's ctor is
// header-inline while its dtor/navigate() are never linker-referenced yet.
#endif
