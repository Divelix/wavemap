#include <nanobind/nanobind.h>

#include "pywavemap/logging.h"
#include "pywavemap/map.h"
#include "pywavemap/measurements.h"
#include "pywavemap/param.h"
#include "pywavemap/pipeline.h"

using namespace wavemap;  // NOLINT
namespace nb = nanobind;

NB_MODULE(pywavemap, m) {
  m.doc() =
      "A fast, efficient and accurate multi-resolution, multi-sensor 3D "
      "occupancy mapping framework.";

  // Setup logging for the C++ Library
  nb::module_ m_logging =
      m.def_submodule("logging", "Submodule for pywavemap's logging system.");
  add_logging_module(m_logging);

  // Bindings and implicit conversions for wavemap's config system
  nb::module_ m_param =
      m.def_submodule("param", "Submodule for wavemap's config system.");
  add_param_module(m_param);

  // Bindings for measurement types
  add_measurement_bindings(m);

  // Bindings for wavemap maps
  add_map_bindings(m);

  // Bindings for measurement integration and map update pipelines
  add_pipeline_bindings(m);
}
