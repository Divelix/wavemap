#include "pywavemap/maps.h"

#include <nanobind/eigen/dense.h>
#include <nanobind/stl/filesystem.h>
#include <nanobind/stl/shared_ptr.h>
#include <wavemap/core/map/hashed_chunked_wavelet_octree.h>
#include <wavemap/core/map/hashed_wavelet_octree.h>
#include <wavemap/core/map/map_base.h>
#include <wavemap/core/map/map_factory.h>
#include <wavemap/core/utils/query/query_accelerator.h>
#include <wavemap/io/file_conversions.h>

using namespace nb::literals;  // NOLINT

namespace wavemap {
void add_map_bindings(nb::module_& m) {
  auto map_base =
      nb::class_<MapBase>(m, "Map", "Base class for wavemap maps.")
          .def_prop_ro("empty", &MapBase::empty, "Whether the map is empty.")
          .def_prop_ro("size", &MapBase::size,
                       "The number of cells or nodes in the map, for fixed or "
                       "multi-resolution maps, respectively.")
          .def("threshold", &MapBase::threshold,
               "Threshold the occupancy values of all cells in the map to stay "
               "within the range specified by its min_log_odds and "
               "max_log_odds.")
          .def(
              "prune", &MapBase::prune,
              "Free up memory by pruning nodes that are no longer needed. Note "
              "that this pruning operation is lossless and does not alter the "
              "estimated occupancy posterior.")
          .def("pruneSmart", &MapBase::pruneSmart,
               "Similar to prune(), but avoids de-allocating nodes that were "
               "recently updated and will likely be used again in the near "
               "future.")
          .def("clear", &MapBase::clear, "Erase all cells in the map.")
          .def_prop_ro(
              "min_cell_width", &MapBase::getMinCellWidth,
              "Maximum map resolution, set as width of smallest cell it "
              "can represent.")
          .def_prop_ro("min_log_odds", &MapBase::getMinLogOdds,
                       "Lower threshold for the occupancy values stored in the "
                       "map, in log-odds.")
          .def_prop_ro("max_log_odds", &MapBase::getMaxLogOdds,
                       "Upper threshold for the occupancy values stored in the "
                       "map, in log-odds.")
          .def_prop_ro("memory_usage", &MapBase::getMemoryUsage,
                       "The amount of memory used by the map, in bytes.")
          .def_prop_ro(
              "tree_height", &MapBase::getTreeHeight,
              "Height of the octree used to store the map. Note that this "
              "value is only defined for multi-resolution maps.")
          .def_prop_ro("min_index", &MapBase::getMinIndex,
                       "Index of the minimum corner of the map's Axis Aligned "
                       "Bounding Box.")
          .def_prop_ro("max_index", &MapBase::getMaxIndex,
                       "Index of the maximum corner of the map's Axis Aligned "
                       "Bounding Box.")
          .def("getCellValue", &MapBase::getCellValue, "index"_a,
               "Query the value of the map at a given index.")
          .def("setCellValue", &MapBase::setCellValue, "index"_a,
               "new_value"_a
               "Set the value of the map at a given index.")
          .def("addToCellValue", &MapBase::addToCellValue, "index"_a,
               "update"_a, "Increment the value of the map at a given index.")
          .def_static(
              "create",
              [](const param::Value& params) -> std::shared_ptr<MapBase> {
                return MapFactory::create(params);
              },
              nb::sig("def create(parameters: dict) -> Map"), "parameters"_a,
              "Create a new map based on the given settings.")
          .def_static(
              "load",
              [](const std::filesystem::path& file_path)
                  -> std::shared_ptr<MapBase> {
                std::shared_ptr<MapBase> map;
                if (wavemap::io::fileToMap(file_path, map)) {
                  return map;
                }
                return nullptr;
              },
              "file_path"_a, "Load a wavemap map from a .wvmp file.")
          .def(
              "store",
              [](const MapBase& self, const std::filesystem::path& file_path)
                  -> bool { return wavemap::io::mapToFile(self, file_path); },
              "file_path"_a, "Store a wavemap map as a .wvmp file.");

  nb::class_<HashedWaveletOctree>(
      m, "HashedWaveletOctree", map_base,
      "A class that stores maps using hashed wavelet octrees.")
      .def("getCellValue", &MapBase::getCellValue, "index"_a,
           "Query the value of the map at a given index.")
      .def("getCellValue",
           nb::overload_cast<const OctreeIndex&>(
               &HashedWaveletOctree::getCellValue, nb::const_),
           "node_index"_a,
           "Query the value of the map at a given octree node index.");

  nb::class_<HashedChunkedWaveletOctree>(
      m, "HashedChunkedWaveletOctree", map_base,
      "A class that stores maps using hashed chunked wavelet octrees.")
      .def("getCellValue", &MapBase::getCellValue, "index"_a,
           "Query the value of the map at a given index.")
      .def("getCellValue",
           nb::overload_cast<const OctreeIndex&>(
               &HashedChunkedWaveletOctree::getCellValue, nb::const_),
           "node_index"_a,
           "Query the value of the map at a given octree node index.");

  nb::class_<QueryAccelerator<HashedWaveletOctree>>(
      m, "HashedWaveletOctreeQueryAccelerator",
      "A class that accelerates queries by caching block and parent node "
      "addresses to speed up data structure traversals, and intermediate "
      "wavelet decompression results to reduce redundant computation.")
      .def(nb::init<const HashedWaveletOctree&>(), "map"_a)
      .def("getCellValue",
           nb::overload_cast<const Index3D&>(
               &QueryAccelerator<HashedWaveletOctree>::getCellValue),
           "index"_a, "Query the value of the map at a given index.")
      .def("getCellValue",
           nb::overload_cast<const OctreeIndex&>(
               &QueryAccelerator<HashedWaveletOctree>::getCellValue),
           "node_index"_a,
           "Query the value of the map at a given octree node index.")
      .def(
          "getCellValues",
          [](QueryAccelerator<HashedWaveletOctree>& self,
             const nb::ndarray<IndexElement, nb::shape<-1, 3>, nb::device::cpu>&
                 indices) {
            // Create nb::ndarray view for efficient access
            const auto index_view = indices.view();
            const auto num_queries = index_view.shape(0);
            // Allocate and populate raw results array
            auto* results = new float[num_queries];
            for (size_t query_idx = 0; query_idx < num_queries; ++query_idx) {
              results[query_idx] = self.getCellValue(
                  {index_view(query_idx, 0), index_view(query_idx, 1),
                   index_view(query_idx, 2)});
            }
            // Create Python capsule that deallocates the results array when
            // all references to it expire
            nb::capsule owner(results, [](void* p) noexcept {
              delete[] reinterpret_cast<float*>(p);
            });
            // Return results as numpy array
            return nb::ndarray<nb::numpy, float>{
                results, {num_queries, 1u}, owner};
          },
          "index_list"_a,
          "Query the map at the given indices, provided as a matrix with one "
          "(x, y, z) index per row.")
      .def(
          "getCellValues",
          [](QueryAccelerator<HashedWaveletOctree>& self,
             const nb::ndarray<IndexElement, nb::shape<-1, 4>, nb::device::cpu>&
                 indices) {
            // Create nb::ndarray view for efficient access
            auto index_view = indices.view();
            const auto num_queries = index_view.shape(0);
            // Allocate and populate raw results array
            auto* results = new float[num_queries];
            for (size_t query_idx = 0; query_idx < num_queries; ++query_idx) {
              const OctreeIndex node_index{
                  index_view(query_idx, 0),
                  {index_view(query_idx, 1), index_view(query_idx, 2),
                   index_view(query_idx, 3)}};
              results[query_idx] = self.getCellValue(node_index);
            }
            // Create Python capsule that deallocates the results array when
            // all references to it expire
            nb::capsule owner(results, [](void* p) noexcept {
              delete[] reinterpret_cast<float*>(p);
            });
            // Return results as numpy array
            return nb::ndarray<nb::numpy, float>{
                results, {num_queries, 1u}, owner};
          },
          "node_index_list"_a,
          "Query the map at the given node indices, provided as a matrix with "
          "one (height, x, y, z) node index per row.");
}
}  // namespace wavemap
