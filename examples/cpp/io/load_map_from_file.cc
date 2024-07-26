#include <wavemap/io/file_conversions.h>

int main(int, char**) {
  // Create a smart pointer that will own the loaded map
  wavemap::MapBase::Ptr loaded_map;

  // Load the map
  wavemap::io::fileToMap("/some/path/to/your/map.wvmp", loaded_map);
}
