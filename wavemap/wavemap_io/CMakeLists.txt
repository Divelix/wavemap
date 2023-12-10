add_library(wavemap_io STATIC)
target_sources(wavemap_io PRIVATE
    src/file_conversions.cc
    src/stream_conversions.cc
)
target_include_directories(wavemap_io PRIVATE include ../wavemap_core/include)
target_link_libraries(wavemap_io PRIVATE wavemap_core)
set_global_target_properties(wavemap_io)
