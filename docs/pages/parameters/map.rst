Map
###
.. highlight:: YAML
.. rstcheck: ignore-directives=doxygenstruct

These settings control the data structure that is used to store the map.
They are nested in the top level config under ``map``.

The following three data structures are fully supported by wavemap's ROS server: ``wavelet_octree``, ``hashed_wavelet_octree`` or ``hashed_chunked_wavelet_octree``. For general use, we recommend the ``hashed_wavelet_octree`` data structure. If you need the best possible performance, the ``hashed_chunked_wavelet_octree`` data structure is faster and uses less RAM. However, it is still under active development.

Wavelet octree
**************
Selected by setting ``map/type: "wavelet_octree"``.

The ``wavelet_octree`` is the simplest of the wavelet-based data structures and stores the wavelet coefficients in a standard octree. This data structure can be useful in case you need the entire map to be contained in a single tree and don't mind sacrificing performance.

.. doxygenstruct:: wavemap::WaveletOctreeConfig
    :project: wavemap_cpp
    :members:

Hashed wavelet octree
*********************
Selected by setting ``map/type: "hashed_wavelet_octree"``.

The ``hashed_wavelet_octree`` combines the strengths of wavelet octrees with block-hashing. At the top level, the map is split into blocks which are accessed through a hash table. Each block in turn contains a small wavelet octree.

.. doxygenstruct:: wavemap::HashedWaveletOctreeConfig
    :project: wavemap_cpp
    :members:

Hashed chunked wavelet octree
*****************************
Selected by setting ``map/type: "hashed_chunked_wavelet_octree"``.

The ``hashed_chunked_wavelet_octree`` is similar to the ``hashed_wavelet_octree``, but instead of storing all octree nodes separately it groups them into chunks.

.. doxygenstruct:: wavemap::HashedChunkedWaveletOctreeConfig
    :project: wavemap_cpp
    :members:
