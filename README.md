# IsoOctree library

An implementation of the _Unconstrained Isosurface Extraction on Arbitrary Octrees_ algorithm
forked from the original [IsoOctree code](https://www.cs.jhu.edu/~misha/Code/IsoOctree/) by Michael Kazhdan.

Main modifications:
 * Add an API (the original code was a command line application), see `include/IsoOctree/IsoOctree.hpp` and `example/`.
 * Port to C++11
 * Add CMake installation scripts
 * Python bindings
