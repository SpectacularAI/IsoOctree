/*
   Copyright 2024 Spectacular AI Ltd

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/

#ifndef ISO_OCTREE_API_INCLUDED
#define ISO_OCTREE_API_INCLUDED

#include <vector>
#include <unordered_map>
#include <cstdint>
#include <array>
#include <functional>
#include <memory>

#ifdef _MSC_VER
    #define ISO_OCTREE_API __declspec(dllexport)
#else
    #define ISO_OCTREE_API __attribute__((visibility("default")))
#endif

namespace isoOctree {
using MeshIndex = std::size_t;
using TriangleIndex = std::array<MeshIndex, 3>;
template <class Real> using Point3D = std::array<Real, 3>;

template <class MeshReal>
struct MeshInfo {
    std::vector<TriangleIndex> triangles;
    std::vector<Point3D<MeshReal> > vertices;
    // Dropped edgeNormals
};

template <class Real> struct Octree {

    struct Voxel {
        using Index = std::array<MeshIndex, 3>;
        Index index;
        int depth;
        Point3D<Real> minCorner;
        Real width;
    };

    struct RootInfo {
        Point3D<Real> center;
        Real width;
        int maxDepth; // must be known beforehand for practical reasons
    };

    using CornerValues = std::array<float, 8>;

    using NodeIndex = Voxel; // backwards compatibility

    struct Traverser {
        virtual const RootInfo &root() = 0;
        virtual bool shouldExpand(const NodeIndex &node, const CornerValues &corners) = 0;
        virtual float isoValue(const Point3D<Real> &point) = 0;
        // no virtual dtor, do not delete this class
    };
};

template <class Real>
ISO_OCTREE_API void buildMesh(typename Octree<Real>::Traverser &traverser, MeshInfo<Real> &output);

}

#endif
