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
    struct NodeIndex {
        using Index = std::array<MeshIndex, 3>;
        Index index;
        int depth;
    };

    struct RootInfo {
        Point3D<Real> center;
        Real width;
        int maxDepth; // must be known beforehand for practical reasons

        // TODO
        // Point3D<Real> getNodeCoordinates(const NodeIndex &nodeIndex, float offset = 0.0) const;
        // Real getNodeWidth(const NodeIndex &nodeIndex) const;
    };

    using CornerValues = std::array<float, 8>;

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
