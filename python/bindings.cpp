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

#include <memory>
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/functional.h>
#include <pybind11/stl.h>
#include "IsoOctree/IsoOctree.hpp"
#include <iostream>

#define DEF_READONLY(klass, name, doc) def_readonly(#name, &klass::name, doc)
#define DEF_COPY_READONLY(klass, name, doc) def_property_readonly(\
    #name,\
    [](const klass &self) { return self.name; },\
    py::return_value_policy::copy, doc)


namespace py = pybind11;

namespace {
using Real = double;
using Octree = isoOctree::Octree<Real>;
using Voxel = Octree::Voxel;
using Point3D = isoOctree::Point3D<Real>;
using CornerValues = Octree::CornerValues;
using RootInfo = Octree::RootInfo;
using MeshInfo = isoOctree::MeshInfo<Real>;

using ShouldExpandCallback = std::function<bool(const Voxel &, const CornerValues &corners)>;
using GetValueCallback = std::function<float(const Point3D &)>;

class CallbackTraverser : public Octree::Traverser {
private:
    RootInfo rootInfo;
    ShouldExpandCallback shouldExpandCallback;
    GetValueCallback getValueCallback;

public:
    CallbackTraverser(
        const RootInfo &rootInfo,
        const ShouldExpandCallback &shouldExpandCallback,
        const GetValueCallback &getValueCallback)
    :
        rootInfo(rootInfo),
        shouldExpandCallback(shouldExpandCallback),
        getValueCallback(getValueCallback)
    {}

    const RootInfo &root() final { return rootInfo; }
    bool shouldExpand(const Voxel &voxel, const CornerValues &corners) final {
        return shouldExpandCallback(voxel, corners);
    }

    float isoValue(const Point3D &point) final {
        return getValueCallback(point);
    }
};
}

PYBIND11_MODULE(IsoOctree, m) {
    m.doc() = "IsoOctree Python wrapper";

    py::class_<Voxel>(m, "Voxel")

        .DEF_READONLY(Voxel, index,
            "Level-specific octree node index triplet (i,j,k), each in the range [0, 2^depth-1]")

        .DEF_READONLY(Voxel, depth,
            "Octree level: 0 is root, 1 is the first level, etc.")

        .DEF_COPY_READONLY(Voxel, minCorner,
            "Voxel minimum coordinate corner in world coordinates")

        .DEF_READONLY(Voxel, width,
            "Voxel cube width");

    py::class_<MeshInfo, std::unique_ptr<MeshInfo>>(m, "MeshInfo")

        .def_property_readonly("vertices",
            [](MeshInfo &self) {
                return py::array_t<Real>(
                    std::vector<ptrdiff_t> { static_cast<long>(self.vertices.size()), 3 },
                    reinterpret_cast<const Real*>(self.vertices.data())
                );
            },
            "Vertices numpy array of shape (N, 3)")

        .def_property_readonly("triangles",
            [](MeshInfo &self) {
                return py::array_t<isoOctree::MeshIndex>(
                    std::vector<ptrdiff_t> { static_cast<long>(self.triangles.size()), 3 },
                    reinterpret_cast<const isoOctree::MeshIndex*>(self.triangles.data())
                );
            },
            "Faces (all triangles) as an index array of shape (M, 3), each element\n"
            "is an index to the rows in vertices, and thus in the range (0, N-1)");

    m.def("buildMesh",
        [](
            GetValueCallback isoFunction,
            ShouldExpandCallback expandFunction,
            Point3D center,
            Real width,
            int maxDepth)
        {
            Octree::RootInfo rootInfo;
            rootInfo.center = center;
            rootInfo.width = width;
            rootInfo.maxDepth = maxDepth;
            CallbackTraverser traverser(rootInfo, expandFunction, isoFunction);
            std::unique_ptr<isoOctree::MeshInfo<Real>> output = std::make_unique<isoOctree::MeshInfo<Real>>();
            isoOctree::buildMesh(traverser, *output);
            return std::move(output);
        },
        py::arg("isoFunction"),
        py::arg("expandFunction"),
        py::kw_only(),
        py::arg("center") = Point3D { 0, 0, 0 },
        py::arg("width") = 1.0,
        py::arg("maxDepth") = 10,
        "Mesh an implicit function using the Iso-Octree algorithm \n"
        "\n"
        "Applied to a cubical region [cx-w/2, cx+w/2] x [cy-w/2, cy+w/2] x [cz-w/2, cz+w/2]\n"
        "called the (octree) root voxel.\n"
        "\n"
        "Arguments:\n"
        "\t isoFunction: Function f that defines the surface (f < 0 is inside)\n"
        "\t expandFunction: Function that decides whether to expand a Voxel\n"
        "\t center: Center (cx, cy, cz) of the root voxel\n"
        "\t width: Width of the root voxel w\n"
        "\t maxDepth: Maximum octree depth d. The smallest possible voxel dimension is w/2^d\n"
        "\n"
        "Returns: a MeshInfo object");
}

