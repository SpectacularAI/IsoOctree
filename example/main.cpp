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

#include <IsoOctree/IsoOctree.hpp>
#include <fstream>
#include <cassert>

class OctreeBuilder : public isoOctree::Octree<float>::Traverser {
    static constexpr int BASE_DEPTH = 3;
    isoOctree::Octree<float>::RootInfo rootInfo;

public:
    OctreeBuilder() {
        rootInfo.maxDepth = 6;
        rootInfo.center = { 0.1, 0.2, 0.3 };
        rootInfo.width = 6.0;
    }

    const isoOctree::Octree<float>::RootInfo &root() final {
        return rootInfo;
    }

    float isoValue(const isoOctree::Point3D<float> &point) final {
        float x = point[0], y = point[1], z = point[2];
        return x*x*x*x - 5*x*x + y*y*y*y - 5*y*y + z*z*z*z - 5*z*z + 11.8; // Tanglecube
    }

    bool shouldExpand(
        const isoOctree::Octree<float>::NodeIndex &node,
        const isoOctree::Octree<float>::CornerValues &corners) final
    {
        if (node.depth < BASE_DEPTH) return true;

        int nNonNegative = 0, nNegative = 0;
        for (float v : corners) {
            if (v < 0.0) nNegative++;
            else nNonNegative++;
        }

        if (nNonNegative == 0 || nNegative == 0) return false;
        // Note: this logic is rather flawed for adaptive refinement,
        // but results to leaf nodes at different level, which is a good
        // test to the IsoOctree algorithm
        return nNonNegative != nNegative;
    }
};

template <class Real>
void writeMeshAsObj(const isoOctree::MeshInfo<Real> &mesh, const char *outFileName) {
    std::ofstream out(outFileName);
    for (const auto &vertex : mesh.vertices) {
        out << "v";
        for (auto c : vertex) out << " " << c;
        out << "\n";
    }
    for (const auto &tri : mesh.triangles) {
        out << "f";
        for (auto vi : tri) out << " " << (vi+1);
        out << "\n";
    }
}

int main(int argc, char *argv[]) {
    isoOctree::MeshInfo<float> mesh;
    OctreeBuilder builder;
    isoOctree::buildMesh(builder, mesh);
    if (argc > 1) {
        assert(argc == 2);
        writeMeshAsObj(mesh, argv[1]);
    }
}
