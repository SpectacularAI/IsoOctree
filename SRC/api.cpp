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

#define ISO_OCTREE_API_IMPL
#include "../include/IsoOctree/IsoOctree.hpp"

#define MARCHING_CUBES_IMPL // TODO

#include <cassert>
#include "IsoOctree.h"
#include "VertexData.h"
#include "Geometry.h"
#include "MAT.h"
#include "logging.hpp"
#include "ZOrderOctree.hpp"

namespace {
template<class VertexData, class Real>
struct NodeDataImpl {
	int mcIndex;
	Point3D<Real> center;
	VertexData v;
};

template <class Real>
struct VertexImpl {
	Point3D<Real> point;
};

template <class Real> isoOctree::Point3D<Real> getMappedCornerPosition(
    const typename isoOctree::Octree<Real>::RootInfo &root,
    Real x, Real y, Real z)
{
	return {
        Real((x - 0.5) * root.width + root.center[0]),
        Real((y - 0.5) * root.width + root.center[1]),
        Real((z - 0.5) * root.width + root.center[2])
	};
}

template <class Real> isoOctree::Point3D<Real> getMappedCornerPosition(
    const typename isoOctree::Octree<Real> &root,
    const Point3D<Real> &center)
{
    return getMappedCornerPosition(root, center.coords[0], center.coords[1], center.coords[2]);
}

template <class Real, class NodeIndex> typename isoOctree::Octree<Real>::Voxel convertIdx(
	const NodeIndex &nIdx,
	const typename isoOctree::Octree<Real>::RootInfo &root)
{
    typename isoOctree::Octree<Real>::Voxel idxApi;
    using isoOctree::MeshIndex;
    idxApi.depth = nIdx.depth;
    idxApi.index = { MeshIndex(nIdx.offset[0]), MeshIndex(nIdx.offset[1]), MeshIndex(nIdx.offset[2]) };
	const MeshIndex idxUpperBound = MeshIndex(1) << nIdx.depth;
	idxApi.width = root.width / idxUpperBound;
	idxApi.minCorner = getMappedCornerPosition<Real>(root,
		Real(nIdx.offset[0]) / idxUpperBound,
		Real(nIdx.offset[1]) / idxUpperBound,
		Real(nIdx.offset[2]) / idxUpperBound);
    return idxApi;
}

template<class Vertex,class Real>
void PolygonToTriangleMesh(const std::vector<Vertex>& vertices,const std::vector< std::vector<int> >& polygons,
						   isoOctree::MeshInfo<Real> &output)
{
	MinimalAreaTriangulation<Real> mat;
	output.triangles.clear();

	std::vector< Point3D<Real> > loop;
	std::vector<int> vertexMap;
	std::vector<TriangleIndex> tgl;

	for(size_t i=0;i<polygons.size();i++)
	{
		loop.clear();
		loop.resize(polygons[i].size());
		vertexMap.clear();
		vertexMap.resize(polygons[i].size());
		tgl.clear();
		for(size_t j=0;j<polygons[i].size();j++)
		{
			loop[j] = vertices[polygons[i][j]].point;
			vertexMap[j]=polygons[i][j];
		}
		mat.GetTriangulation(loop,tgl);

		size_t tSize=output.triangles.size();
		output.triangles.resize(tSize+tgl.size());
		for(size_t j=0;j<tgl.size();j++)
		{
			for(int k=0;k<3;k++)
				output.triangles[tSize+j][k]=vertexMap[tgl[j].idx[k]];
		}
	}
}

template<class Vertex,class Real>
void PolygonToManifoldTriangleMesh( std::vector<Vertex>& vertices , const std::vector< std::vector<int> >& polygons ,
								    isoOctree::MeshInfo<Real> &output)
{
	std::array<isoOctree::MeshIndex, 3> t;
	output.triangles.clear();
	for( int i=0 ; i<polygons.size() ; i++ )
	{
		if( polygons[i].size()==3 ) {
			for (int j=0; j<3; ++j) t[j] = isoOctree::MeshIndex(polygons[i][j]);
			output.triangles.push_back(t);
		}
		else if( polygons[i].size()>3 )
		{
			Point3D< Real > center;
			center *= 0;
			for( int j=0 ; j<polygons[i].size() ; j++ ) center += vertices[ polygons[i][j] ].point;
			center /= polygons[i].size();

			Vertex centerV;
			centerV.point = center;

			int idx = vertices.size();
			vertices.push_back( centerV );
			t[2] = idx;
			for( int j=0 ; j<polygons[i].size() ; j++ )
			{
				t[0] = polygons[i][j];
				t[1] = polygons[i][(j+1)%polygons[i].size()];
				output.triangles.push_back(t);
			}
		}
	}
}
}

namespace isoOctree {

template <class Real>
void buildMesh(typename isoOctree::Octree<Real>::Traverser &traverser, isoOctree::MeshInfo<Real> &output)
{
    static bool isoOctreeCaseInit = false;
    if (!isoOctreeCaseInit) {
        isoOctreeCaseInit = true;
        MarchingCubes::SetCaseTable();
        MarchingCubes::SetFullCaseTable();
    }

	using VertexVal = VertexValue<Real>;
	IsoOctree<NodeDataImpl<VertexVal, Real>, Real, VertexVal> isoTree;
	isoTree.set(traverser);

	log_debug("Nodes In: %d / %d", isoTree.tree.nodes(), isoTree.tree.leaves());
	log_debug("Values In: %zu", isoTree.cornerValues.size());

    bool fullMarchingCubes = true;
    bool manifoldVersion = false;

	std::vector<VertexImpl<Real>> vertices;
	std::vector<std::vector<int> > polygons;
    isoTree.getIsoSurface(0, vertices, polygons, fullMarchingCubes);
    log_debug("Vertices %zu, Polygons: %zu", vertices.size(), polygons.size());

	if(manifoldVersion) {
		PolygonToManifoldTriangleMesh<VertexImpl<Real>, Real>(vertices, polygons, output);
	} else {
		PolygonToTriangleMesh<VertexImpl<Real>, Real>(vertices, polygons, output);
	}

	output.vertices.clear();
	const auto &root = traverser.root();
	for (auto &vertex : vertices) {
		output.vertices.push_back(getMappedCornerPosition<Real>(root, vertex.point[0], vertex.point[1], vertex.point[2]));
	}

    log_debug("Vertices: %zu", output.vertices.size());
    log_debug("Triangles: %zu", output.triangles.size());
}

template <class Real>
void buildMeshWithPointCloudHint(
    const std::function<float(const Point3D<Real> &)> &isoFunction,
    const typename Octree<Real>::PointCloudHint &hint,
    MeshInfo<Real> &output)
{
	log_debug("building mesh from point cloud hint with %zu point(s)", hint.nPoints);
	output.triangles.clear();
	output.vertices.clear();

	if (hint.nPoints == 0) {
		log_warn("empty point cloud hint: returning empty mesh");
		return;
	}

	typename Octree<Real>::RootInfo root;
	root.maxDepth = hint.maxDepth;

	{
		Point3D<Real> minCorner = hint.points[0];
		Point3D<Real> maxCorner = hint.points[0];

		for (std::size_t i = 0; i < hint.nPoints; i++) {
			for (int j = 0; j < 3; j++) {
				minCorner[j] = std::min(minCorner[j], hint.points[i][j]);
				maxCorner[j] = std::max(maxCorner[j], hint.points[i][j]);
			}
		}

		root.width = std::max(
			maxCorner[0] - minCorner[0],
			std::max(
				maxCorner[1] - minCorner[1],
				maxCorner[2] - minCorner[2]));

		if (root.width <= 0) {
			log_warn("point cloud hint has zero width: returning empty mesh");
			return;
		}

		root.center = {
			(minCorner[0] + maxCorner[0]) / 2,
			(minCorner[1] + maxCorner[1]) / 2,
			(minCorner[2] + maxCorner[2]) / 2
		};
	}

	using SearchTree = ZOrderOctree<Point3D<Real>, Real>;

	typename SearchTree::Parameters searchTreeParameters;
	// OK to cap. Just gets slower to search
	searchTreeParameters.rootLevel = std::min(hint.maxDepth + 1, (int)SearchTree::MAX_ROOT_LEVEL);
	searchTreeParameters.leafSize = root.width / (1 << searchTreeParameters.rootLevel);
	searchTreeParameters.origin = root.center;

	SearchTree searchTree(searchTreeParameters);
	searchTree.addData(hint.points, hint.nPoints);

	class TraverserImpl : public Octree<Real>::Traverser {
	private:
		const std::function<float(const Point3D<Real> &)> &f;
		const typename Octree<Real>::RootInfo &rootInfo;
		const int subdivisionThreshold;
		const SearchTree &searchTree;

	public:
		const typename Octree<Real>::RootInfo &root() final { return rootInfo; }

		bool shouldExpand(
			const typename Octree<Real>::Voxel &voxel,
			const typename Octree<Real>::CornerValues &corners) final
		{
			(void)corners;
			Point3D<Real> voxelCenter = {
				Real(0.5) * voxel.width + voxel.minCorner[0],
				Real(0.5) * voxel.width + voxel.minCorner[1],
				Real(0.5) * voxel.width + voxel.minCorner[2]
			};

			// note: searches beyond voxel bounds on purpose
			Real searchRadius = voxel.width;
			std::size_t nMatches = 0;
			for (const auto &match : searchTree.searchWithRadius(voxelCenter, searchRadius)) {
				nMatches++;
			}

			return nMatches >= subdivisionThreshold;
		}

		float isoValue(const Point3D<Real> &point) final {
			return f(point);
		}

		TraverserImpl(
			const std::function<float(const Point3D<Real> &)> &f,
			const typename Octree<Real>::RootInfo &r,
			int subdivisionThreshold,
			const SearchTree &searchTree
		) :
			f(f),
			rootInfo(r),
			subdivisionThreshold(subdivisionThreshold),
			searchTree(searchTree)
		{}
	};

	TraverserImpl traverser(isoFunction, root, hint.subdivisionThreshold, searchTree);
	buildMesh(traverser, output);
}

#define SPECIALIZE(real) \
template void buildMesh<real>( \
    Octree<real>::Traverser &, \
    MeshInfo<real> &); \
template void buildMeshWithPointCloudHint<real>( \
    const std::function<float(const Point3D<real> &)> &, \
    const typename Octree<real>::PointCloudHint &, \
    MeshInfo<real> &)

SPECIALIZE(float);
SPECIALIZE(double);
#undef SPECIALIZE

}

template<class NodeData, class Real, class VertexData>
bool IsoOctree<NodeData, Real, VertexData>::set(typename isoOctree::Octree<Real>::Traverser &traverser)
{
	maxDepth = traverser.root().maxDepth;
    assert(maxDepth > 0);
	typename OctNode<NodeData,Real>::NodeIndex nIdx; // Initialized to zero
	cornerValues.clear();

	setChildren(&tree, nIdx, traverser);

	return true;
}

template<class NodeData, class Real, class VertexData>
void IsoOctree<NodeData, Real, VertexData>::setChildren(
    OctNode<NodeData,Real>* node,
    const typename OctNode<NodeData,Real>::NodeIndex& nIdx,
    typename isoOctree::Octree<Real>::Traverser &traverser)
{
	long long key;
	Real w;
	Point3D<Real> ctr, p;
    const auto &root = traverser.root();

	OctNode<NodeData,Real>::CenterAndWidth(nIdx,ctr,w);

    typename isoOctree::Octree<Real>::CornerValues apiCornerVals;
	// log_debug("diii %d,%d,%d,%d", nIdx.depth, nIdx.offset[0], nIdx.offset[1], nIdx.offset[2]);

	for (int i=0; i<Cube::CORNERS; i++) {
	    key = OctNode<NodeData,Real>::CornerIndex(nIdx, i, root.maxDepth);
		if(cornerValues.find(key) == cornerValues.end()) {
            int x,y,z;
            Cube::FactorCornerIndex(i, x, y, z);
			auto coords = getMappedCornerPosition<Real>(
				root,
				ctr.coords[0] + Real(x - 0.5) * w,
				ctr.coords[1] + Real(y - 0.5) * w,
				ctr.coords[2] + Real(z - 0.5) * w
			);
			cornerValues[key] = VertexData(traverser.isoValue(coords));
			// log_debug("xyz %g,%g,%g (%d,%d,%d,%d), %g/%g", coords[0], coords[1], coords[2], i,x,y,z, w, root.width);
        }

        apiCornerVals[i] = cornerValues.at(key).v;
	}

    if (nIdx.depth == root.maxDepth) return;

    if (!traverser.shouldExpand(convertIdx<Real>(nIdx, root), apiCornerVals)) {
        return;
    }

	if (!node->children) node->initChildren();
    for (int i=0; i<Cube::CORNERS; i++) {
	    key = OctNode<NodeData,Real>::CornerIndex(nIdx, i, root.maxDepth);
		setChildren(&node->children[i], nIdx.child(i), traverser);
    }
}
