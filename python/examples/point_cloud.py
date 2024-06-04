"""
Example meshing with point cloud hints.
Requires: pip install scipy
"""
import IsoOctree
import numpy as np
from scipy.spatial import KDTree
import csv

def buildMesh(points, normals, maxDepth, maxDist, kNearest, subdivisionThreshold):
    tree = KDTree(points)
    md2 = maxDist**2

    def isoValue(point):
        p0 = np.array(list(point))
        _, ii = tree.query(p0, k=kNearest, distance_upper_bound=maxDist)

        ii = [i for i in ii if i < len(points)]

        if len(ii) == 0:
            return 1.0

        sqDists = [np.sum((points[i, :] - p0)**2) for i in ii]

        weights = 1.0 - np.array([min(d2, md2) / md2 for d2 in sqDists])
        return np.sum([weights[i] * np.dot(normals[ii[i], :], p0 - points[ii[i], :]) for i in range(len(ii))])

    return IsoOctree.buildMeshWithPointCloudHint(
        isoValue,
        points,
        maxDepth=maxDepth,
        subdivisionThreshold=subdivisionThreshold)

def readPointCloud(filename):
    with open(filename, 'r') as f:
        reader = csv.DictReader(f)
        rows = [{k: float(v) for k, v in row.items()} for row in reader]

    normals = np.array([[row['n%c' % c] for c in 'xyz'] for row in rows])
    points = np.array([[row[c] for c in 'xyz'] for row in rows])
    return points, normals

def filterPointCloud(points, normals, quantile):
    center = np.mean(points, axis=0)
    dists2 = np.sum((points - center)**2, axis=1)
    index = dists2 < np.quantile(dists2, quantile)
    return points[index, :], normals[index, :]

def writeMeshAsObj(mesh, filename):
    print('writing', filename)
    with open(filename, 'wt') as f:
        for v in mesh.vertices:
            f.write('v %f %f %f\n' % (v[0], v[1], v[2]))
        for t in mesh.triangles:
            f.write('f %d %d %d\n' % (t[0]+1, t[1]+1, t[2]+1))

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument('input_csv_filename')
    parser.add_argument('output_mesh_filename')
    parser.add_argument('--filter_quantile', type=float, default=0.99)
    parser.add_argument('--max_depth', type=int, default=9)
    parser.add_argument('--max_search_distance', type=float, default=0.3)
    parser.add_argument('--k_nearest', type=int, default=30)
    parser.add_argument('--subdivision_threshold', type=int, default=50)
    args = parser.parse_args()

    points, normals = readPointCloud(args.input_csv_filename)
    points, normals = filterPointCloud(points, normals, args.filter_quantile)

    mesh = buildMesh(points, normals,
        maxDepth=args.max_depth,
        maxDist=args.max_search_distance,
        kNearest=args.k_nearest,
        subdivisionThreshold=args.subdivision_threshold)

    writeMeshAsObj(mesh, args.output_mesh_filename)
