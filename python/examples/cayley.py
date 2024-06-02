import IsoOctree
import numpy as np

def getValue(p):
    x, y, z = p

    # Cayley cubic
    return x**2 + y**2 - x**2*z + y**2*z + z**2 - 1

def getGradient(p):
    x, y, z = p

    dx = 2*x - 2*x*z
    dy = 2*y + 2*y*z
    dz = -x**2 + y**2 + 2*z

    return np.array([dx, dy, dz])

def getMinDot(node):
    h = node.width / 2
    p = (node.minCorner[0] + h, node.minCorner[1] + h, node.minCorner[2] + h)
    normals = []
    for dx in [-1, 1]:
        for dy in [-1, 1]:
            for dz in [-1, 1]:
                p2 = (p[0] + dx * h, p[1] + dy * h, p[2] + dz * h)
                grad = getGradient(p2)
                normal = grad / max(np.linalg.norm(grad), 1e-6)
                normals.append(normal)

    mean_normal = np.mean(normals, axis=0)
    mean_normal = mean_normal / max(np.linalg.norm(mean_normal), 1e-6)
    min_dot = np.max([np.dot(mean_normal, n) for n in normals])
    return min_dot

MIN_DOT_THR = np.cos(3 / 180.0 * np.pi)

def shouldExpand(node, cornerValues):
    # expand all very big nodes
    if node.depth < 3: return True

    # early culling for voxels that seem to be far from the surface
    min_v = min(cornerValues)
    max_v = max(cornerValues)
    if min_v * max_v > 0 and min(abs(max_v), abs(min_v)) > max(abs(max_v), abs(min_v)) * 0.5:
        return False

    if node.depth < 5: return True

    # refine high-curvature nodes near the surface
    return getMinDot(node) < MIN_DOT_THR

def refineVertices(vertices, iterations=5):
    for _ in range(iterations):
        x, y, z = [vertices[:, i] for i in range(3)]
        grad = getGradient((x, y, z)).T
        value = getValue((x, y, z))
        vertices = vertices - grad * ((value / np.sum(grad**2, axis=1)))[:, np.newaxis]

    return vertices

def writeMeshAsObj(vertices, triangles, filename):
    print('writing', filename)
    with open(filename, 'wt') as f:
        for v in vertices:
            f.write('v %f %f %f\n' % (v[0], v[1], v[2]))
        for t in triangles:
            f.write('f %d %d %d\n' % (t[0]+1, t[1]+1, t[2]+1))

if __name__ == '__main__':
    mesh = IsoOctree.buildMesh(
        getValue,
        shouldExpand,
        center = [0.1, 0.2, 0.3],
        width = 6.0,
        maxDepth = 10)

    writeMeshAsObj(refineVertices(mesh.vertices), mesh.triangles, 'cayley.obj')