import IsoOctree

def getValue(p):
    x, y, z = p
    return x*x*x*x - 5*x*x + y*y*y*y - 5*y*y + z*z*z*z - 5*z*z + 11.8

def shouldExpand(node, corners):
    BASE_DEPTH = 3
    if node.depth < BASE_DEPTH: return True

    nNonNegative = 0
    nNegative = 0
    for v in corners:
        if v < 0.0: nNegative += 1
        else: nNonNegative += 1

    if nNonNegative == 0 or nNegative == 0: return False

    # Note: this logic is rather flawed for adaptive refinement,
    # but results to leaf nodes at different level, which is a good
    # test to the IsoOctree algorithm
    return nNonNegative != nNegative

def writeMeshAsObj(mesh, filename):
    with open(filename, 'wt') as f:
        for v in mesh.vertices:
            f.write('v %f %f %f\n' % (v[0], v[1], v[2]))
        for t in mesh.triangles:
            f.write('f %d %d %d\n' % (t[0]+1, t[1]+1, t[2]+1))

if __name__ == '__main__':
    mesh = IsoOctree.buildMesh(
        getValue,
        shouldExpand,
        center = [0.1, 0.2, 0.3],
        width = 6.0,
        maxDepth = 6)

    writeMeshAsObj(mesh, 'example.obj')