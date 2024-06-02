import IsoOctree

def getValue(p):
    x, y, z = p
    # Tanglecube
    return x*x*x*x - 5*x*x + y*y*y*y - 5*y*y + z*z*z*z - 5*z*z + 11.8

def shouldExpand(node, corners):
    BASE_DEPTH = 3
    if node.depth < BASE_DEPTH: return True
    return not (all([v < 0.0 for v in corners]) or all([v > 0.0 for v in corners]))

def writeMeshAsObj(mesh, filename):
    print('writing', filename)
    with open(filename, 'wt') as f:
        for v in mesh.vertices:
            f.write('v %f %f %f\n' % (v[0], v[1], v[2]))
        for t in mesh.triangles:
            f.write('f %d %d %d\n' % (t[0]+1, t[1]+1, t[2]+1))

if __name__ == '__main__':
    mesh = IsoOctree.buildMesh(
        getValue,
        shouldExpand,
        width = 6.0)

    writeMeshAsObj(mesh, 'example.obj')
