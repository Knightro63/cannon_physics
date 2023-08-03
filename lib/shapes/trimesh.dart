import 'dart:typed_data';
import 'dart:math' as math;
import '../shapes/shape.dart';
import '../math/vec3.dart';
import '../math/transform.dart';
import '../collision/aabb.dart';
import '../utils/octree.dart';
import '../math/quaternion.dart';

final _cb = Vec3();
final _ab = Vec3();

final _va = Vec3();
final _vb = Vec3();
final _vc = Vec3();

/// Trimesh.
/// @example
///     // How to make a mesh with a single triangle
///     final vertices = [
///         0, 0, 0, // vertex 0
///         1, 0, 0, // vertex 1
///         0, 1, 0  // vertex 2
///     ]
///     final indices = [
///         0, 1, 2  // triangle 0
///     ]
///     final trimeshShape = CANNON.Trimesh(vertices, indices)
class Trimesh extends Shape {
  late Float32List vertices;

  /// Array of integers, indicating which vertices each triangle consists of. The length of this array is thus 3 times the number of triangles.
  late Uint16List indices; 
  late Float32List normals;
  /// The local AABB of the mesh.
  AABB aabb = AABB();

  ///References to vertex pairs, making up all unique edges in the trimesh.
  Uint16List? edges;

  /// Local scaling of the mesh. Use .setScale() to set it.
  Vec3 scale = Vec3(1, 1, 1);

  /// The indexed triangles. Use .updateTree() to update it.
  Octree tree = Octree();

  Trimesh(List<double> vertices, List<int> indices):super(type: ShapeType.trimesh) {
    this.vertices = Float32List.fromList(vertices);
    this.indices = Uint16List.fromList(indices);
    normals = Float32List(indices.length);

    updateEdges();
    updateNormals();
    updateAABB();
    updateBoundingSphereRadius();
    updateTree();
  }

  final _computeNormalsN = Vec3();

  final _unscaledAABB = AABB();

  final _getEdgeVectorVa = Vec3();
  final _getEdgeVectorVb = Vec3();

  final _cliAabb = AABB();

  final _computeLocalAABBWorldVert = Vec3();

  final _calculateWorldAABBFrame = Transform();
  final _calculateWorldAABBAabb = AABB();

  void updateTree() {
    final tree = this.tree;

    tree.reset();
    tree.aabb.copy(aabb);
    final scale = this.scale; // The local mesh AABB is scaled, but the octree AABB should be unscaled
    tree.aabb.lowerBound.x *= 1 / scale.x;
    tree.aabb.lowerBound.y *= 1 / scale.y;
    tree.aabb.lowerBound.z *= 1 / scale.z;
    tree.aabb.upperBound.x *= 1 / scale.x;
    tree.aabb.upperBound.y *= 1 / scale.y;
    tree.aabb.upperBound.z *= 1 / scale.z;

    // Insert all triangles
    final triangleAABB = AABB();
    final a = Vec3();
    final b = Vec3();
    final c = Vec3();
    final points = [a, b, c];
    for (int i = 0; i < indices.length / 3; i++) {
      //this.getTriangleVertices(i, a, b, c);

      // Get unscaled triangle verts
      final i3 = i * 3;
      _getUnscaledVertex(indices[i3], a);
      _getUnscaledVertex(indices[i3 + 1], b);
      _getUnscaledVertex(indices[i3 + 2], c);

      triangleAABB.setFromPoints(points);
      tree.insert(triangleAABB, i);
    }
    tree.removeEmptyNodes();
  }

  /// Get triangles in a local AABB from the trimesh.
  /// @param result An array of integers, referencing the queried triangles.
  List<int> getTrianglesInAABB(AABB aabb, List<int> result){
    _unscaledAABB.copy(aabb);

    // Scale it to local
    final scale = this.scale;
    final isx = scale.x;
    final isy = scale.y;
    final isz = scale.z;
    final l = _unscaledAABB.lowerBound;
    final u = _unscaledAABB.upperBound;
    l.x /= isx;
    l.y /= isy;
    l.z /= isz;
    u.x /= isx;
    u.y /= isy;
    u.z /= isz;

    return tree.aabbQuery(_unscaledAABB, result);
  }

  void setScale(Vec3 scale) {
    final wasUniform = this.scale.x == this.scale.y && this.scale.y == this.scale.z;
    final isUniform = scale.x == scale.y && scale.y == scale.z;

    if (!(wasUniform && isUniform)) {
      // Non-uniform scaling. Need to update normals.
      updateNormals();
    }
    this.scale.copy(scale);
    updateAABB();
    updateBoundingSphereRadius();
  }

  /// Compute the normals of the faces. Will save in the `.normals` array.
  void updateNormals() {
    final n = _computeNormalsN;

    // Generate normals
    final normals = this.normals;
    for (int i = 0; i < indices.length / 3; i++) {
      final i3 = i * 3;

      final a = indices[i3];
      final b = indices[i3 + 1];
      final c = indices[i3 + 2];

      getVertex(a, _va);
      getVertex(b, _vb);
      getVertex(c, _vc);

      Trimesh.computeNormal(_vb, _va, _vc, n);

      normals[i3] = n.x;
      normals[i3 + 1] = n.y;
      normals[i3 + 2] = n.z;
    }
  }

  void updateEdges() {
    Map<String,bool> edges = {};// { [key: string]: boolean } = {};
    void add(int a, int b){
      final key = a < b ? '${a}_$b' : '${b}_$a';
      edges[key] = true;
    }

    for (int i = 0; i < indices.length / 3; i++) {
      final i3 = i * 3;
      final a = indices[i3];
      final b = indices[i3 + 1];
      final c = indices[i3 + 2];
      add(a, b);
      add(b, c);
      add(c, a);
    }
    final keys = edges.keys.toList();
    this.edges = Uint16List(keys.length * 2);
    for (int i = 0; i < keys.length; i++) {
      final indices = keys[i].split('_');
      this.edges![2 * i] = int.parse(indices[0], radix:  10);
      this.edges![2 * i + 1] = int.parse(indices[1], radix: 10);
    }
  }

  /// Get an edge vertex
  /// @param firstOrSecond 0 or 1, depending on which one of the vertices you need.
  /// @param vertexStore Where to store the result
  void getEdgeVertex(int edgeIndex, int firstOrSecond, Vec3 vertexStore) {
    final vertexIndex = edges![edgeIndex * 2 + (firstOrSecond >= 1? 1 : 0)];
    getVertex(vertexIndex, vertexStore);
  }

  /// Get a vector along an edge.
  void getEdgeVector(int edgeIndex, Vec3 vectorStore) {
    final va = _getEdgeVectorVa;
    final vb = _getEdgeVectorVb;
    getEdgeVertex(edgeIndex, 0, va);
    getEdgeVertex(edgeIndex, 1, vb);
    vb.vsub(va, vectorStore);
  }

  /// Get face normal given 3 vertices
  static void computeNormal(Vec3 va, Vec3 vb, Vec3 vc, Vec3 target) {
    vb.vsub(va, _ab);
    vc.vsub(vb, _cb);
    _cb.cross(_ab, target);
    if (!target.isZero()) {
      target.normalize();
    }
  }

  /// Get vertex i.
  /// @return The "out" vector object
  Vec3 getVertex(int i, Vec3 out) {
    final scale = this.scale;
    _getUnscaledVertex(i, out);
    out.x *= scale.x;
    out.y *= scale.y;
    out.z *= scale.z;
    return out;
  }

  /// Get raw vertex i
  /// @return The "out" vector object
  Vec3 _getUnscaledVertex(int i, Vec3 out) {
    final i3 = i * 3;
    final vertices = this.vertices;
    return out.set(vertices[i3], vertices[i3 + 1], vertices[i3 + 2]);
  }

  /// Get a vertex from the trimesh,transformed by the given position and quaternion.
  /// @return The "out" vector object
  Vec3 getWorldVertex(int i, Vec3 pos, Quaternion quat, Vec3 out) {
    getVertex(i, out);
    Transform.pointToWorldFrame(pos, quat, out, out);
    return out;
  }

  /// Get the three vertices for triangle i.
  void getTriangleVertices(int i, Vec3 a, Vec3 b, Vec3 c) {
    final i3 = i * 3;
    getVertex(indices[i3], a);
    getVertex(indices[i3 + 1], b);
    getVertex(indices[i3 + 2], c);
  }

  /// Compute the normal of triangle i.
  /// @return The "target" vector object
  Vec3 getNormal(int i, Vec3 target) {
    final i3 = i * 3;
    return target.set(normals[i3], normals[i3 + 1], normals[i3 + 2]);
  }

  /// @return The "target" vector object
  @override
  Vec3 calculateLocalInertia(double mass, Vec3 target) {
    // Approximate with box inertia
    // Exact inertia calculation is overkill, but see http://geometrictools.com/Documentation/PolyhedralMassProperties.pdf for the correct way to do it
    computeLocalAABB(_cliAabb);
    final x = _cliAabb.upperBound.x - _cliAabb.lowerBound.x;
    final y = _cliAabb.upperBound.y - _cliAabb.lowerBound.y;
    final z = _cliAabb.upperBound.z - _cliAabb.lowerBound.z;
    return target.set(
      (1.0 / 12.0) * mass * (2 * y * 2 * y + 2 * z * 2 * z),
      (1.0 / 12.0) * mass * (2 * x * 2 * x + 2 * z * 2 * z),
      (1.0 / 12.0) * mass * (2 * y * 2 * y + 2 * x * 2 * x)
    );
  }

  /// Compute the local AABB for the trimesh
  void computeLocalAABB(AABB aabb) {
    final l = aabb.lowerBound;
    final u = aabb.upperBound;
    final n = vertices.length;
    final v = _computeLocalAABBWorldVert;

    getVertex(0, v);
    l.copy(v);
    u.copy(v);

    for (int  i = 0; i != n; i++) {
      getVertex(i, v);

      if (v.x < l.x) {
        l.x = v.x;
      } else if (v.x > u.x) {
        u.x = v.x;
      }

      if (v.y < l.y) {
        l.y = v.y;
      } else if (v.y > u.y) {
        u.y = v.y;
      }

      if (v.z < l.z) {
        l.z = v.z;
      } else if (v.z > u.z) {
        u.z = v.z;
      }
    }
  }

  /// Update the `.aabb` property
  void updateAABB() {
    computeLocalAABB(aabb);
  }

  /// Will update the `.boundingSphereRadius` property
  @override
  void updateBoundingSphereRadius() {
    // Assume points are distributed with local (0,0,0) as center
    double max2 = 0;
    final vertices = this.vertices;
    final v = Vec3();
    for (int i = 0, N = vertices.length ~/ 3; i != N; i++) {
      getVertex(i, v);
      final norm2 = v.lengthSquared();
      if (norm2 > max2) {
        max2 = norm2;
      }
    }
    boundingSphereRadius = math.sqrt(max2);
  }

  /// calculateWorldAABB
  @override
  void calculateWorldAABB(Vec3 pos, Quaternion quat, Vec3 min, Vec3 max) {
    /*
        final n = this.vertices.length / 3,
            verts = this.vertices;
        final minx,miny,minz,maxx,maxy,maxz;

        final v = tempWorldVertex;
        for(let i=0; i<n; i++){
            this.getVertex(i, v);
            quat.vmult(v, v);
            pos.vadd(v, v);
            if (v.x < minx || minx===undefined){
                minx = v.x;
            } else if(v.x > maxx || maxx===undefined){
                maxx = v.x;
            }

            if (v.y < miny || miny===undefined){
                miny = v.y;
            } else if(v.y > maxy || maxy===undefined){
                maxy = v.y;
            }

            if (v.z < minz || minz===undefined){
                minz = v.z;
            } else if(v.z > maxz || maxz===undefined){
                maxz = v.z;
            }
        }
        min.set(minx,miny,minz);
        max.set(maxx,maxy,maxz);
        */

    // Faster approximation using local AABB
    final frame = _calculateWorldAABBFrame;
    final result = _calculateWorldAABBAabb;
    frame.position = pos;
    frame.quaternion = quat;
    aabb.toWorldFrame(frame, result);
    min.copy(result.lowerBound);
    max.copy(result.upperBound);
  }

  /// Get approximate volume
  @override
  double volume() {
    return (4.0 * math.pi * boundingSphereRadius) / 3.0;
  }

  /// Create a Trimesh instance, shaped as a torus.
  static Trimesh createTorus([double radius = 1, double tube = 0.5, int radialSegments = 8, int tubularSegments = 6, double arc = math.pi * 2]){
    List<double> vertices = [];
    List<int> indices = [];

    for (int j = 0; j <= radialSegments; j++) {
      for (int i = 0; i <= tubularSegments; i++) {
        final u = (i / tubularSegments) * arc;
        final v = (j / radialSegments) * math.pi * 2;

        final x = (radius + tube * math.cos(v)) * math.cos(u);
        final y = (radius + tube * math.cos(v)) * math.sin(u);
        final z = tube * math.sin(v);

        vertices.add(x);
        vertices.add(y);
        vertices.add(z);
      }
    }

    for (int j = 1; j <= radialSegments; j++) {
      for (int i = 1; i <= tubularSegments; i++) {
        final a = (tubularSegments + 1) * j + i - 1;
        final b = (tubularSegments + 1) * (j - 1) + i - 1;
        final c = (tubularSegments + 1) * (j - 1) + i;
        final d = (tubularSegments + 1) * j + i;

        indices.add(a);
        indices.add(b);
        indices.add(d);

        indices.add(b);
        indices.add(c);
        indices.add(d);
      }
    }

    return Trimesh(vertices, indices);
  }
}