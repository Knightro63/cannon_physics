import 'dart:math' as math;
import 'shape.dart';
import '../math/vec3.dart';
import '../math/transform.dart';
import '../collision/aabb.dart';
import '../utils/octree.dart';
import 'package:vector_math/vector_math.dart';

class TorusGeometry{
  TorusGeometry([
    this.radius = 1,
    this.tube = 0.5,
    this.radialSegments = 8,
    this.tubularSegments = 6,
    this.arc = math.pi * 2
  ]);

  final double arc;
  final double radius;
  final double tube;
  final int radialSegments;
  final int tubularSegments;
}

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
  late final List<double> vertices;

  /// Array of integers, indicating which vertices each triangle consists of. The length of this array is thus 3 times the number of triangles.
  late final List<int> indices; 
  late final List<double>? normals;
  late final List<double> faceNormals;
  late final List<double>? uvs;

  /// The local AABB of the mesh.
  final AABB aabb = AABB();
  final AABB unscaledAABB = AABB();

  ///References to vertex pairs, making up all unique edges in the trimesh.
  List<int> edges = [];

  /// Local scaling of the mesh. Use .setScale() to set it.
  final Vector3 scale = Vector3(1, 1, 1);

  /// The indexed triangles. Use .updateTree() to update it.
  final Octree tree = Octree();

  late final TorusGeometry torus;

  Trimesh(
    this.vertices, 
    this.indices, 
    [
      this.normals, 
      this.uvs
    ]
  ):super(type: ShapeType.trimesh) {
    updateEdges();
    updateNormals();
    
    updateAABB();
    updateBoundingSphereRadius();
    updateTree();
  }

  final _computeNormalsN = Vector3.zero();

  final _getEdgeVectorVa = Vector3.zero();
  final _getEdgeVectorVb = Vector3.zero();

  final _cliAabb = AABB();

  final _computeLocalAABBWorldVert = Vector3.zero();

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
    final a = Vector3.zero();
    final b = Vector3.zero();
    final c = Vector3.zero();
    final points = [a, b, c];
    for (int i = 0; i < indices.length / 3; i++) {
      // Get unscaled triangle verts
      int i3 = i * 3;
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
    unscaledAABB.copy(aabb);// = aabb.clone();

    // Scale it to local
    final scale = this.scale;
    final isx = scale.x;
    final isy = scale.y;
    final isz = scale.z;
    final l = unscaledAABB.lowerBound;
    final u = unscaledAABB.upperBound;
    l.x /= isx;
    l.y /= isy;
    l.z /= isz;
    u.x /= isx;
    u.y /= isy;
    u.z /= isz;

    return tree.aabbQuery(unscaledAABB, result);
  }

  void setScale(Vector3 scale) {
    final wasUniform = this.scale.x == this.scale.y && this.scale.y == this.scale.z;
    final isUniform = scale.x == scale.y && scale.y == scale.z;

    if (!(wasUniform && isUniform)) {
      // Non-uniform scaling. Need to update normals.
      updateNormals();
    }
    this.scale.setFrom(scale);
    updateAABB();
    updateBoundingSphereRadius();
  }

  final _va = Vector3.zero();
  final _vb = Vector3.zero();
  final _vc = Vector3.zero();
  /// Compute the normals of the faces. Will save in the `.normals` array.
  void updateNormals() {
    // Generate normals
    //final normals = faceNormals;
    faceNormals = [];
    final n = _computeNormalsN;
    for (int i = 0; i < indices.length / 3; i++) {
      final i3 = i * 3;

      final a = indices[i3];
      final b = indices[i3 + 1];
      final c = indices[i3 + 2];

      getVertex(a, _va);
      getVertex(b, _vb);
      getVertex(c, _vc);

      Trimesh.computeNormal(_vb, _va, _vc, n);
      faceNormals.addAll([n.x,n.y,n.z]);
    }
  }

  void updateEdges() {
    Map<String,bool> edges = {};
    void add(int a, int b){
      final key = a < b ? '${a}_$b' : '${b}_$a';
      edges[key] = true;
    }
    for (int i = 0; i < indices.length / 3; i++) {
      int i3 = i * 3;
      final a = indices[i3];
      final b = indices[i3 + 1];
      final c = indices[i3 + 2];
      add(a, b);
      add(b, c);
      add(c, a);
    }

    this.edges = List.filled(edges.length * 2,0);
    int i = 0;
    for (String ind in edges.keys) {
      List<String> indices = ind.split('_');
      this.edges[2 * i] = int.parse(indices[0]);
      this.edges[2 * i + 1] = int.parse(indices[1]);
      i++;
    }
  }

  /// Get an edge vertex
  /// @param firstOrSecond 0 or 1, depending on which one of the vertices you need.
  /// @param vertexStore Where to store the result
  void getEdgeVertex(int edgeIndex, int firstOrSecond, Vector3 vertexStore) {
    final vertexIndex = edges[edgeIndex * 2 + firstOrSecond];
    getVertex(vertexIndex, vertexStore);
  }

  /// Get a vector along an edge.
  void getEdgeVector(int edgeIndex, Vector3 vectorStore) {
    final va = _getEdgeVectorVa;
    final vb = _getEdgeVectorVb;
    getEdgeVertex(edgeIndex, 0, va);
    getEdgeVertex(edgeIndex, 1, vb);
    vb.sub2(va, vectorStore);
  }

  /// Get face normal given 3 vertices
  static void computeNormal(Vector3 va, Vector3 vb, Vector3 vc, Vector3 target) {
    final cb = Vector3.zero();
    final ab = Vector3.zero();

    vb.sub2(va, ab);
    vc.sub2(vb, cb);
    cb.cross2(ab, target);
    if (!target.isZero()) {
      target.normalize();
    }
  }

  /// Get vertex i.
  /// @return The "out" vector object
  Vector3 getVertex(int i, [Vector3? out]) {
    out ??= Vector3.zero();
    final scale = this.scale;
    _getUnscaledVertex(i, out);
    out.x *= scale.x;
    out.y *= scale.y;
    out.z *= scale.z;
    return out;
  }

  /// Get raw vertex i
  /// @return The "out" vector object
  Vector3 _getUnscaledVertex(int i, Vector3 out) {
    final i3 = i * 3;
    final vertices = this.vertices;
    return out..setValues(vertices[i3], vertices[i3 + 1], vertices[i3 + 2]);
  }

  /// Get a vertex from the trimesh,transformed by the given position and quaternion.
  /// @return The "out" vector object
  Vector3 getWorldVertex(int i, Vector3 pos, Quaternion quat, Vector3 out) {
    getVertex(i, out);
    Transform.pointToWorldFrame(pos, quat, out, out);
    return out;
  }

  /// Get the three vertices for triangle i.
  void getTriangleVertices(int i, Vector3 a, Vector3 b, Vector3 c) {
    final i3 = i * 3;
    getVertex(indices[i3], a);
    getVertex(indices[i3 + 1], b);
    getVertex(indices[i3 + 2], c);
  }
  /// Get the three vertices for triangle i.
  void getTriangleNormals(int i, Vector3 a, Vector3 b, Vector3 c) {
    final i3 = i * 3;
    getIndicesNormal(indices[i3], a);
    getIndicesNormal(indices[i3 + 1], b);
    getIndicesNormal(indices[i3 + 2], c);
  }
  Vector3 getFaceNormal(int i, Vector3 target) {
    final i3 = i * 3;
    final Vector3 a = Vector3.zero();
    final Vector3 b = Vector3.zero();
    final Vector3 c = Vector3.zero();
    getIndicesNormal(indices[i3], a);
    getIndicesNormal(indices[i3 + 1], b);
    getIndicesNormal(indices[i3 + 2], c);
    return target..setValues(
      (a.x+b.x+c.x)/3,
      (a.y+b.y+c.y)/3,
      (a.z+b.z+c.z)/3
    );
  }
  /// Compute the normal of triangle i.
  /// @return The "target" vector object
  // Vector3 getNormal(int i, Vector3 target) {
  //   final i3 = i * 3;
  //   return target.set(normals![i3], normals![i3 + 1], normals![i3 + 2]);
  // }
  /// Compute the normal of triangle i.
  /// @return The "target" vector object
  Vector3 getIndicesNormal(int i, Vector3 target) {
    final i3 = i * 3;
    return target..setValues(faceNormals[i3], faceNormals[i3 + 1], faceNormals[i3 + 2]);
  }
  /// @return The "target" vector object
  @override
  Vector3 calculateLocalInertia(double mass, Vector3 target) {
    // Approximate with box inertia
    // Exact inertia calculation is overkill, but see http://geometrictools.com/Documentation/PolyhedralMassProperties.pdf for the correct way to do it
    computeLocalAABB(_cliAabb);
    final x = _cliAabb.upperBound.x - _cliAabb.lowerBound.x;
    final y = _cliAabb.upperBound.y - _cliAabb.lowerBound.y;
    final z = _cliAabb.upperBound.z - _cliAabb.lowerBound.z;
    return target..setValues(
      (1.0 / 12.0) * mass * (2*y*2*y + 2*z*2*z),
      (1.0 / 12.0) * mass * (2*x*2*x + 2*z*2*z),
      (1.0 / 12.0) * mass * (2*y*2*y + 2*x*2*x)
    );
  }

  /// Compute the local AABB for the trimesh
  void computeLocalAABB(AABB aabb) {
    final l = aabb.lowerBound;
    final u = aabb.upperBound;
    final v = _computeLocalAABBWorldVert;
    getVertex(0, v);
    l.setFrom(v);
    u.setFrom(v);
    for (int  i = 0; i < vertices.length/3; i++) {//!= n
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
    final v = Vector3.zero();
    for (int i = 0; i < vertices.length/ 3; i++) {
      getVertex(i, v);
      double norm2 = v.length2;
      if (norm2 > max2) {
        max2 = norm2;
      }
    }
    boundingSphereRadius = math.sqrt(max2);
  }

  /// calculateWorldAABB
  @override
  void calculateWorldAABB(Vector3 pos, Quaternion quat, Vector3 min, Vector3 max) {
    // Faster approximation using local AABB
    final frame = _calculateWorldAABBFrame;
    final result = _calculateWorldAABBAabb;
    frame.position = pos;
    frame.quaternion = quat;
    aabb.toWorldFrame(frame, result);
    min.setFrom(result.lowerBound);
    max.setFrom(result.upperBound);
  }

  /// Get approximate volume
  @override
  double volume() {
    return (4.0 * math.pi * boundingSphereRadius) / 3.0;
  }

  /// Create a Trimesh instance, shaped as a torus.
  Trimesh.createTorus(this.torus):super(type: ShapeType.trimesh){
    final List<double> vertices = [];
    List<double> normals = [];
    List<int> indices = [];
    List<double> uvs = [];

    for (int j = 0; j <= torus.radialSegments; j++) {
      for (int i = 0; i <= torus.tubularSegments; i++) {
          final center = Vector3.zero();
          final vertex = Vector3.zero();
          final normal = Vector3.zero();
          
          final u = i / torus.tubularSegments * torus.arc;
          final v = j / torus.radialSegments * math.pi * 2;

          vertex.setValues(
            (torus.radius + torus.tube * math.cos(v)) * math.cos(u),
            (torus.radius + torus.tube * math.cos(v)) * math.sin(u),
            torus.tube * math.sin(v)
          );

          vertices.addAll([vertex.x, vertex.y, vertex.z]);

          center.setValues(torus.radius * math.cos(u),torus.radius * math.sin(u),0);
          normal..setFrom(vertex)..sub(center)..normalize();
          normals.addAll([normal.x, normal.y, normal.z]);

          uvs.add(i / torus.tubularSegments);
          uvs.add(j / torus.radialSegments);

        if(i != 0 && j != 0){
          final a = (torus.tubularSegments + 1) * j + i - 1;
          final b = (torus.tubularSegments + 1) * (j - 1) + i - 1;
          final c = (torus.tubularSegments + 1) * (j - 1) + i;
          final d = (torus.tubularSegments + 1) * j + i;

          indices.addAll([a,b,d,b,c,d]);
        }
      }
    }
    this.vertices = vertices;
    this.indices = indices;
    this.uvs = uvs;
    this.normals = normals;
    
    updateNormals();
    updateEdges();
    updateAABB();
    updateBoundingSphereRadius();
    updateTree();
  }
}