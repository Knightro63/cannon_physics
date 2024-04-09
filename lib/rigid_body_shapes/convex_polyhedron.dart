import 'dart:math' as math;
import 'package:cannon_physics/cannon_physics.dart';
import 'package:cannon_physics/utils/logger.dart';
import 'package:vector_math/vector_math.dart';

final _projectLocalAxis = Vector3.zero();
final _projectLocalOrigin = Vector3.zero();
// final _projectWorldVertex = Vector3.zero();

class ConvexPolyhedronContactPoint{
  const ConvexPolyhedronContactPoint(this.point,this.normal,this.depth);
  final Vector3 point;
  final Vector3 normal;
  final double depth;
}

class Polygon{
  Polygon(
    this.faces,
    this.connectedFaces
  );
  List<int> connectedFaces;
  List<int> faces;

  int get length => faces.length;
  int indexOf(int element){
    return faces.indexOf(element);
  }
  //void operator []=(int addr, int value) => puts(addr,Uint8List.fromList([value]));
  int operator [](int addr) => faces[addr];

  bool contains(Object? element) => faces.contains(element);
}

/// A set of polygons describing a convex shape.
///
/// The shape MUST be convex for the code to work properly. No polygons may be coplanar (contained
/// in the same 3D plane), instead these should be merged into one polygon.
///
/// @author qiao / https://github.com/qiao (original author, see https://github.com/qiao/three.js/commit/85026f0c769e4000148a67d45a9e9b9c5108836f)
/// @author schteppe / https://github.com/schteppe
/// @see https://www.altdevblogaday.com/2011/05/13/contact-generation-between-3d-convex-meshes/
///
/// @todo Move the clipping functions to ContactGenerator?
/// @todo Automatically merge coplanar polygons in constructor.
/// @example
///     const convexShape = CANNON.ConvexPolyhedron({ vertices, faces })
///     const convexBody = CANNON.Body({ mass: 1, shape: convexShape })
///     world.addBody(convexBody)

class ConvexPolyhedron extends Shape {
  late List<Vector3> vertices = [];
  /// Array of integer arrays, indicating which vertices each face consists of
  late List<List<int>>faces;
  late List<Vector3?> faceNormals;
  late List<Vector3> worldVertices;
  late bool worldVerticesNeedsUpdate;
  late List<Vector3> worldFaceNormals;
  late bool worldFaceNormalsNeedsUpdate;
  /// If given, these locally defined, normalized axes are the only ones being checked when doing separating axis check.
  List<Vector3>? uniqueAxes;
  late List<Vector3> uniqueEdges;

  /// @param vertices An array of Vector3's
  /// @param faces Array of integer arrays, describing which vertices that is included in each face.
  ConvexPolyhedron({
    List<Vector3>?  vertices,
    List<List<int>>? faces,
    List<Vector3>? normals,
    List<Vector3>? axes,
    double? boundingSphereRadius,
    ShapeType type = ShapeType.convex
  }):super(type: type){
    init(vertices,faces,normals,axes,boundingSphereRadius);
  }

  // final Vector3 _convexPolyhedronPointIsInside = Vector3.zero();
  // final Vector3 _convexPolyhedronVToP = Vector3.zero();
  // final Vector3 _convexPolyhedronVToPointInside = Vector3.zero();
  void init(
    List<Vector3>?  vertices,
    List<List<int>>? faces,
    List<Vector3?>? normals,
    List<Vector3>? axes,
    double? boundingSphereRadius
  ){
    //const { vertices = [], faces = [], normals = [], axes, boundingSphereRadius } = props
    this.vertices = vertices ?? [];
    faceNormals = normals ?? [];
    this.faces = faces ?? [];
    if (faceNormals.isEmpty) {
      computeNormals();
    }
    if (boundingSphereRadius == null) {
      updateBoundingSphereRadius();
    } else {
      this.boundingSphereRadius = boundingSphereRadius;
    }

    worldVertices = []; // World transformed version of .vertices
    worldVerticesNeedsUpdate = true;
    worldFaceNormals = []; // World transformed version of .faceNormals
    worldFaceNormalsNeedsUpdate = true;
    uniqueAxes = axes;
    uniqueEdges = [];
    computeEdges();
  }

  /// Computes uniqueEdges
  void computeEdges() {
    final List<List<int>> faces = this.faces;
    final List<Vector3> vertices = this.vertices;
    final List<Vector3> edges = uniqueEdges;

    edges.clear();

    Vector3 edge = Vector3.zero();

    for (int i = 0; i != faces.length; i++) {
      final List<int> face = faces[i];
      int numVertices = face.length;
      for (int j = 0; j != numVertices; j++) {
        int k = (j + 1) % numVertices;
        vertices[face[j]].sub2(vertices[face[k]], edge);
        edge.normalize();
        bool found = false;
        for (int p = 0; p != edges.length; p++) {
          if (edges[p].almostEquals(edge) || edges[p].almostEquals(edge)) {
            found = true;
            break;
          }
        }

        if (!found) {
          edges.add(edge.clone());
        }
      }
    }
  }

  /// Compute the normals of the faces.
  /// Will reuse existing Vector3 objects in the `faceNormals` array if they exist.
  void computeNormals() {
    faceNormals.length = faces.length;
    // Generate normals
    for (int i = 0; i < faces.length; i++) {
      final Vector3 n = faceNormals[i] ?? Vector3.zero();
      getFaceNormal(i, n);
      n.negate();
      faceNormals[i] = n;
      final Vector3 vertex = vertices[faces[i][0]];
      if (n.dot(vertex) < 0) {
        logger?.warning(
          '.faceNormals[$i] = Vector3(${n.toString()}) looks like it points into the shape? The vertices follow. Make sure they are ordered CCW around the normal, using the right hand rule.'
        );
        int len =faces[i].length;
        for (int j = 0; j < len; j++) {
          logger?.verbose('.vertices[${faces[i][j]}] = Vector3(${vertices[faces[i][j]].toString()})');
        }
      }
    }
  }

  /// Compute the normal of a face from its vertices
  Vector3 getFaceNormal(int i, final Vector3 target) {
    final List<int> f = faces[i];
    final Vector3 va = vertices[f[0]];
    final Vector3 vb = vertices[f[1]];
    final Vector3 vc = vertices[f[2]];
    return ConvexPolyhedron.computeNormal(va, vb, vc, target);
  }

  /// Get face normal given 3 vertices
  static Vector3 computeNormal(final Vector3 va, final Vector3 vb, final Vector3 vc, final Vector3 target) {
    final cb = Vector3.zero();
    final ab = Vector3.zero();
    vb.sub2(va, ab);
    vc.sub2(vb, cb);
    cb.cross2(ab, target);
    if (!target.isZero()) {
      target.normalize();
    }

    return target;
  }

  /// @param minDist Clamp distance
  /// @param result The an array of contact point objects, see clipFaceAgainstHull
  void clipAgainstHull(
    final Vector3 posA,
    final Quaternion quatA,
    final ConvexPolyhedron hullB ,
    final Vector3 posB,
    final Quaternion quatB,
    final Vector3 separatingNormal,
    final double minDist,
    final double maxDist,
    final List<ConvexPolyhedronContactPoint> result
  ) {
    final worldNormal = Vector3.zero();
    int closestFaceB = -1;
    double dmax = -double.infinity;
    for (int face = 0; face < hullB.faces.length; face++) {
      worldNormal.setFrom(hullB.faceNormals[face]!);
      quatB.vmult(worldNormal, worldNormal);
      final double d = worldNormal.dot(separatingNormal);
      if (d > dmax) {
        dmax = d;
        closestFaceB = face;
      }
    }

    List<Vector3> worldVertsB1 = [];
    int len = hullB.faces.isEmpty?0:hullB.faces[closestFaceB].length;
    for (int i = 0; i < len; i++) {
      final b = hullB.vertices[hullB.faces[closestFaceB][i]];
      final worldb = Vector3.zero();
      worldb.setFrom(b);
      quatB.vmult(worldb, worldb);
      posB.add2(worldb, worldb);
      worldVertsB1.add(worldb);
    }

    if (closestFaceB >= 0) {
      clipFaceAgainstHull(separatingNormal, posA, quatA, worldVertsB1, minDist, maxDist, result);
    }
  }

  /// Find the separating axis between this hull and another
  /// @param target The target vector to save the axis in
  /// @return Returns false if a separation is found, else true
  bool findSeparatingAxis(
    ConvexPolyhedron hullB,
    Vector3 posA,
    Quaternion quatA,
    Vector3 posB,
    Quaternion quatB,
    final Vector3 target,[
    List<int>? faceListA,
    List<int>? faceListB
  ]) {
    final faceANormalWS3 = Vector3.zero();
    final worldNormal1 = Vector3.zero();
    final deltaC = Vector3.zero();
    final worldEdge0 = Vector3.zero();
    final worldEdge1 = Vector3.zero();
    final cross = Vector3.zero();

    double dmin = double.infinity;
    final hullA = this;
    //int curPlaneTests = 0;

    if (hullA.uniqueAxes != null) {
      final numFacesA = faceListA != null? faceListA.length : hullA.faces.length;

      // Test face normals from hullA
      for (int i = 0; i < numFacesA; i++) {
        final fi = faceListA != null? faceListA[i] : i;

        // Get world face normal
        faceANormalWS3.setFrom(hullA.faceNormals[fi]!);
        quatA.vmult(faceANormalWS3, faceANormalWS3);

        final d = hullA.testSepAxis(faceANormalWS3, hullB, posA, quatA, posB, quatB);
        if (d == null) {
          return false;
        }
        if (d < dmin) {
          dmin = d;
          target.setFrom(faceANormalWS3);
        }
      }
    } else if(hullA.uniqueAxes != null){
      // Test unique axes
      for (int i = 0; i != hullA.uniqueAxes!.length; i++) {
        // Get world axis
        quatA.vmult(hullA.uniqueAxes![i], faceANormalWS3);

        final d = hullA.testSepAxis(faceANormalWS3, hullB, posA, quatA, posB, quatB);
        if (d == null) {
          return false;
        }
        if (d < dmin) {
          dmin = d;
          target.setFrom(faceANormalWS3);
        }
      }
    }

    if (hullB.uniqueAxes !=  null) {
      // Test face normals from hullB
      final numFacesB = faceListB != null ? faceListB.length : hullB.faces.length;
      for (int i = 0; i < numFacesB; i++) {
        final fi = faceListB != null? faceListB[i] : i;

        worldNormal1.setFrom(hullB.faceNormals[fi]!);
        quatB.vmult(worldNormal1, worldNormal1);
        //curPlaneTests++;
        final d = hullA.testSepAxis(worldNormal1, hullB, posA, quatA, posB, quatB);
        if (d == null) {
          return false;
        }
        if (d < dmin) {
          dmin = d;
          target.setFrom(worldNormal1);
        }
      }
    } else {
      // Test unique axes in B
      final int len = hullB.uniqueAxes?.length ?? 0;
      for (int i = 0; i != len; i++) {
        quatB.vmult(hullB.uniqueAxes![i], worldNormal1);

        //curPlaneTests++;
        final d = hullA.testSepAxis(worldNormal1, hullB, posA, quatA, posB, quatB);
        if (d == null) {
          return false;
        }
        if (d < dmin) {
          dmin = d;
          target.setFrom(worldNormal1);
        }
      }
    }

    // Test edges
    for (int e0 = 0; e0 != hullA.uniqueEdges.length; e0++) {
      // Get world edge
      quatA.vmult(hullA.uniqueEdges[e0], worldEdge0);

      for (int e1 = 0; e1 != hullB.uniqueEdges.length; e1++) {
        // Get world edge 2
        quatB.vmult(hullB.uniqueEdges[e1], worldEdge1);
        worldEdge0.cross2(worldEdge1, cross);

        if (!cross.almostZero()) {
          cross.normalize();
          final dist = hullA.testSepAxis(cross, hullB, posA, quatA, posB, quatB);
          if (dist == null) {
            return false;
          }
          if (dist < dmin) {
            dmin = dist;
            target.setFrom(cross);
          }
        }
      }
    }

    posB.sub2(posA, deltaC);
    if (deltaC.dot(target) > 0.0) {
      target.negate();
    }

    return true;
  }

  /// Test separating axis against two hulls. Both hulls are projected onto the axis and the overlap size is returned if there is one.
  /// @return The overlap depth, or FALSE if no penetration.
  double? testSepAxis(
    Vector3 axis,
    ConvexPolyhedron hullB,
    Vector3 posA,
    Quaternion quatA,
    Vector3 posB,
    Quaternion quatB 
  ){
    final hullA = this;
    final List<double> maxminA = [0,0];
    final List<double> maxminB = [0,0];
    ConvexPolyhedron.project(hullA, axis, posA, quatA, maxminA);
    ConvexPolyhedron.project(hullB, axis, posB, quatB, maxminB);
    final maxA = maxminA[0];
    final minA = maxminA[1];
    final maxB = maxminB[0];
    final minB = maxminB[1];
    if (maxA < minB || maxB < minA) {
      return null; // Separated
    }
    final double d0 = maxA - minB;
    final d1 = maxB - minA;
    final double depth = d0 < d1 ? d0 : d1;
    return depth;
  }

  final aabbmax = Vector3.zero();
  final aabbmin = Vector3.zero();
  @override
  Vector3 calculateLocalInertia(double mass, final Vector3 target) {
    // Approximate with box inertia
    // Exact inertia calculation is overkill, but see http://geometrictools.com/Documentation/PolyhedralMassProperties.pdf for the correct way to do it

    computeLocalAABB(aabbmin, aabbmax);
    final x = aabbmax.x - aabbmin.x;
    final y = aabbmax.y - aabbmin.y;
    final z = aabbmax.z - aabbmin.z;
    target.x = (1.0 / 12.0) * mass * (2 * y * 2 * y + 2 * z * 2 * z);
    target.y = (1.0 / 12.0) * mass * (2 * x * 2 * x + 2 * z * 2 * z);
    target.z = (1.0 / 12.0) * mass * (2 * y * 2 * y + 2 * x * 2 * x);

    return target;
  }

  /// @param face_i Index of the face
  double getPlaneConstantOfFace(int faceI) {
    final f = faces[faceI];
    final n = faceNormals[faceI]!;
    final v = vertices[f[0]];
    final c = -n.dot(v);
    return c;
  }

  /// Clip a face against a hull.
  /// @param worldVertsB1 An array of Vector3 with vertices in the world frame.
  /// @param minDist Distance clamping
  /// @param Array result Array to store resulting contact points in. Will be objects with properties: point, depth, normal. These are represented in world coordinates.
  void clipFaceAgainstHull(
    Vector3 separatingNormal,
    Vector3 posA,
    Quaternion quatA,
    List<Vector3> worldVertsB1,
    double minDist,
    double maxDist,
    final List<ConvexPolyhedronContactPoint> result
  ) {
    final faceANormalWS = Vector3.zero();
    final edge0 = Vector3.zero();
    final worldEdge0 = Vector3.zero();
    final worldPlaneAnormal1 = Vector3.zero();
    final planeNormalWS1 = Vector3.zero();
    final worldA1 = Vector3.zero();
    final localPlaneNormal = Vector3.zero();
    final planeNormalWS = Vector3.zero();
    final hullA = this;
    List<Vector3> worldVertsB2 = [];
    final pVtxIn = worldVertsB1;
    final pVtxOut = worldVertsB2;

    int closestFaceA = -1;
    double dmin = double.infinity;

    // Find the face with normal closest to the separating axis
    for (int face = 0; face < hullA.faces.length; face++) {
      faceANormalWS.setFrom(hullA.faceNormals[face]!);
      quatA.vmult(faceANormalWS, faceANormalWS);
      final double d = faceANormalWS.dot(separatingNormal);
      if (d < dmin) {
        dmin = d;
        closestFaceA = face;
      }
    }
    if (closestFaceA < 0) {
      return;
    }

    // Get the face and construct connected faces
    final Polygon polyA = Polygon(hullA.faces[closestFaceA],[]);// as number[] & { connectedFaces: number[] }
    polyA.connectedFaces = [];
    for (int i = 0; i < hullA.faces.length; i++) {
      int len = hullA.faces[i].length;
      for (int j = 0; j < len; j++) {
        if (
          /* Sharing a vertex*/
          polyA.contains(hullA.faces[i][j]) &&
          /* Not the one we are looking for connections from */
          i != closestFaceA &&
          /* Not already added */
          !polyA.connectedFaces.contains(i)
        ) {
          polyA.connectedFaces.add(i);
        }
      }
    }

    // Clip the polygon to the back of the planes of all faces of hull A,
    // that are adjacent to the witness face
    final int numVerticesA = polyA.length;
    for (int i = 0; i < numVerticesA; i++) {
      final a = hullA.vertices[polyA[i]];
      final b = hullA.vertices[polyA[(i + 1) % numVerticesA]];
      a.sub2(b, edge0);
      worldEdge0.setFrom(edge0);
      quatA.vmult(worldEdge0, worldEdge0);
      posA.add2(worldEdge0, worldEdge0);
      worldPlaneAnormal1.setFrom(faceNormals[closestFaceA]!);
      quatA.vmult(worldPlaneAnormal1, worldPlaneAnormal1);
      posA.add2(worldPlaneAnormal1, worldPlaneAnormal1);
      worldEdge0.cross2(worldPlaneAnormal1, planeNormalWS1);
      planeNormalWS1.negate();
      worldA1.setFrom(a);
      quatA.vmult(worldA1, worldA1);
      posA.add2(worldA1, worldA1);
      
      final otherFace = polyA.connectedFaces.isNotEmpty? polyA.connectedFaces[i]:0;
      localPlaneNormal.setFrom(faceNormals[otherFace]!);
      final localPlaneEq = getPlaneConstantOfFace(otherFace);
      planeNormalWS.setFrom(localPlaneNormal);
      quatA.vmult(planeNormalWS, planeNormalWS);
      double planeEqWS = localPlaneEq - planeNormalWS.dot(posA);

      // Clip face against our constructed plane
      clipFaceAgainstPlane(pVtxIn, pVtxOut, planeNormalWS, planeEqWS);

      // Throw away all clipped points, but save the remaining until next clip
      while (pVtxIn.isNotEmpty) {
        pVtxIn.removeAt(0);
      }
      while(pVtxOut.isNotEmpty) {
        pVtxIn.add(pVtxOut.removeAt(0));
      }
    }

    // only keep contact points that are behind the witness face
    localPlaneNormal.setFrom(faceNormals[closestFaceA]!);

    final localPlaneEq = getPlaneConstantOfFace(closestFaceA);
    planeNormalWS.setFrom(localPlaneNormal);
    quatA.vmult(planeNormalWS, planeNormalWS);

    final planeEqWS = localPlaneEq - planeNormalWS.dot(posA);
    for (int i = 0; i < pVtxIn.length; i++) {
      double depth = planeNormalWS.dot(pVtxIn[i]) + planeEqWS; // ???

      if (depth <= minDist) {
        logger?.verbose('clamped: depth=$depth to minDist=$minDist');
        depth = minDist;
      }

      if (depth <= maxDist) {
        final point = pVtxIn[i];
        if (depth <= 1e-6) {
          final p = ConvexPolyhedronContactPoint(
            point,
            planeNormalWS,
            depth,
          );
          result.add(p);
        }
      }
    }
  }

  /// Clip a face in a hull against the back of a plane.
  /// @param planeConstant The constant in the mathematical plane equation
  List<Vector3> clipFaceAgainstPlane(List<Vector3> inVertices, List<Vector3> outVertices, Vector3 planeNormal, double planeConstant){
    double nDotFirst;
    double nDotLast;
    final numVerts = inVertices.length;

    if (numVerts < 2) {
      return outVertices;
    }

    Vector3 firstVertex = inVertices[inVertices.length - 1];
    Vector3 lastVertex = inVertices[0];

    nDotFirst = planeNormal.dot(firstVertex) + planeConstant;

    for (int vi = 0; vi < numVerts; vi++) {
      lastVertex = inVertices[vi];
      nDotLast = planeNormal.dot(lastVertex) + planeConstant;
      if (nDotFirst < 0) {
        if (nDotLast < 0) {
          // Start < 0, end < 0, so output lastVertex
          final newv = Vector3.zero();
          newv.setFrom(lastVertex);
          outVertices.add(newv);
        } else {
          // Start < 0, end >= 0, so output intersection
          final newv = Vector3.zero();
          firstVertex.lerp(lastVertex, nDotFirst / (nDotFirst - nDotLast), newv);
          outVertices.add(newv);
        }
      } else {
        if (nDotLast < 0) {
          // Start >= 0, end < 0 so output intersection and end
          final newv = Vector3.zero();
          firstVertex.lerp(lastVertex, nDotFirst / (nDotFirst - nDotLast), newv);
          outVertices.add(newv);
          outVertices.add(lastVertex);
        }
      }
      firstVertex = lastVertex;
      nDotFirst = nDotLast;
    }
    return outVertices;
  }

  /// Updates `.worldVertices` and sets `.worldVerticesNeedsUpdate` to false.
  void computeWorldVertices(Vector3 position, Quaternion quat) {
    final int n = vertices.length;
    while (worldVertices.length < n) {
      worldVertices.add(Vector3.zero());
    }

    final verts = vertices;
    //final worldVerts = worldVertices;
    for (int i = 0; i != n; i++) {
      quat.vmult(verts[i], worldVertices[i]);
      position.add2(worldVertices[i], worldVertices[i]);
    }

    worldVerticesNeedsUpdate = false;
  }

  void computeLocalAABB(Vector3 aabbmin, Vector3 aabbmax) {
    final vertices = this.vertices;

    aabbmin.setValues(double.infinity, double.infinity, double.infinity);
    aabbmax.setValues(-double.infinity, -double.infinity, -double.infinity);

    for (int i = 0; i < this.vertices.length; i++) {
      final v = vertices[i];
      if (v.x < aabbmin.x) {
        aabbmin.x = v.x;
      } else if (v.x > aabbmax.x) {
        aabbmax.x = v.x;
      }
      if (v.y < aabbmin.y) {
        aabbmin.y = v.y;
      } else if (v.y > aabbmax.y) {
        aabbmax.y = v.y;
      }
      if (v.z < aabbmin.z) {
        aabbmin.z = v.z;
      } else if (v.z > aabbmax.z) {
        aabbmax.z = v.z;
      }
    }
  }

  /// Updates `worldVertices` and sets `worldFaceNormalsNeedsUpdate` to false.
  void computeWorldFaceNormals(Quaternion quat) {
    final N = faceNormals.length;
    while (worldFaceNormals.length < N) {
      worldFaceNormals.add(Vector3.zero());
    }

    final normals = faceNormals;
    //final worldNormals = worldFaceNormals;
    for (int i = 0; i != N; i++) {
      quat.vmult(normals[i]!, worldFaceNormals[i]);
    }

    worldFaceNormalsNeedsUpdate = false;
  }

  @override
  void updateBoundingSphereRadius() {
    // Assume points are distributed with local (0,0,0) as center
    double max2 = 0;
    final verts = vertices;
    for (int i = 0; i != verts.length; i++) {
      final norm2 = verts[i].length2;
      if (norm2 > max2) {
        max2 = norm2;
      }
    }
    boundingSphereRadius = math.sqrt(max2);
  }

  @override
  void calculateWorldAABB(Vector3 pos, Quaternion quat, Vector3 min, Vector3 max) {
    final verts = vertices;
    double? minx;
    double? miny;
    double? minz;
    double? maxx;
    double? maxy;
    double? maxz;
    Vector3 tempWorldVertex = Vector3.zero();
    for (int i = 0; i < verts.length; i++) {
      tempWorldVertex.setFrom(verts[i]);
      quat.vmult(tempWorldVertex, tempWorldVertex);
      pos.add2(tempWorldVertex, tempWorldVertex);
      final v = tempWorldVertex;
      if (minx == null || v.x < minx) {
        minx = v.x;
      }

      if (maxx == null || v.x > maxx) {
        maxx = v.x;
      }

      if (miny == null || v.y < miny) {
        miny = v.y;
      }

      if (maxy == null || v.y > maxy) {
        maxy = v.y;
      }

      if (minz == null || v.z < minz) {
        minz = v.z;
      }

      if (maxz == null || v.z > maxz) {
        maxz = v.z;
      }
    }
    min.setValues(minx!, miny!, minz!);
    max.setValues(maxx!, maxy!, maxz!);
  }

  /// Get approximate convex volume
  @override
  double volume() {
    return (4.0 * math.pi * boundingSphereRadius) / 3.0;
  }

  /// Get an average of all the vertices position
  Vector3 getAveragePointLocal([Vector3? target]) {
    target ??= Vector3.zero();
    final verts = vertices;
    for (int i = 0; i < verts.length; i++) {
      target.add2(verts[i], target);
    }
    target.scale2(1 / verts.length, target);
    return target;
  }

  /// Transform all local points. Will change the .vertices
  void transformAllPoints([Vector3? offset, Quaternion? quat]) {
    int n = vertices.length;
    final verts = vertices;

    // Apply rotation
    if (quat != null) {
      // Rotate vertices
      for (int i = 0; i < n; i++) {
        final v = verts[i];
        quat.vmult(v, v);
      }
      // Rotate face normals
      for (int i = 0; i < faceNormals.length; i++) {
        final v = faceNormals[i]!;
        quat.vmult(v, v);
      }

      // Rotate edges
      // for(int i=0; i< uniqueEdges.length; i++){
      //   final v = uniqueEdges[i];
      //   quat.vmult(v,v);
      // }
    }

    // // Apply offset
    if (offset != null) {
      for (int i = 0; i < n; i++) {
        final v = verts[i];
        v.add2(offset, v);
      }
    }
  }

  /// Checks whether p is inside the polyhedra. Must be in local coords.
  /// The point lies outside of the convex hull of the other points if and only if the direction
  /// of all the vectors from it to those other points are on less than one half of a sphere around it.
  /// @param p A point given in local coordinates
  bool pointIsInside(Vector3 p){
    final verts = vertices;
    final faces = this.faces;
    final normals = faceNormals;
    //bool? positiveResult;
    final pointInside = Vector3.zero();//_convexPolyhedronPointIsInside;
    getAveragePointLocal(pointInside);

    for (int i = 0; i < this.faces.length; i++) {
      final n = normals[i]!;
      final v = verts[faces[i][0]]; // We only need one point in the face

      // This dot product determines which side of the edge the point is
      final vToP = Vector3.zero();//_convexPolyhedronVToP;
      p.sub2(v, vToP);
      final r1 = n.dot(vToP);

      final vToPointInside = Vector3.zero();//_convexPolyhedronVToPointInside;//
      pointInside.sub2(v, vToPointInside);
      final r2 = n.dot(vToPointInside);

      if ((r1 < 0 && r2 > 0) || (r1 > 0 && r2 < 0)) {
        return false; // Encountered some other sign. Exit.
      }
    }
    // If we got here, all dot products were of the same sign.
    return true;
  }

  static Body trimeshToPolyhedron(Trimesh trimesh,Body body, [Vector3? upvector]){
    final p1 = Vector3.zero();
    final p2 = Vector3.zero();
    final p3 = Vector3.zero();
    final p4 = Vector3.zero();
    final mp = Vector3.zero();
    final tmp = Vector3.zero();
    final e1 = Vector3.zero();
    final e2 = Vector3.zero();

    for(int i = 0; i < trimesh.indices.length/3;i++){
      mp.setValues(0,0,0);
      trimesh.getTriangleVertices(i,p1,p2,p3);
      trimesh.getIndicesNormal(i,p4);
      if(upvector != null && p4.dot(upvector) < 0){
        p4.scale2(-1,p4);
      }
      p4.normalize();
      mp..add(p1)..add(p2)..add(p3)..scale(1/3);
      final List<Vector3> vertices = [Vector3.copy(p1),Vector3.copy(p2),Vector3.copy(p3),mp..add(Vector3.copy(p4)..scale(-1))];
      final faces = [[0,1,2],[0,3,1],[1,3,2],[2,3,0]];
      final List<Vector3> normals = [Vector3.copy(p4)];
      for(int j = 0;j < 3;j++){
        vertices[j].sub2(vertices[(j+1)%3],e1);
        vertices[(j+1)%3].sub2(p4,e2);
        tmp.setValues(1,1,1);
        final points = faces[j+1];
        for(int p = 0;p < points.length;p++){
            tmp.add2(vertices[points[p]],tmp);
        }
        tmp.scale2(1/points.length,tmp);
        final normal = e1.cross(e2);
        normal.normalize();
        normal.scale2(-1,normal);
        final angle = normal.dot(tmp);
        
        if(angle <= 0 ){
          normal.scale2(-1,normal);
        }
        normals.add(normal);
      }
      final polyhedron = ConvexPolyhedron(
        vertices:vertices,
        faces:faces,
        normals:normals
      );
      body.addShape(polyhedron);
    }
    return body;
  }

  /// Get max and min dot product of a convex hull at position (pos,quat) projected onto an axis.
  /// Results are saved in the array maxmin.
  /// @param result result[0] and result[1] will be set to maximum and minimum, respectively.
  static void project(ConvexPolyhedron shape, Vector3 axis, Vector3 pos, Quaternion quat, final List<double> result) {
    final int n = shape.vertices.length;
    //final worldVertex = project_worldVertex;
    final localAxis = _projectLocalAxis;
    double max = 0;
    double min = 0;
    final localOrigin = _projectLocalOrigin;
    final vs = shape.vertices;

    localOrigin.setZero();

    // Transform the axis to local
    Transform.vectorToLocalFrame(pos, quat, axis, localAxis);
    Transform.pointToLocalFrame(pos, quat, localOrigin, localOrigin);
    final add = localOrigin.dot(localAxis);

    min = max = vs[0].dot(localAxis);

    for (int i = 1; i < n; i++) {
      final val = vs[i].dot(localAxis);
      if (val > max) {
        max = val;
      }
      if (val < min) {
        min = val;
      }
    }

    min -= add;
    max -= add;

    if (min > max) {
      // Inconsistent - swap
      final temp = min;
      min = max;
      max = temp;
    }
    // Output
    result[0] = max;
    result[1] = min;
  }
}
