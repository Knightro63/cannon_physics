import 'shape.dart';
import '../math/vec3.dart';
import 'convex_polyhedron.dart';
import '../math/quaternion.dart';
import 'package:vector_math/vector_math.dart';

/// A 3d box shape.
/// @example
///     const size = 1
///     const halfExtents = CANNON.Vector3(size, size, size)
///     const boxShape = CANNON.Box(halfExtents)
///     const boxBody = CANNON.Body({ mass: 1, shape: boxShape })
///     world.addBody(boxBody)
class Box extends Shape {
  /// The half extents of the box.
  Vector3 halfExtents;

  /// Used by the contact generator to make contacts with other convex polyhedra for example.
  late ConvexPolyhedron convexPolyhedronRepresentation;

  Box(this.halfExtents):super(type: ShapeType.box){
    //this.convexPolyhedronRepresentation = null as unknown as ConvexPolyhedron;
    updateConvexPolyhedronRepresentation();
    updateBoundingSphereRadius();
  }

  final Vector3 _worldCornerTempPos = Vector3.zero();

  final List<Vector3> _worldCornersTemp = [
    Vector3.zero(),
    Vector3.zero(),
    Vector3.zero(),
    Vector3.zero(),
    Vector3.zero(),
    Vector3.zero(),
    Vector3.zero(),
    Vector3.zero(),
  ];

  /// Updates the local convex polyhedron representation used for some collisions.
  void updateConvexPolyhedronRepresentation(){
    final double sx = halfExtents.x;
    final double sy = halfExtents.y;
    final double sz = halfExtents.z;

    List<Vector3> vertices = [
      Vector3(-sx, -sy, -sz),
      Vector3(sx, -sy, -sz),
      Vector3(sx, sy, -sz),
      Vector3(-sx, sy, -sz),
      Vector3(-sx, -sy, sz),
      Vector3(sx, -sy, sz),
      Vector3(sx, sy, sz),
      Vector3(-sx, sy, sz),
    ];

    const faces = [
      [3, 2, 1, 0], // -z
      [4, 5, 6, 7], // +z
      [5, 4, 0, 1], // -y
      [2, 3, 7, 6], // +y
      [0, 4, 7, 3], // -x
      [1, 2, 6, 5], // +x
    ];

    final List<Vector3> axes = [
      Vector3(0, 0, 1), 
      Vector3(0, 1, 0), 
      Vector3(1, 0, 0)
    ];

    convexPolyhedronRepresentation = ConvexPolyhedron(
      vertices:vertices, 
      faces:faces,
      axes:axes,
    );
    convexPolyhedronRepresentation.material = material;
  }

  /// Calculate the inertia of the box.
  @override
  Vector3 calculateLocalInertia(num mass,[ Vector3? target]){
    target ??= Vector3.zero();
    Box.calculateInertia(halfExtents, mass, target);
    return target;
  }

  static Vector3 calculateInertia(Vector3 halfExtents,num mass, Vector3 target){
    final Vector3 e = halfExtents;
    target.x = 1.0 / 12.0 * mass * (2 * e.y * 2 * e.y + 2 * e.z * 2 * e.z);
    target.y = 1.0 / 12.0 * mass * (2 * e.x * 2 * e.x + 2 * e.z * 2 * e.z);
    target.z = 1.0 / 12.0 * mass * (2 * e.y * 2 * e.y + 2 * e.x * 2 * e.x);
    return target;
  }

  /// Get the box 6 side normals
  /// @param sixTargetVectors An array of 6 vectors, to store the resulting side normals in.
  /// @param quat Orientation to apply to the normal vectors. If not provided, the vectors will be in respect to the local frame.
  List<Vector3> getSideNormals(List<Vector3> sixTargetVectors, [Quaternion? quat ]){
    final List<Vector3> sides = sixTargetVectors;
    final Vector3 ex = halfExtents;
    sides[0].setValues(ex.x, 0, 0);
    sides[1].setValues(0, ex.y, 0);
    sides[2].setValues(0, 0, ex.z);
    sides[3].setValues(-ex.x, 0, 0);
    sides[4].setValues(0, -ex.y, 0);
    sides[5].setValues(0, 0, -ex.z);

    if (quat != null) {
      for (int i = 0; i != sides.length; i++) {
        quat.vmult(sides[i], sides[i]);
      }
    }
    return sides;
  }

  /// Returns the volume of the box.
  @override
  double volume() {
    return 8.0 * halfExtents.x * halfExtents.y * halfExtents.z;
  }

  @override
  void updateBoundingSphereRadius() {
    boundingSphereRadius = halfExtents.length;
  }

  void forEachWorldCorner(Vector3 pos, Quaternion quat, void Function(num x, num y, num z) callback) {
    final Vector3 e = halfExtents;
    final List<List<double>> corners = [
      [e.x, e.y, e.z],
      [-e.x, e.y, e.z],
      [-e.x, -e.y, e.z],
      [-e.x, -e.y, -e.z],
      [e.x, -e.y, -e.z],
      [e.x, e.y, -e.z],
      [-e.x, e.y, -e.z],
      [e.x, -e.y, e.z],
    ];
    for (int i = 0; i < corners.length; i++) {
      _worldCornerTempPos.setValues(corners[i][0], corners[i][1], corners[i][2]);
      quat.vmult(_worldCornerTempPos, _worldCornerTempPos);
      pos.add2(_worldCornerTempPos, _worldCornerTempPos);
      callback(_worldCornerTempPos.x, _worldCornerTempPos.y, _worldCornerTempPos.z);
    }
  }

  @override
  void calculateWorldAABB(Vector3 pos, Quaternion quat,Vector3 min, Vector3 max) {
    final Vector3 e = halfExtents;
    _worldCornersTemp[0].setValues(e.x, e.y, e.z);
    _worldCornersTemp[1].setValues(-e.x, e.y, e.z);
    _worldCornersTemp[2].setValues(-e.x, -e.y, e.z);
    _worldCornersTemp[3].setValues(-e.x, -e.y, -e.z);
    _worldCornersTemp[4].setValues(e.x, -e.y, -e.z);
    _worldCornersTemp[5].setValues(e.x, e.y, -e.z);
    _worldCornersTemp[6].setValues(-e.x, e.y, -e.z);
    _worldCornersTemp[7].setValues(e.x, -e.y, e.z);

    final Vector3 wc = _worldCornersTemp[0];
    quat.vmult(wc, wc);
    pos.add2(wc, wc);
    max.setFrom(wc);
    min.setFrom(wc);
    for (int i = 1; i < 8; i++) {
      final Vector3 wc = _worldCornersTemp[i];
      quat.vmult(wc, wc);
      pos.add2(wc, wc);
      double x = wc.x;
      double y = wc.y;
      double z = wc.z;
      if (x > max.x) {
        max.x = x;
      }
      if (y > max.y) {
        max.y = y;
      }
      if (z > max.z) {
        max.z = z;
      }

      if (x < min.x) {
        min.x = x;
      }
      if (y < min.y) {
        min.y = y;
      }
      if (z < min.z) {
        min.z = z;
      }
    }
  }
}
