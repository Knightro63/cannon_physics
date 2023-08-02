import '../shapes/shape.dart';
import '../math/vec3.dart';
import '../shapes/convex_polyhedron.dart';
import '../math/quaternion.dart';

/// A 3d box shape.
/// @example
///     const size = 1
///     const halfExtents = CANNON.Vec3(size, size, size)
///     const boxShape = CANNON.Box(halfExtents)
///     const boxBody = CANNON.Body({ mass: 1, shape: boxShape })
///     world.addBody(boxBody)
class Box extends Shape {
  /// The half extents of the box.
  Vec3 halfExtents;

  /// Used by the contact generator to make contacts with other convex polyhedra for example.
  ConvexPolyhedron? convexPolyhedronRepresentation;

  Box(this.halfExtents):super(type: ShapeType.box){
    //this.convexPolyhedronRepresentation = null as unknown as ConvexPolyhedron;
    updateConvexPolyhedronRepresentation();
    updateBoundingSphereRadius();
  }

  /// Updates the local convex polyhedron representation used for some collisions.
  void updateConvexPolyhedronRepresentation(){
    double sx = halfExtents.x;
    double sy = halfExtents.y;
    double sz = halfExtents.z;

    List<Vec3> vertices = [
      Vec3(-sx, -sy, -sz),
      Vec3(sx, -sy, -sz),
      Vec3(sx, sy, -sz),
      Vec3(-sx, sy, -sz),
      Vec3(-sx, -sy, sz),
      Vec3(sx, -sy, sz),
      Vec3(sx, sy, sz),
      Vec3(-sx, sy, sz),
    ];

    const faces = [
      [3, 2, 1, 0], // -z
      [4, 5, 6, 7], // +z
      [5, 4, 0, 1], // -y
      [2, 3, 7, 6], // +y
      [0, 4, 7, 3], // -x
      [1, 2, 6, 5], // +x
    ];

    final List<Vec3> axes = [Vec3(0, 0, 1), Vec3(0, 1, 0), Vec3(1, 0, 0)];

    final ConvexPolyhedron h = ConvexPolyhedron(vertices:vertices, faces:faces, axes:axes );
    convexPolyhedronRepresentation = h;
    h.material = material;
  }

  /// Calculate the inertia of the box.
  @override
  Vec3 calculateLocalInertia(num mass,[ Vec3? target]){
    target ??= Vec3();
    calculateInertia(halfExtents, mass, target);
    return target;
  }

  static void calculateInertia(Vec3 halfExtents,num mass, Vec3 target){
    final Vec3 e = halfExtents;
    target.x = (1.0 / 12.0) * mass * (2 * e.y * 2 * e.y + 2 * e.z * 2 * e.z);
    target.y = (1.0 / 12.0) * mass * (2 * e.x * 2 * e.x + 2 * e.z * 2 * e.z);
    target.z = (1.0 / 12.0) * mass * (2 * e.y * 2 * e.y + 2 * e.x * 2 * e.x);
  }

  /// Get the box 6 side normals
  /// @param sixTargetVectors An array of 6 vectors, to store the resulting side normals in.
  /// @param quat Orientation to apply to the normal vectors. If not provided, the vectors will be in respect to the local frame.
  List<Vec3> getSideNormals(List<Vec3> sixTargetVectors, [Quaternion? quat ]){
    final List<Vec3>  sides = sixTargetVectors;
    final Vec3 ex = halfExtents;
    sides[0].set(ex.x, 0, 0);
    sides[1].set(0, ex.y, 0);
    sides[2].set(0, 0, ex.z);
    sides[3].set(-ex.x, 0, 0);
    sides[4].set(0, -ex.y, 0);
    sides[5].set(0, 0, -ex.z);

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
    boundingSphereRadius = halfExtents.length();
  }

  void forEachWorldCorner(Vec3 pos, Quaternion quat, void Function(num x, num y, num z) callback) {
    final Vec3 e = halfExtents;
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
      worldCornerTempPos.set(corners[i][0], corners[i][1], corners[i][2]);
      quat.vmult(worldCornerTempPos, worldCornerTempPos);
      pos.vadd(worldCornerTempPos, worldCornerTempPos);
      callback(worldCornerTempPos.x, worldCornerTempPos.y, worldCornerTempPos.z);
    }
  }

  @override
  void calculateWorldAABB(Vec3 pos, Quaternion quat,Vec3 min, Vec3 max) {
    final Vec3 e = halfExtents;
    worldCornersTemp[0].set(e.x, e.y, e.z);
    worldCornersTemp[1].set(-e.x, e.y, e.z);
    worldCornersTemp[2].set(-e.x, -e.y, e.z);
    worldCornersTemp[3].set(-e.x, -e.y, -e.z);
    worldCornersTemp[4].set(e.x, -e.y, -e.z);
    worldCornersTemp[5].set(e.x, e.y, -e.z);
    worldCornersTemp[6].set(-e.x, e.y, -e.z);
    worldCornersTemp[7].set(e.x, -e.y, e.z);

    final Vec3 wc = worldCornersTemp[0];
    quat.vmult(wc, wc);
    pos.vadd(wc, wc);
    max.copy(wc);
    min.copy(wc);
    for (int i = 1; i < 8; i++) {
      final Vec3 wc = worldCornersTemp[i];
      quat.vmult(wc, wc);
      pos.vadd(wc, wc);
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

    // Get each axis max
    // min.set(Infinity,Infinity,Infinity);
    // max.set(-Infinity,-Infinity,-Infinity);
    // this.forEachWorldCorner(pos,quat,function(x,y,z){
    //     if(x > max.x){
    //         max.x = x;
    //     }
    //     if(y > max.y){
    //         max.y = y;
    //     }
    //     if(z > max.z){
    //         max.z = z;
    //     }

    //     if(x < min.x){
    //         min.x = x;
    //     }
    //     if(y < min.y){
    //         min.y = y;
    //     }
    //     if(z < min.z){
    //         min.z = z;
    //     }
    // });
  }
}

Vec3 worldCornerTempPos = Vec3();

List<Vec3> worldCornersTemp = [
  Vec3(),
  Vec3(),
  Vec3(),
  Vec3(),
  Vec3(),
  Vec3(),
  Vec3(),
  Vec3(),
];
