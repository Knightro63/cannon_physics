import 'shape.dart';
import 'package:vector_math/vector_math.dart';
import 'convex_polyhedron.dart';
/// A plane, facing in the Z direction. The plane has its surface at z=0 and everything below z=0 is assumed to be solid plane. To make the plane face in some other direction than z, you must put it inside a Body and rotate that body. See the demos.
/// @example
///     const planeShape = new CANNON.Plane()
///     const planeBody = new CANNON.Body({ mass: 0, shape:  planeShape })
///     planeBody.quaternion.setFromEuler(-Math.PI / 2, 0, 0) // make it face up
///     world.addBody(planeBody)
class SizedPlane extends ConvexPolyhedron {
  late Vector3 worldNormal; 
  late bool worldNormalNeedsUpdate;
  //late double boundingSphereRadius;

  final double width;
  final double height;

  SizedPlane([
    this.width = 1,this.height= 1
  ]):super(type: ShapeType.sizedPlane ){
    final double sx = width/2;
    const double sy = 0;
    final double sz = height/2;

    List<Vector3> vertices = [
      Vector3(-sx, sy, -sz),
      Vector3(sx, sy, -sz),
      Vector3(sx, sy, sz),
      Vector3(-sx, sy, sz),
    ];

    const faces = [
      [3, 2, 1, 0]
    ];

    init(vertices, faces);
  }
}
