import 'shape.dart';
import '../math/quaternion.dart';
import 'package:vector_math/vector_math.dart';

/// A plane, facing in the Z direction. The plane has its surface at z=0 and everything below z=0 is assumed to be solid plane. To make the plane face in some other direction than z, you must put it inside a Body and rotate that body. See the demos.
/// @example
///     const planeShape = new CANNON.Plane()
///     const planeBody = new CANNON.Body({ mass: 0, shape:  planeShape })
///     planeBody.quaternion.setFromEuler(-Math.PI / 2, 0, 0) // make it face up
///     world.addBody(planeBody)
class Plane extends Shape {
  late Vector3 worldNormal; 
  late bool worldNormalNeedsUpdate;
  //late double boundingSphereRadius;

  Plane():super(type: ShapeType.plane ){
    // World oriented normal
    worldNormal = Vector3.zero();
    worldNormalNeedsUpdate = true;
    boundingSphereRadius = double.infinity;
  }

  final _tempNormal = Vector3.zero();

  void computeWorldNormal(Quaternion quat){
    final n = worldNormal;
    n.setValues(0, 0, 1);
    quat.vmult(n, n);
    worldNormalNeedsUpdate = false;
  }

  @override
  Vector3 calculateLocalInertia(double mass, [Vector3? target]) {
    target ??= Vector3.zero();
    return target;
  }

  @override
  double volume() {
    return double.infinity;
  }

  @override
  void calculateWorldAABB(Vector3 pos, Quaternion quat, Vector3 min, Vector3 max) {
    // The plane AABB is infinite, except if the normal is pointing along any axis
    _tempNormal.setValues(0, 0, 1); // Default plane normal is z
    quat.vmult(_tempNormal, _tempNormal);
    const maxVal = double.infinity;
    min.setValues(-maxVal, -maxVal, -maxVal);
    max.setValues(maxVal, maxVal, maxVal);

    if (_tempNormal.x == 1) {
      max.x = pos.x;
    } else if (_tempNormal.x == -1) {
      min.x = pos.x;
    }

    if (_tempNormal.y == 1) {
      max.y = pos.y;
    } else if (_tempNormal.y == -1) {
      min.y = pos.y;
    }

    if (_tempNormal.z == 1) {
      max.z = pos.z;
    } else if (_tempNormal.z == -1) {
      min.z = pos.z;
    }
  }

  @override
  void updateBoundingSphereRadius() {
    boundingSphereRadius = double.infinity;
  }
}
