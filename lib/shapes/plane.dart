import '../shapes/shape.dart';
import '../math/vec3.dart';
import '../math/quaternion.dart';

/// A plane, facing in the Z direction. The plane has its surface at z=0 and everything below z=0 is assumed to be solid plane. To make the plane face in some other direction than z, you must put it inside a Body and rotate that body. See the demos.
/// @example
///     const planeShape = new CANNON.Plane()
///     const planeBody = new CANNON.Body({ mass: 0, shape:  planeShape })
///     planeBody.quaternion.setFromEuler(-Math.PI / 2, 0, 0) // make it face up
///     world.addBody(planeBody)
class Plane extends Shape {
  late Vec3 worldNormal; 
  late bool worldNormalNeedsUpdate;
  //late double boundingSphereRadius;

  final double width;
  final double height;

  Plane([this.width = double.infinity,this.height= double.infinity]):super(type: ShapeType.plane ){
    // World oriented normal
    worldNormal = Vec3();
    worldNormalNeedsUpdate = true;
    boundingSphereRadius = double.infinity;
  }

  final _tempNormal = Vec3();

  void computeWorldNormal(Quaternion quat){
    final n = worldNormal;
    n.set(0, 0, 1);
    quat.vmult(n, n);
    worldNormalNeedsUpdate = false;
  }

  @override
  Vec3 calculateLocalInertia(double mass, [Vec3? target]) {
    target ??= Vec3();
    return target;
  }

  @override
  double volume() {
    return double.infinity;
  }

  @override
  void calculateWorldAABB(Vec3 pos, Quaternion quat, Vec3 min, Vec3 max) {
    // The plane AABB is infinite, except if the normal is pointing along any axis
    _tempNormal.set(0, 0, 1); // Default plane normal is z
    quat.vmult(_tempNormal, _tempNormal);
    const maxVal = double.infinity;
    min.set(-maxVal, -maxVal, -maxVal);
    max.set(maxVal, maxVal, maxVal);

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
