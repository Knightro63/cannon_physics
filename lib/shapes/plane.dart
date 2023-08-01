import '../shapes/shape.dart';
import '../math/vec3.dart';
import '../math/quaternion.dart';

/**
 * A plane, facing in the Z direction. The plane has its surface at z=0 and everything below z=0 is assumed to be solid plane. To make the plane face in some other direction than z, you must put it inside a Body and rotate that body. See the demos.
 * @example
 *     const planeShape = new CANNON.Plane()
 *     const planeBody = new CANNON.Body({ mass: 0, shape:  planeShape })
 *     planeBody.quaternion.setFromEuler(-Math.PI / 2, 0, 0) // make it face up
 *     world.addBody(planeBody)
 */
class Plane extends Shape {
  /** worldNormal */
  late Vec3 worldNormal; 
  /** worldNormalNeedsUpdate */
  late bool worldNormalNeedsUpdate;
  late double boundingSphereRadius;

  Plane():super(type: ShapeType.plane ){
    // World oriented normal
    worldNormal = Vec3();
    worldNormalNeedsUpdate = true;
    boundingSphereRadius = double.infinity;
  }

  /** computeWorldNormal */
  void computeWorldNormal(Quaternion quat){
    final n = worldNormal;
    n.set(0, 0, 1);
    quat.vmult(n, n);
    worldNormalNeedsUpdate = false;
  }

  Vec3 calculateLocalInertia(double mass, [Vec3? target]) {
    target ??= Vec3();
    return target;
  }

  double volume() {
    return double.infinity;
  }

  void calculateWorldAABB(Vec3 pos, Quaternion quat, Vec3 min, Vec3 max) {
    // The plane AABB is infinite, except if the normal is pointing along any axis
    tempNormal.set(0, 0, 1); // Default plane normal is z
    quat.vmult(tempNormal, tempNormal);
    final maxVal = double.infinity;
    min.set(-maxVal, -maxVal, -maxVal);
    max.set(maxVal, maxVal, maxVal);

    if (tempNormal.x == 1) {
      max.x = pos.x;
    } else if (tempNormal.x == -1) {
      min.x = pos.x;
    }

    if (tempNormal.y == 1) {
      max.y = pos.y;
    } else if (tempNormal.y == -1) {
      min.y = pos.y;
    }

    if (tempNormal.z == 1) {
      max.z = pos.z;
    } else if (tempNormal.z == -1) {
      min.z = pos.z;
    }
  }

  void updateBoundingSphereRadius() {
    boundingSphereRadius = double.infinity;
  }
}

final tempNormal = Vec3();
