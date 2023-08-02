import 'dart:math' as math;
import '../shapes/shape.dart';
import '../math/vec3.dart';
import '../math/quaternion.dart';

/// Spherical shape
/// @example
///     const radius = 1
///     const sphereShape = new CANNON.Sphere(radius)
///     const sphereBody = new CANNON.Body({ mass: 1, shape: sphereShape })
///     world.addBody(sphereBody)
class Sphere extends Shape {
  ///The radius of the sphere.
  double radius;

  /// @param radius The radius of the sphere, a non-negative number.
  Sphere([this.radius = 1.0]): super(type: ShapeType.sphere ){
    if (radius < 0) {
      throw('The sphere radius cannot be negative.');
    }

    updateBoundingSphereRadius();
  }

  @override
  Vec3 calculateLocalInertia(num mass, [Vec3? target]) {
    target ??= Vec3();
    double I = (2.0 * mass * radius * radius) / 5.0;
    target.x = I;
    target.y = I;
    target.z = I;
    return target;
  }

  @override
  double volume() {
    return (4.0 * math.pi * math.pow(radius, 3)) / 3.0;
  }
  @override
  void updateBoundingSphereRadius() {
    boundingSphereRadius = radius;
  }
  @override
  void calculateWorldAABB(Vec3 pos,Quaternion quat,Vec3 min,Vec3 max) {
    num r = radius;
    //List<int> axes = [0,1,2] ;
    //for (int i = 0; i < axes.length; i++) {
      //int ax = axes[i];
      min.x = pos.x - r;
      max.x = pos.x + r;

      min.y = pos.y - r;
      max.y = pos.y + r;
  
      min.z = pos.z - r;
      max.z = pos.z + r;
    //}
  }
}
