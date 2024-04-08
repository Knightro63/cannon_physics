import 'dart:math' as math;
import 'shape.dart';
import 'package:vector_math/vector_math.dart';

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
  Vector3 calculateLocalInertia(num mass, [Vector3? target]) {
    target ??= Vector3.zero();
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
  void calculateWorldAABB(Vector3 pos,Quaternion quat,Vector3 min,Vector3 max) {
    double r = radius;
    min.x = pos.x - r;
    max.x = pos.x + r;

    min.y = pos.y - r;
    max.y = pos.y + r;

    min.z = pos.z - r;
    max.z = pos.z + r;
  }
}
