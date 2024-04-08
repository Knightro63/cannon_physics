import 'shape.dart';
import 'package:vector_math/vector_math.dart';

/// Particle shape.
/// @example
///     const particleShape = new CANNON.Particle()
///     const particleBody = new CANNON.Body({ mass: 1, shape: particleShape })
///     world.addBody(particleBody)
class Particle extends Shape {
  Particle():super(type: ShapeType.particle );

  @override
  Vector3 calculateLocalInertia(double mass, [Vector3? target]) {
    target ??= Vector3.zero();
    target.setValues(0, 0, 0);
    return target;
  }

  @override
  double volume(){
    return 0;
  }

  @override
  void updateBoundingSphereRadius() {
    boundingSphereRadius = 0;
  }

  @override
  void calculateWorldAABB(Vector3 pos, Quaternion quat, Vector3 min, Vector3 max) {
    // Get each axis max
    min.setFrom(pos);
    max.setFrom(pos);
  }
}
