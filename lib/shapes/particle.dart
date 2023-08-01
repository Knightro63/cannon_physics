import '../shapes/shape.dart';
import '../math/vec3.dart';
import '../math/quaternion.dart';

/**
 * Particle shape.
 * @example
 *     const particleShape = new CANNON.Particle()
 *     const particleBody = new CANNON.Body({ mass: 1, shape: particleShape })
 *     world.addBody(particleBody)
 */
class Particle extends Shape {
  Particle():super(type: ShapeType.particle );

  /**
   * calculateLocalInertia
   */
  Vec3 calculateLocalInertia(double mass, [Vec3? target]) {
    target ??= Vec3();
    target.set(0, 0, 0);
    return target;
  }

  double volume(){
    return 0;
  }

  void updateBoundingSphereRadius() {
    boundingSphereRadius = 0;
  }

  void calculateWorldAABB(Vec3 pos, Quaternion quat, Vec3 min,Vec3 max) {
    // Get each axis max
    min.copy(pos);
    max.copy(pos);
  }
}
