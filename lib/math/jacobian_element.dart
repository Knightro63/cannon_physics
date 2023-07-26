import 'vec3.dart';

/**
 * An element containing 6 entries, 3 spatial and 3 rotational degrees of freedom.
 */
class JacobianElement {
  /**
   * spatial
   */
  late Vec3 spatial;
  /**
   * rotational
   */
  late Vec3 rotational;

  JacobianElement({
    Vec3? spatial,
    Vec3? rotational
  }) {
    this.spatial = spatial ?? Vec3();
    this.rotational = rotational ?? Vec3();
  }

  /**
   * Multiply with other JacobianElement
   */
  num multiplyElement(JacobianElement element) {
    return element.spatial.dot(spatial) + element.rotational.dot(rotational);
  }

  /**
   * Multiply with two vectors
   */
  num multiplyVectors(Vec3 spatial, Vec3 rotational) {
    return spatial.dot(this.spatial) + rotational.dot(this.rotational);
  }
}
