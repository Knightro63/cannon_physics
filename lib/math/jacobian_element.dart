import 'vec3.dart';

/// An element containing 6 entries, 3 spatial and 3 rotational degrees of freedom.
class JacobianElement {
  Vec3 spatial = Vec3();
  Vec3 rotational = Vec3();

  /// Multiply with other JacobianElement
  double multiplyElement(JacobianElement element) {
    return element.spatial.dot(spatial) + element.rotational.dot(rotational);
  }

  /// Multiply with two vectors
  double multiplyVectors(Vec3 spatial, Vec3 rotational) {
    return spatial.dot(this.spatial) + rotational.dot(this.rotational);
  }
}
