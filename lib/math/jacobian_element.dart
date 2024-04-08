import 'package:vector_math/vector_math.dart';

/// An element containing 6 entries, 3 spatial and 3 rotational degrees of freedom.
class JacobianElement {
  Vector3 spatial = Vector3.zero();
  Vector3 rotational = Vector3.zero();

  /// Multiply with other JacobianElement
  double multiplyElement(JacobianElement element) {
    return element.spatial.dot(spatial) + element.rotational.dot(rotational);
  }

  /// Multiply with two vectors
  double multiplyVectors(Vector3 spatial, Vector3 rotational) {
    return spatial.dot(this.spatial) + rotational.dot(this.rotational);
  }
}
