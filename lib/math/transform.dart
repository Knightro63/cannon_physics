import 'vec3.dart';
import 'quaternion.dart';
import 'package:vector_math/vector_math.dart';

/// Transformation utilities.
class Transform {
  late Vector3 position = Vector3.zero(); 
  late Quaternion quaternion = Quaternion(0,0,0,1);

  Transform({
    Vector3? position,
    Quaternion? quaternion
  }){
    if (position != null) {
      this.position.setFrom(position);
    }

    if (quaternion != null) {
      this.quaternion.setFrom(quaternion);
    }
  }

  /// Get a global point in local transform coordinates.
  Vector3 pointToLocal(Vector3 worldPoint, [Vector3? result]) {
    result ??= Vector3.zero();
    pointToLocalFrame(position, quaternion, worldPoint, result);
    return result;
  }

  /// Get a local point in global transform coordinates.
  Vector3 pointToWorld(Vector3 localPoint, Vector3? result) {
    result ??= Vector3.zero();
    pointToWorldFrame(position, quaternion, localPoint, result);
    return result;
  }

  Vector3 vectorToWorld(Vector3 localVector, [Vector3? result]) {
    result ??= Vector3.zero();
    quaternion.vmult(localVector, result);
    return result;
  }

  static Vector3 pointToLocalFrame(Vector3 position,Quaternion quaternion,Vector3 worldPoint, [Vector3? result]) {
    result ??= Vector3.zero();
    worldPoint.sub2(position, result);
    final tmpQuat = Quaternion.copy(quaternion);
    tmpQuat.conjugate();
    tmpQuat.vmult(result, result);
    return result;
  }

  static Vector3 pointToWorldFrame(Vector3 position,Quaternion quaternion,Vector3 localPoint, [Vector3? result]) {
    result ??= Vector3.zero();
    quaternion.vmult(localPoint, result);
    result.add2(position, result);
    return result;
  }

  static Vector3 vectorToWorldFrame(Quaternion quaternion,Vector3 localVector, [Vector3? result]){
    result ??= Vector3.zero();
    quaternion.vmult(localVector, result);
    return result;
  }

  static Vector3 vectorToLocalFrame(Vector3 position,Quaternion quaternion,Vector3 worldVector, [Vector3? result]){
    result ??= Vector3.zero();
    quaternion.w *= -1;
    quaternion.vmult(worldVector, result);
    quaternion.w *= -1;
    return result;
  }
}
