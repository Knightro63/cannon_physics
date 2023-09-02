import 'vec3.dart';
import 'quaternion.dart';

final _tmpQuat = Quaternion();

/// Transformation utilities.
class Transform {
  late Vec3 position = Vec3(); 
  late Quaternion quaternion = Quaternion();

  Transform({
    Vec3? position,
    Quaternion? quaternion
  }){
    if (position != null) {
      this.position.copy(position);
    }

    if (quaternion != null) {
      this.quaternion.copy(quaternion);
    }
  }

  /// Get a global point in local transform coordinates.
  Vec3 pointToLocal(Vec3 worldPoint, [Vec3? result]) {
    result ??= Vec3();
    pointToLocalFrame(position, quaternion, worldPoint, result);
    return result;
  }

  /// Get a local point in global transform coordinates.
  Vec3 pointToWorld(Vec3 localPoint, Vec3? result) {
    result ??= Vec3();
    pointToWorldFrame(position, quaternion, localPoint, result);
    return result;
  }

  Vec3 vectorToWorld(Vec3 localVector, [Vec3? result]) {
    result ??= Vec3();
    quaternion.vmult(localVector, result);
    return result;
  }

  static Vec3 pointToLocalFrame(Vec3 position,Quaternion quaternion,Vec3 worldPoint, [Vec3? result]) {
    result ??= Vec3();
    worldPoint.vsub(position, result);
    quaternion.conjugate(_tmpQuat);
    _tmpQuat.vmult(result, result);
    return result;
  }

  static Vec3 pointToWorldFrame(Vec3 position,Quaternion quaternion,Vec3 localPoint, [Vec3? result]) {
    result ??= Vec3();
    quaternion.vmult(localPoint, result);
    result.vadd(position, result);
    return result;
  }

  static Vec3 vectorToWorldFrame(Quaternion quaternion,Vec3 localVector, [Vec3? result]){
    result ??= Vec3();
    quaternion.vmult(localVector, result);
    return result;
  }

  static Vec3 vectorToLocalFrame(Vec3 position,Quaternion quaternion,Vec3 worldVector, [Vec3? result]){
    result ??= Vec3();
    quaternion.w *= -1;
    quaternion.vmult(worldVector, result);
    quaternion.w *= -1;
    return result;
  }
}
