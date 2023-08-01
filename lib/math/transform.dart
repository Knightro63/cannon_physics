import 'vec3.dart';
import 'quaternion.dart';

//export type TransformOptions = ConstructorParameters<typeof Transform>[0];
final tmpQuat = Quaternion();
/**
 * Transformation utilities.
 */
class Transform {
  /**
   * position
   */
  late Vec3 position; 
  /**
   * quaternion
   */
  late Quaternion quaternion;

  Transform({
    Vec3? position,
    Quaternion? quaternion
  }){
    this.position = position ?? Vec3();
    this.quaternion = quaternion ?? Quaternion();
  }

  /**
   * Get a global point in local transform coordinates.
   */
  Vec3 pointToLocal(Vec3 worldPoint, [Vec3? result]) {
    return pointToLocalFrame(position, quaternion, worldPoint, result);
  }

  /**
   * Get a local point in global transform coordinates.
   */
  Vec3 pointToWorld(Vec3 localPoint, Vec3? result) {
    return pointToWorldFrame(position, quaternion, localPoint, result);
  }

  /**
   * vectorToWorldFrame
   */
  Vec3 vectorToWorld(Vec3 localVector, [Vec3? result]) {
    result ??= Vec3();
    quaternion.vmult(localVector, result);
    return result;
  }

  /**
   * pointToLocalFrame
   */
  static Vec3 pointToLocalFrame(Vec3 position,Quaternion quaternion,Vec3 worldPoint, [Vec3? result]) {
    result ??= Vec3();
    worldPoint.vsub(position, result);
    quaternion.conjugate(tmpQuat);
    tmpQuat.vmult(result, result);
    return result;
  }

  /**
   * pointToWorldFrame
   */
  static Vec3 pointToWorldFrame(Vec3 position,Quaternion quaternion,Vec3 localPoint, [Vec3? result]) {
    result ??= Vec3();
    quaternion.vmult(localPoint, result);
    result.vadd(position, result);
    return result;
  }

  /**
   * vectorToWorldFrame
   */
  static Vec3 vectorToWorldFrame(Quaternion quaternion,Vec3 localVector, [Vec3? result]){
    result ??= Vec3();
    quaternion.vmult(localVector, result);
    return result;
  }

  /**
   * vectorToLocalFrame
   */
  static Vec3 vectorToLocalFrame(Vec3 position,Quaternion quaternion,Vec3 worldVector, [Vec3? result]){
    result ??= Vec3();
    quaternion.w *= -1;
    quaternion.vmult(worldVector, result);
    quaternion.w *= -1;
    return result;
  }
}
