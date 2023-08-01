import '../utils/pool.dart';
import '../math/vec3.dart';

/**
 * Vec3Pool
 */
class Vec3Pool extends Pool {
  Object type = Vec3();

  /**
   * Construct a vector
   */
  Vec3 constructObject(){
    return Vec3();
  }
}
