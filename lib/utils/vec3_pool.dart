import '../utils/pool.dart';
import '../math/vec3.dart';

/**
 * Vec3Pool
 */
class Vec3Pool extends Pool {
  Vec3 type = Vec3();

  /**
   * Construct a vector
   */
  Vec3 constructObject(){
    return Vec3();
  }
}
