import '../utils/pool.dart';
import '../math/vec3.dart';

class Vec3Pool extends Pool {
  Object type = Vec3();

  /// Construct a vector
  @override
  Vec3 constructObject(){
    return Vec3();
  }
}
