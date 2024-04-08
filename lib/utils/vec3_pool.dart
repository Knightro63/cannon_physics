import '../utils/pool.dart';
import 'package:vector_math/vector_math.dart';

class Vec3Pool extends Pool {
  Vec3Pool(){
    type = Vector3.zero();
  }

  /// Construct a vector
  @override
  Vector3 constructObject(){
    return Vector3.zero();
  }
}
