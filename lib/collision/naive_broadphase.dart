import 'broadphase.dart';
import  '../objects/body.dart';
import  '../collision/aabb.dart';
import  '../world/world_class.dart';

/// Naive broadphase implementation, used in lack of better ones.
///
/// The naive broadphase looks at all possible pairs without restriction, therefore it has complexity N^2 _(which is bad)_
class NaiveBroadphase extends Broadphase {
  NaiveBroadphase():super();

  /// Get all the collision pairs in the physics world
  @override
  void collisionPairs(World world, List<Body> pairs1, List<Body> pairs2) {
    final bodies = world.bodies;
    final int n = bodies.length;
    Body bi;
    Body bj;

    // Naive N^2 ftw!
    for (int i = 0; i != n; i++) {
      for (int j = 0; j != i; j++) {
        bi = bodies[i];
        bj = bodies[j];

        if (!needBroadphaseCollision(bi, bj)) {
          continue;
        }

        intersectionTest(bi, bj, pairs1, pairs2);
      }
    }
  }


  /// Returns all the bodies within an AABB.
  /// [result] An array to store resulting bodies in.
  @override
  List<Body> aabbQuery(World world, AABB aabb, [List<Body>? result]){
    result ??= [];
    
    for (int i = 0; i < world.bodies.length; i++) {
      final b = world.bodies[i];

      if (b.aabbNeedsUpdate) {
        b.updateAABB();
      }

      // Ugly hack until Body gets aabb
      if (b.aabb.overlaps(aabb)) {
        result.add(b);
      }
    }

    return result;
  }
}
