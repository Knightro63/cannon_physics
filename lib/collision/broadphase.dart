import 'dart:math' as math;
import  '../objects/body.dart';
import  '../math/vec3.dart';
import  '../math/quaternion.dart';
import  '../collision/aabb.dart';
import  '../world/world.dart' hide Body;

// Temp objects
final Vec3 Broadphase_collisionPairs_r = Vec3();

final Vec3 Broadphase_collisionPairs_normal = Vec3();
final Quaternion Broadphase_collisionPairs_quat = Quaternion();
final Vec3 Broadphase_collisionPairs_relpos = Vec3();

final Map<String,dynamic> Broadphase_makePairsUnique_temp = {};
final List<Body> Broadphase_makePairsUnique_p1=[];
final List<Body> Broadphase_makePairsUnique_p2=[];

final Vec3 bsc_dist = Vec3();

/**
 * Base class for broadphase implementations
 * @author schteppe
 */
class Broadphase {
  /**
   * The world to search for collisions in.
   */
  World? world;
  /**
   * If set to true, the broadphase uses bounding boxes for intersection tests, else it uses bounding spheres.
   */
  bool useBoundingBoxes;
  /**
   * Set to true if the objects in the world moved.
   */
  bool dirty;

  Broadphase({
    this.world,
    this.useBoundingBoxes = false,
    this.dirty = true
  });

  /**
   * Get the collision pairs from the world
   * @param world The world to search in
   * @param p1 Empty array to be filled with body objects
   * @param p2 Empty array to be filled with body objects
   */
  void collisionPairs(World world,List<Body> p1,List<Body> p2) {
    throw('collisionPairs not implemented for this BroadPhase class!');
  }

  /**
   * Check if a body pair needs to be intersection tested at all.
   */
  bool needBroadphaseCollision(Body bodyA, Body bodyB) {
    // Check collision filter masks
    if (
      (bodyA.collisionFilterGroup & bodyB.collisionFilterMask) == 0 ||
      (bodyB.collisionFilterGroup & bodyA.collisionFilterMask) == 0
    ) {
      return false;
    }

    // Check types
    if (
      (bodyA.type == BodyTypes.static || bodyA.sleepState == BodySleepStates.sleeping) &&
      (bodyB.type == BodyTypes.static || bodyB.sleepState == BodySleepStates.sleeping)
    ) {
      // Both bodies are static or sleeping. Skip.
      return false;
    }

    return true;
  }

  /**
   * Check if the bounding volumes of two bodies intersect.
   */
  void intersectionTest(Body bodyA, Body bodyB, List<Body> pairs1, List<Body> pairs2) {
    if (useBoundingBoxes) {
      doBoundingBoxBroadphase(bodyA, bodyB, pairs1, pairs2);
    } else {
      doBoundingSphereBroadphase(bodyA, bodyB, pairs1, pairs2);
    }
  }

  /**
   * Check if the bounding spheres of two bodies are intersecting.
   * @param pairs1 bodyA is appended to this array if intersection
   * @param pairs2 bodyB is appended to this array if intersection
   */
  void doBoundingSphereBroadphase(Body bodyA, Body bodyB, List<Body> pairs1, List<Body> pairs2) {
    final r = Broadphase_collisionPairs_r;
    bodyB.position.vsub(bodyA.position, r);
    final boundingRadiusSum2 = math.pow(bodyA.boundingRadius + bodyB.boundingRadius,2);// ** 2
    final norm2 = r.lengthSquared();
    if (norm2 < boundingRadiusSum2) {
      pairs1.add(bodyA);
      pairs2.add(bodyB);
    }
  }

  /**
   * Check if the bounding boxes of two bodies are intersecting.
   */
  void doBoundingBoxBroadphase(Body bodyA, Body bodyB,List<Body> pairs1,List<Body> pairs2) {
    if (bodyA.aabbNeedsUpdate) {
      bodyA.updateAABB();
    }
    if (bodyB.aabbNeedsUpdate) {
      bodyB.updateAABB();
    }

    // Check AABB / AABB
    if (bodyA.aabb.overlaps(bodyB.aabb)) {
      pairs1.add(bodyA);
      pairs2.add(bodyB);
    }
  }

  /**
   * Removes duplicate pairs from the pair arrays.
   */
  void makePairsUnique(List<Body> pairs1,List<Body> pairs2) {
    final t = Broadphase_makePairsUnique_temp;
    final p1 = Broadphase_makePairsUnique_p1;
    final p2 = Broadphase_makePairsUnique_p2;
    final N = pairs1.length;

    for (int i = 0; i != N; i++) {
      p1[i] = pairs1[i];
      p2[i] = pairs2[i];
    }

    pairs1.length = 0;
    pairs2.length = 0;

    for (int i = 0; i != N; i++) {
      final id1 = p1[i].id;
      final id2 = p2[i].id;
      final key = id1 < id2 ? '$id1,$id2' : '$id2,$id1';
      t[key] = i;
      //t.keys.add(key);
    }

    for (int i = 0; i != t.keys.length; i++) {
      final key = t.keys;
      final pairIndex = t[key];
      pairs1.add(p1[pairIndex]);
      pairs2.add(p2[pairIndex]);
      t.remove(key);//delete t[key];
    }
  }

  /**
   * To be implemented by subcasses
   */
  void setWorld(World world){

  }

  /**
   * Check if the bounding spheres of two bodies overlap.
   */
  static bool boundingSphereCheck(Body bodyA, Body bodyB) {
    final Vec3 dist = Vec3(); // bsc_dist;
    bodyA.position.vsub(bodyB.position, dist);
    final sa = bodyA.shapes[0];
    final sb = bodyB.shapes[0];
    return math.pow(sa.boundingSphereRadius + sb.boundingSphereRadius, 2) > dist.lengthSquared();
  }

  /**
   * Returns all the bodies within the AABB.
   */
  List<Body> aabbQuery(World world,AABB aabb,List<Body> result) {
    print('.aabbQuery is not implemented in this Broadphase subclass.');
    return [];
  }
}
