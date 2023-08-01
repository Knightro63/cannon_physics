import '../math/vec3.dart';
import '../objects/body.dart';
import  '../shapes/shape.dart';

/**
 * Storage for Ray casting data
 */
class RaycastResult {
  /**
   * rayFromWorld
   */
  Vec3 rayFromWorld = Vec3();
  /**
   * rayToWorld
   */
  Vec3 rayToWorld = Vec3();
  /**
   * hitNormalWorld
   */
  Vec3 hitNormalWorld = Vec3();
  /**
   * hitPointWorld
   */
  Vec3 hitPointWorld = Vec3();
  /**
   * hasHit
   */
  bool hasHit = false;
  /**
   * shape
   */
  Shape? shape;
  /**
   * body
   */
  Body? body;
  /**
   * The index of the hit triangle, if the hit shape was a trimesh
   */
  int hitFaceIndex = -1;
  /**
   * Distance to the hit. Will be set to -1 if there was no hit
   */
  double distance = -1;
  /**
   * If the ray should stop traversing the bodies
   */
  bool shouldStop = false;

  /**
   * Reset all result data.
   */
  void reset() {
    rayFromWorld.setZero();
    rayToWorld.setZero();
    hitNormalWorld.setZero();
    hitPointWorld.setZero();
    hasHit = false;
    shape = null;
    body = null;
    hitFaceIndex = -1;
    distance = -1;
    shouldStop = false;
  }

  /**
   * abort
   */
  void abort() {
    shouldStop = true;
  }

  /**
   * Set result data.
   */
  void set(
    Vec3 rayFromWorld,
    Vec3 rayToWorld,
    Vec3 hitNormalWorld,
    Vec3 hitPointWorld,
    Shape shape,
    Body body,
    double distance
  ){
    this.rayFromWorld.copy(rayFromWorld);
    this.rayToWorld.copy(rayToWorld);
    this.hitNormalWorld.copy(hitNormalWorld);
    this.hitPointWorld.copy(hitPointWorld);
    this.shape = shape;
    this.body = body;
    this.distance = distance;
  }
}
