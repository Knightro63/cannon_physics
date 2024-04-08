import '../objects/rigid_body.dart';
import  '../rigid_body_shapes/shape.dart';
import 'package:vector_math/vector_math.dart';

/// Storage for Ray casting data
class RaycastResult {
  /// rayFromWorld
  Vector3 rayFromWorld = Vector3.zero();

  /// rayToWorld
  Vector3 rayToWorld = Vector3.zero();

  /// hitNormalWorld
  Vector3 hitNormalWorld = Vector3.zero();

  /// hitPointWorld
  Vector3 hitPointWorld = Vector3.zero();

  /// hasHit
  bool hasHit = false;

  /// shape
  Shape? shape;

  /// body
  Body? body;
  
  /// The index of the hit triangle, if the hit shape was a trimesh
  int hitFaceIndex = -1;

  /// Distance to the hit. Will be set to -1 if there was no hit
  double distance = -1;

  /// If the ray should stop traversing the bodies
  bool shouldStop = false;

  /// Reset all result data.
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

  /// abort
  void abort() {
    shouldStop = true;
  }

  /// Set result data.
  void set(
    Vector3 rayFromWorld,
    Vector3 rayToWorld,
    Vector3 hitNormalWorld,
    Vector3 hitPointWorld,
    Shape shape,
    Body body,
    double distance
  ){
    this.rayFromWorld.setFrom(rayFromWorld);
    this.rayToWorld.setFrom(rayToWorld);
    this.hitNormalWorld.setFrom(hitNormalWorld);
    this.hitPointWorld.setFrom(hitPointWorld);
    this.shape = shape;
    this.body = body;
    this.distance = distance;
  }
}
