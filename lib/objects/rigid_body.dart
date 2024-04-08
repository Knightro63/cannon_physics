import 'dart:math' as math;
import 'package:cannon_physics/utils/logger.dart';

import '../utils/event_target.dart';
import '../math/vec3.dart';
import '../math/mat3.dart';
import '../math/quaternion.dart';
import '../collision/aabb.dart';
import '../rigid_body_shapes/box.dart';
import '../rigid_body_shapes/shape.dart';
import '../material/material.dart';
import '../world/world_class.dart';
import 'package:vector_math/vector_math.dart';

enum BodyTypes{dynamic,static,kinematic}
enum BodySleepStates{awake,sleepy,sleeping,}

/// Base class for all body types.
/// @example;
///     final shape = CANNON.Sphere(1);
///     final body = CANNON.Body({
///       1 *mass;
///       shape,;
///     })
///     world.addBody(body);
class Body extends EventTarget {
  Body({
    this.collisionFilterGroup = 1,
    this.collisionFilterMask = -1,
    this.collisionResponse = true,
    Vector3? position,
    Vector3? velocity,
    this.mass = 0,
    this.material,
    this.linearDamping = 0.01,
    BodyTypes? type,
    this.allowSleep = true,
    this.sleepSpeedLimit = 0.1,
    this.sleepTimeLimit = 1,
    Quaternion? quaternion,
    Vector3? angularVelocity,
    this.fixedRotation = false,
    this.angularDamping = 0.01,
    Vector3? linearFactor,
    Vector3? angularFactor,
    Shape? shape,
    this.isTrigger = false,
  }):super() {
    id = Body.idCounter++;
    if (position != null) {
      this.position.setFrom(position);
      previousPosition.setFrom(position);
      interpolatedPosition.setFrom(position);
      initPosition.setFrom(position);
    }
    if (velocity != null) {
      this.velocity.setFrom(velocity);
    }

    invMass = mass > 0 ? 1.0 / mass : 0;
    this.type = mass <= 0.0 ? BodyTypes.static : BodyTypes.dynamic;
    // if (type == BodyTypes.static) {
    //   this.type = type!;
    // }
    this.type = type ?? this.type;
    if (quaternion != null) {
      this.quaternion.setFrom(quaternion);
      previousQuaternion.setFrom(quaternion);
      interpolatedQuaternion.setFrom(quaternion);
      initQuaternion.setFrom(quaternion);
    }
    if (angularVelocity != null) {
      this.angularVelocity.setFrom(angularVelocity);
    }
    if (linearFactor != null) {
      this.linearFactor.setFrom(linearFactor);
    }
    if (angularFactor != null) {
      this.angularFactor.setFrom(angularFactor);
    }
    if (shape != null) {
      addShape(shape);
    }

    updateMassProperties();
  }

  static int idCounter = 0;

  /// Dispatched after two bodies collide. This event is dispatched on each
  /// of the two bodies involved in the collision.;
  /// @event collide;
  /// @param body The body that was involved in the collision.;
  /// @param contact The details of the collision.;
  static String collideEventName = 'collide';

  /// Dispatched after a sleeping body has woken up.
  /// @event wakeup;
  static BodyEvent wakeupEvent = BodyEvent(type:'wakeup');

  /// Dispatched after a body has gone in to the sleepy state.
  /// @event sleepy;
  static BodyEvent sleepyEvent = BodyEvent(type:'sleepy');

  /// Dispatched after a body has fallen asleep.
  /// @event sleep;
  static BodyEvent sleepEvent = BodyEvent(type:'sleep');

  /// Identifier of the body.
  late int id;

  /// Position of body in World.bodies. Updated by World and used in ArrayCollisionMatrix.
  int index = -1;

  /// Reference to the world the body is living in.
  World? world;

  Vector3 vlambda = Vector3.zero();

  /// The collision group the body belongs to.
  int collisionFilterGroup;

  /// The collision group the body can collide with.
  int collisionFilterMask;

  /// Whether to produce contact forces when in contact with other bodies. Note that contacts will be generated, but they will be disabled - i.e. "collide" events will be raised, but forces will not be altered.
  bool collisionResponse;

  /// World space position of the body.
  Vector3 position = Vector3.zero();

  Vector3 previousPosition = Vector3.zero();

  /// Interpolated position of the body.
  Vector3 interpolatedPosition = Vector3.zero();

  /// Initial position of the body.
  Vector3 initPosition = Vector3.zero();

  /// World space velocity of the body.
  Vector3 velocity = Vector3.zero();

  /// Initial velocity of the body.
  Vector3 initVelocity = Vector3.zero();

  /// Linear force on the body in world space.
  Vector3 force = Vector3.zero();

  /// The mass of the body.
  double mass;
  late double invMass;

  /// The physics material of the body. It defines the body interaction with other bodies.
  Material? material;

  /// How much to damp the body velocity each step. It can go from 0 to 1.
  double linearDamping;

  /// One of: `Body.DYNAMIC`, `Body.static` and `Body.kinematic`.
  late BodyTypes type;

  /// If true, the body will automatically fall to sleep.
  bool allowSleep;

  /// Current sleep state.
  BodySleepStates sleepState = BodySleepStates.awake;

  /// If the speed (the norm of the velocity) is smaller than this value, the body is considered sleepy.
  double sleepSpeedLimit;

  /// If the body has been sleepy for this sleepTimeLimit seconds, it is considered sleeping.
  num sleepTimeLimit;

  num timeLastSleepy = 0;

  bool wakeUpAfterNarrowphase = false;

  /// World space rotational force on the body, around center of mass.
  Vector3 torque = Vector3.zero();

  /// World space orientation of the body.
  Quaternion quaternion = Quaternion(0,0,0,1);

  /// Initial quaternion of the body.
  Quaternion initQuaternion = Quaternion(0,0,0,1);

  Quaternion previousQuaternion = Quaternion(0,0,0,1);

  /// Interpolated orientation of the body.
  Quaternion interpolatedQuaternion = Quaternion(0,0,0,1);

  /// Angular velocity of the body, in world space. Think of the angular velocity as a vector, which the body rotates around. The length of this vector determines how fast (in radians per second) the body rotates.
  Vector3 angularVelocity = Vector3.zero();

  /// Initial angular velocity of the body.
  Vector3 initAngularVelocity = Vector3.zero();

  /// List of Shapes that have been added to the body.
  List<Shape> shapes = [];

  /// Position of each Shape in the body, given in local Body space.
  List<Vector3> shapeOffsets = [];

  /// Orientation of each Shape, given in local Body space.
  List<Quaternion> shapeOrientations = [];

  /// The inertia of the body.
  Vector3 inertia = Vector3.zero();

  Vector3 invInertia = Vector3.zero();
  Matrix3 invInertiaWorld = Matrix3.zero();
  double invMassSolve = 0;
  Vector3 invInertiaSolve  = Vector3.zero();
  Matrix3 invInertiaWorldSolve = Matrix3.zero();

  /// Set to true if you don't want the body to rotate. Make sure to run .updateMassProperties() if you change this after the body creation.
  bool fixedRotation;

  /// How much to damp the body angular velocity each step. It can go from 0 to 1.
  double angularDamping;

  /// Use this property to limit the motion along any world axis. (1,1,1) will allow motion along all axes while (0,0,0) allows none.
  Vector3 linearFactor = Vector3(1,1,1);

  /// Use this property to limit the rotational motion along any world axis. (1,1,1) will allow rotation along all axes while (0,0,0) allows none.
  Vector3 angularFactor = Vector3(1,1,1);

  /// World space bounding box of the body and its shapes.
  AABB aabb = AABB();

  /// Indicates if the AABB needs to be updated before use.
  bool aabbNeedsUpdate = true;

  /// Total bounding radius of the Body including its shapes, relative to body.position.
  double boundingRadius = 0;
  Vector3 wlambda = Vector3.zero();

  /// When true the body behaves like a trigger. It does not collide
  /// with other bodies but collision events are still triggered.;
  bool isTrigger;

  final _tmpVec = Vector3.zero();
  final _tmpQuat = Quaternion(0,0,0,1);
  final _updateShapeAABB = AABB();
  final _uiwM1 = Matrix3.zero();
  final _uiwM2 = Matrix3.zero();
  //final _uiwM3 = Matrix3();

  final _bodyApplyRotateForce = Vector3.zero();

  final _bodyApplyLocalWorldForce = Vector3.zero();
  final _bodyApplyLocalForceRelativePointWorld = Vector3.zero();

  final _bodyApplyImpulseVelo = Vector3.zero();
  final _bodyApplyImpulseRotVelo = Vector3.zero();

  final _bodyApplyLocalImpulseWorldImpulse = Vector3.zero();
  final _bodyApplyLocalImpulseRelativePoint = Vector3.zero();

  final _bodyUpdateMassPropertiesHalfExtents = Vector3.zero();

  /// Wake the body up.
  void wakeUp(){
    final prevState = sleepState;
    sleepState = BodySleepStates.awake;
    wakeUpAfterNarrowphase = false;
    if (prevState == BodySleepStates.sleeping) {
      dispatchEvent(Body.wakeupEvent);
    }
  }

  /// Force body sleep
  void sleep(){
    sleepState = BodySleepStates.sleeping;
    velocity.setValues(0, 0, 0);
    angularVelocity.setValues(0, 0, 0);
    wakeUpAfterNarrowphase = false;
  }

  /// Called every timestep to update internal sleep timer and change sleep state if needed.
  /// @param time The world time in seconds;
  void sleepTick(double time){
    if (allowSleep) {
      final sleepState = this.sleepState;
      final speedSquared = velocity.length2 + angularVelocity.length2;
      final speedLimitSquared =  math.pow(sleepSpeedLimit, 2);
      if (sleepState == BodySleepStates.awake && speedSquared < speedLimitSquared) {
        this.sleepState = BodySleepStates.sleepy; // Sleepy
        timeLastSleepy = time;
        dispatchEvent(Body.sleepyEvent);
      } 
      else if (sleepState == BodySleepStates.sleepy && speedSquared > speedLimitSquared) {
        wakeUp(); // Wake up
      } 
      else if (sleepState == BodySleepStates.sleepy && time - timeLastSleepy > sleepTimeLimit) {
        sleep(); // Sleeping
        dispatchEvent(Body.sleepEvent);
      }
    }
  }

  /// If the body is sleeping, it should be immovable / have infinite mass during solve. We solve it by having a separate "solve mass".
  void updateSolveMassProperties(){
    if (sleepState == BodySleepStates.sleeping || type == BodyTypes.kinematic) {
      invMassSolve = 0;
      invInertiaSolve.setZero();
      invInertiaWorldSolve.setZero();
    } 
    else {
      invMassSolve = invMass;
      invInertiaSolve.setFrom(invInertia);
      invInertiaWorldSolve.setFrom(invInertiaWorld);
    }
  }

  /// Convert a world point to local body frame.
  Vector3 pointToLocalFrame(Vector3 worldPoint,Vector3? result){
    result ??= Vector3.zero();
    worldPoint.sub2(position, result);
    quaternion..conjugate()..vmult(result, result);
    return result;
  }

  /// Convert a world vector to local body frame
  Vector3 vectorToLocalFrame(Vector3 worldVector,[Vector3? result]){
    result ??= Vector3.zero();
    quaternion..conjugate()..vmult(worldVector, result);
    return result;
  }

  /// Convert a local body point to world frame.
  Vector3 pointToWorldFrame(Vector3 localPoint, Vector3? result){
    result ??= Vector3.zero();
    quaternion.vmult(localPoint, result);
    result.add2(position, result);
    return result;
  }

  /// Convert a local body point to world frame.
  Vector3 vectorToWorldFrame(Vector3 localVector, [Vector3? result]){
    result ??= Vector3.zero();
    quaternion.vmult(localVector, result);
    return result;
  }

  /// Add a shape to the body with a local offset and orientation.
  /// @return The body object, for chainability.
  Body addShape(Shape shape,[Vector3? offset, Quaternion? orientation]){
    final off = Vector3.zero();
    final or = Quaternion(0,0,0,1);

    if (offset != null) {
      off.setFrom(offset);
    }
    if (orientation != null) {
      or.setFrom(orientation);
    }

    shapes.add(shape);
    shapeOffsets.add(off);
    shapeOrientations.add(or);
    updateMassProperties();
    updateBoundingRadius();

    aabbNeedsUpdate = true;
    shape.body = this;

    return this;
  }

  /// Remove a shape from the body.
  /// @return The body object, for chainability.
  Body removeShape(Shape shape){
    final index = shapes.indexOf(shape);

    if (index == -1) {
      logger?.warning('Shape does not belong to the body');
      return this;
    }

    shapes.removeAt(index);
    shapeOffsets.removeAt(index);
    shapeOrientations.removeAt(index);
    updateMassProperties();
    updateBoundingRadius();

    aabbNeedsUpdate = true;

    shape.body = null;

    return this;
  }

  /// Update the bounding radius of the body. Should be done if any of the shapes are changed.
  void updateBoundingRadius(){
    final shapes = this.shapes;
    final shapeOffsets = this.shapeOffsets;
    final N = shapes.length;
    double radius = 0;

    for (int i = 0; i != N; i++) {
      final shape = shapes[i];
      shape.updateBoundingSphereRadius();
      final offset = shapeOffsets[i].length;
      final r = shape.boundingSphereRadius;
      if (offset + r > radius) {
        radius = offset + r;
      }
    }

    boundingRadius = radius;
  }

  /// Updates the .aabb
  void updateAABB(){
    final shapes = this.shapes;
    final shapeOffsets = this.shapeOffsets;
    final shapeOrientations = this.shapeOrientations;
    final N = shapes.length;
    final offset = _tmpVec;
    final orientation = _tmpQuat;
    final bodyQuat = quaternion;
    final aabb = this.aabb;
    final shapeAABB = _updateShapeAABB;

    for (int i = 0; i != N; i++) {
      final shape = shapes[i];

      // Get shape world position
      bodyQuat.vmult(shapeOffsets[i], offset);
      offset.add2(position, offset);

      // Get shape world quaternion
      bodyQuat.multiply2(shapeOrientations[i], orientation);

      // Get shape AABB
      shape.calculateWorldAABB(offset, orientation, shapeAABB.lowerBound, shapeAABB.upperBound);

      if (i == 0) {
        aabb.copy(shapeAABB);
      } else {
        aabb.extend(shapeAABB);
      }
    }

    aabbNeedsUpdate = false;
  }

  /// Update `.inertiaWorld` and `.invInertiaWorld`
  void updateInertiaWorld([bool force = false]) {
    final I = invInertia;
    if (I.x == I.y && I.y == I.z && !force) {
      // If inertia M = s*I, where I is identity and s a scalar, then
      //    R*M*R' = R*(s*I)*R' = s*R*I*R' = s*R*R' = s*I = M
      // where R is the rotation matrix.
      // In other words, we don't have to transform the inertia if all
      // inertia diagonal entries are equal.
    } else {
      final m1 = _uiwM1;
      final m2 = _uiwM2;
      m1.setRotationFromQuaternion(quaternion);
      m2..setFrom(m1)..transpose();
      m1.vscale(I, m1);
      m1.multiply2(m2, invInertiaWorld);
    }
  }

  /// Apply force to a point of the body. This could for example be a point on the Body surface.
  /// Applying force this way will add to Body.force and Body.torque.
  /// @param force The amount of force to add.
  /// @param relativePoint A point relative to the center of mass to apply the force on.
  void applyForce(Vector3 force, [Vector3? relativePoint]) {
    relativePoint ??= Vector3.zero();
    // Needed?
    if (type != BodyTypes.dynamic) {
      return;
    }

    if (sleepState == BodySleepStates.sleeping) {
      wakeUp();
    }

    // Compute produced rotational force
    final rotForce = _bodyApplyRotateForce;
    relativePoint.cross2(force, rotForce);

    // Add linear force
    this.force.add2(force, this.force);

    // Add rotational force
    torque.add2(rotForce, torque);
  }

  /// Apply force to a local point in the body.
  /// @param force The force vector to apply, defined locally in the body frame.
  /// @param localPoint A local point in the body to apply the force on.
  void applyLocalForce(Vector3 localForce, Vector3 localPoint){
    if (type != BodyTypes.dynamic) {
      return;
    }

    final worldForce = _bodyApplyLocalWorldForce;
    final relativePointWorld = _bodyApplyLocalForceRelativePointWorld;

    // Transform the force vector to world space
    vectorToWorldFrame(localForce, worldForce);
    vectorToWorldFrame(localPoint, relativePointWorld);

    applyForce(worldForce, relativePointWorld);
  }

  /// Apply torque to the body.
  /// @param torque The amount of torque to add.
  void applyTorque(Vector3 torque){
    if (type != BodyTypes.dynamic) {
      return;
    }

    if (sleepState == BodySleepStates.sleeping) {
      wakeUp();
    }

    // Add rotational force
    this.torque.add2(torque, this.torque);
  }

  /// Apply impulse to a point of the body. This could for example be a point on the Body surface.
  /// An impulse is a force added to a body during a short period of time (impulse = force * time).
  /// Impulses will be added to Body.velocity and Body.angularVelocity.;
  /// @param impulse The amount of impulse to add.;
  /// @param relativePoint A point relative to the center of mass to apply the force on.
  void applyImpulse(Vector3 impulse, [Vector3? relativePoint]){
    relativePoint ??= Vector3.zero();
    if (type != BodyTypes.dynamic) {
      return;
    }

    if (sleepState == BodySleepStates.sleeping) {
      wakeUp();
    }

    // Compute point position relative to the body center
    final Vector3 r = relativePoint;

    // Compute produced central impulse velocity
    final velo = _bodyApplyImpulseVelo;
    velo.setFrom(impulse);
    velo.scale2(invMass, velo);

    // Add linear impulse
    velocity.add2(velo, velocity);

    // Compute produced rotational impulse velocity
    final rotVelo = _bodyApplyImpulseRotVelo;
    r.cross2(impulse, rotVelo);

    /*
     rotVelo.x *= this.invInertia.x;
     rotVelo.y *= this.invInertia.y;
     rotVelo.z *= this.invInertia.z;
     */
    invInertiaWorld.vmult(rotVelo, rotVelo);

    // Add rotational Impulse
    angularVelocity.add2(rotVelo, angularVelocity);
  }

  /// Apply locally-defined impulse to a local point in the body.
  /// @param force The force vector to apply, defined locally in the body frame.
  /// @param localPoint A local point in the body to apply the force on.
  void applyLocalImpulse(Vector3 localImpulse, Vector3 localPoint){
    if (type != BodyTypes.dynamic) {
      return;
    }

    final worldImpulse = _bodyApplyLocalImpulseWorldImpulse;
    final relativePointWorld = _bodyApplyLocalImpulseRelativePoint;

    // Transform the force vector to world space
    vectorToWorldFrame(localImpulse, worldImpulse);
    vectorToWorldFrame(localPoint, relativePointWorld);

    applyImpulse(worldImpulse, relativePointWorld);
  }

  /// Should be called whenever you change the body shape or mass.
  void updateMassProperties(){
    final halfExtents = _bodyUpdateMassPropertiesHalfExtents;

    invMass = mass > 0 ? 1.0 / mass : 0;
    final I = inertia;
    final fixed =fixedRotation;

    // Approximate with AABB box
    updateAABB();
    halfExtents.setValues(
      (aabb.upperBound.x - aabb.lowerBound.x) / 2,
      (aabb.upperBound.y - aabb.lowerBound.y) / 2,
      (aabb.upperBound.z - aabb.lowerBound.z) / 2
    );
    Box.calculateInertia(halfExtents, mass, I);

    invInertia.setValues(
      I.x > 0 && !fixed ? 1.0 / I.x : 0,
      I.y > 0 && !fixed ? 1.0 / I.y : 0,
      I.z > 0 && !fixed ? 1.0 / I.z : 0
    );
    updateInertiaWorld(true);
  }

  /// Get world velocity of a point in the body.
  /// @param worldPoint;
  /// @param result;
  /// @return The result vector.;
  Vector3 getVelocityAtWorldPoint(Vector3 worldPoint, Vector3 result){
    final Vector3 r = Vector3.zero();
    worldPoint.sub2(position, r);
    angularVelocity.cross2(r, result);
    velocity.add2(result, result);
    return result;
  }

  /// Move the body forward in time.
  /// @param dt Time step;
  /// @param quatNormalize Set to true to normalize the body quaternion;
  /// @param quatNormalizeFast If the quaternion should be normalized using "fast" quaternion normalization;
  void integrate(double dt,[bool quatNormalize = false,bool quatNormalizeFast = false]){
    // Save previous position
    previousPosition.setFrom(position);
    previousQuaternion.setFrom(quaternion);

    if (!(type == BodyTypes.dynamic || type == BodyTypes.kinematic) || sleepState == BodySleepStates.sleeping) {
      // Only for dynamic
      return;
    }

    final velo = velocity;
    final angularVelo = angularVelocity;
    final pos = position;
    final force = this.force;
    final torque = this.torque;
    final quat = quaternion;
    final invMass = this.invMass;
    final invInertia = invInertiaWorld;
    final linearFactor = this.linearFactor;

    final iMdt = invMass * dt;
    velo.x += force.x * iMdt * linearFactor.x;
    velo.y += force.y * iMdt * linearFactor.y;
    velo.z += force.z * iMdt * linearFactor.z;

    final e = invInertia.storage;
    final angularFactor = this.angularFactor;
    final tx = torque.x * angularFactor.x;
    final ty = torque.y * angularFactor.y;
    final tz = torque.z * angularFactor.z;
    angularVelo.x += dt * (e[0] * tx + e[1] * ty + e[2] * tz);
    angularVelo.y += dt * (e[3] * tx + e[4] * ty + e[5] * tz);
    angularVelo.z += dt * (e[6] * tx + e[7] * ty + e[8] * tz);

    // Use velocity  - leap frog
    pos.x += velo.x * dt;
    pos.y += velo.y * dt;
    pos.z += velo.z * dt;

    quat.integrate(angularVelocity, dt, this.angularFactor, quat);

    if (quatNormalize) {
      if (quatNormalizeFast) {
        quat.normalizeFast();
      } else {
        quat.normalize();
      }
    }

    aabbNeedsUpdate = true;

    // Update world inertia
    updateInertiaWorld();
  }
}
