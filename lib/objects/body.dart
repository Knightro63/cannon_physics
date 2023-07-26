import '../utils/event_target.dart';
import '../math/vec3.dart';
import '../math/mat3.dart';
import '../math/quaternion.dart';
import '../collision/aabb.dart';
import '../shapes/box.dart';
import '../shapes/shape.dart';
import '../material/material.dart';
import '../world/world.dart';

/**
 * BODY_TYPES
 */
enum BodyTypes{dynamic,static,kinematic}

/**
 * BODY_SLEEP_STATES
 */
enum BodySleepStates{awake,sleepy,sleeping,}

/**
 * BodySleepState
 */
//export type BodySleepState =  BODY_SLEEP_STATES[keyof  BODY_SLEEP_STATES];

//export type BodyOptions = ConstructorParameters< Body>[0];

/**
 * Base class for all body types.
 * @example;
 *     final shape = CANNON.Sphere(1);
 *     final body = CANNON.Body({
1 *mass;
 *       shape,;
 *     })
 *     world.addBody(body);
 */
class Body extends EventTarget {
  static int idCounter = 0;

  /**
   * Dispatched after two bodies collide. This event is dispatched on each
   * of the two bodies involved in the collision.;
   * @event collide;
   * @param body The body that was involved in the collision.;
   * @param contact The details of the collision.;
   */
  static String collide_event_name = 'collide';

  /**
   * A dynamic body is fully simulated. Can be moved manually by the user, but normally they move according to forces. A dynamic body can collide with all body types. A dynamic body always has finite, non-zero mass.
   */
  static BodyTypes dynamic = BodyTypes.dynamic;

  /**
   * A static body does not move during simulation and behaves as if it has infinite mass. Static bodies can be moved manually by setting the position of the body. The velocity of a static body is always zero. Static bodies do not collide with other static or kinematic bodies.
   */
  static BodyTypes STATIC = BodyTypes.static;

  /**
   * A kinematic body moves under simulation according to its velocity. They do not respond to forces. They can be moved manually, but normally a kinematic body is moved by setting its velocity. A kinematic body behaves as if it has infinite mass. Kinematic bodies do not collide with other static or kinematic bodies.
   */
  static BodyTypes KINEMATIC = BodyTypes.kinematic;

  /**
   * AWAKE
   */
  static BodySleepStates awake = BodySleepStates.awake;
  /**
   * SLEEPY
   */
  static BodySleepStates sleepy = BodySleepStates.sleepy;
  /**
   * SLEEPING
   */
  static BodySleepStates sleeping = BodySleepStates.sleeping;

  /**
   * Dispatched after a sleeping body has woken up.
   * @event wakeup;
   */
 static Map<String,String> wakeupEvent={'type':'wakeup'};

  /**
   * Dispatched after a body has gone in to the sleepy state.
   * @event sleepy;
   */
 static Map<String,String> sleepyEvent={'type':'sleepy'};

  /**
   * Dispatched after a body has fallen asleep.
   * @event sleep;
   */
  static Map<String,String> sleepEvent={'type':'sleepy'};

  /**
   * Identifier of the body.
   */
  int id;

  /**
   * Position of body in World.bodies. Updated by World and used in ArrayCollisionMatrix.
   */
  int index;

  /**
   * Reference to the world the body is living in.
   */
  World? world;

  Vec3 vlambda;

  /**
   * The collision group the body belongs to.
   * @default 1;
   */
  int collisionFilterGroup;

  /**
   * The collision group the body can collide with.
   * @default -1;
   */
  int collisionFilterMask;

  /**
   * Whether to produce contact forces when in contact with other bodies. Note that contacts will be generated, but they will be disabled - i.e. "collide" events will be raised, but forces will not be altered.
   */
  bool collisionResponse;

  /**
   * World space position of the body.
   */
  Vec3 position;

  Vec3 previousPosition;

  /**
   * Interpolated position of the body.
   */
  Vec3 interpolatedPosition;

  /**
   * Initial position of the body.
   */
  Vec3 initPosition;

  /**
   * World space velocity of the body.
   */
  Vec3 velocity;

  /**
   * Initial velocity of the body.
   */
  Vec3 initVelocity;

  /**
   * Linear force on the body in world space.
   */
  Vec3 force;

  /**
   * The mass of the body.
   * @default 0;
   */
  num mass;

  num invMass;

  /**
   * The physics material of the body. It defines the body interaction with other bodies.
   */
  Material? material;

  /**
   * How much to damp the body velocity each step. It can go from 0 to 1.
   * @default 0.01;
   */
  num linearDamping;

  /**
   * One of: `Body.DYNAMIC`, `Body.STATIC` and `Body.KINEMATIC`.
   */
  BodyType type;

  /**
   * If true, the body will automatically fall to sleep.
   * @default true;
   */
  bool allowSleep;

  /**
   * Current sleep state.
   */
  BodySleepState sleepState;

  /**
   * If the speed (the norm of the velocity) is smaller than this value, the body is considered sleepy.
   * @default 0.1;
   */
  num sleepSpeedLimit;

  /**
   * If the body has been sleepy for this sleepTimeLimit seconds, it is considered sleeping.
   * @default 1;
   */
  num sleepTimeLimit;

  num timeLastSleepy;

  bool wakeUpAfterNarrowphase;

  /**
   * World space rotational force on the body, around center of mass.
   */
  Vec3 torque;

  /**
   * World space orientation of the body.
   */
  Quaternion quaternion;

  /**
   * Initial quaternion of the body.
   */
  Quaternion initQuaternion;

  Quaternion previousQuaternion;

  /**
   * Interpolated orientation of the body.
   */
  Quaternion interpolatedQuaternion;

  /**
   * Angular velocity of the body, in world space. Think of the angular velocity as a vector, which the body rotates around. The length of this vector determines how fast (in radians per second) the body rotates.
   */
  Vec3 angularVelocity;

  /**
   * Initial angular velocity of the body.
   */
  Vec3 initAngularVelocity;

  /**
   * List of Shapes that have been added to the body.
   */
  List<Shape> shapes;

  /**
   * Position of each Shape in the body, given in local Body space.
   */
  List<Vec3> shapeOffsets;

  /**
   * Orientation of each Shape, given in local Body space.
   */
  List<Quaternion> shapeOrientations;

  /**
   * The inertia of the body.
   */
  Vec3 inertia;

  Vec3 invInertia;
  Mat3 invInertiaWorld;
  num invMassSolve;
  Vec3 invInertiaSolve;
  Mat3 invInertiaWorldSolve;

  /**
   * Set to true if you don't want the body to rotate. Make sure to run .updateMassProperties() if you change this after the body creation.
   * @default false;
   */
  bool fixedRotation;

  /**
   * How much to damp the body angular velocity each step. It can go from 0 to 1.
   * @default 0.01;
   */
  num angularDamping;

  /**
   * Use this property to limit the motion along any world axis. (1,1,1) will allow motion along all axes while (0,0,0) allows none.
   */
  Vec3 linearFactor;

  /**
   * Use this property to limit the rotational motion along any world axis. (1,1,1) will allow rotation along all axes while (0,0,0) allows none.
   */
  Vec3 angularFactor;

  /**
   * World space bounding box of the body and its shapes.
   */
  AABB aabb;

  /**
   * Indicates if the AABB needs to be updated before use.
   */
  bool aabbNeedsUpdate;

  /**
   * Total bounding radius of the Body including its shapes, relative to body.position.
   */
  num boundingRadius;
  Vec3 wlambda;
  /**
   * When true the body behaves like a trigger. It does not collide
   * with other bodies but collision events are still triggered.;
   * @default false;
   */
  bool isTrigger;

  Body({
      /**
       * The collision group the body belongs to.
       * @default 1;
       */
    num? collisionFilterGroup,
      /**
       * The collision group the body can collide with.
       * @default -1;
       */
    num? collisionFilterMask,

      /**
       * Whether to produce contact forces when in contact with other bodies. Note that contacts will be generated, but they will be disabled - i.e. "collide" events will be raised, but forces will not be altered.
       */
    bool? collisionResponse,
      /**
       * World space position of the body.
       */
    Vec3? position,
      /**
       * World space velocity of the body.
       */
    Vec3? velocity,
      /**
       * The mass of the body.
       * @default 0;
       */
    num? mass,
      /**
       * The physics material of the body. It defines the body interaction with other bodies.
       */
    Material? material,
      /**
       * How much to damp the body velocity each step. It can go from 0 to 1.
       * @default 0.01;
       */
    num? linearDamping,
      /**
       * One of: `Body.DYNAMIC`, `Body.STATIC` and `Body.KINEMATIC`.
       */
    BodyType? type,
      /**
       * If true, the body will automatically fall to sleep.
       * @default true;
       */
    bool? allowSleep,
      /**
       * If the speed (the norm of the velocity) is smaller than this value, the body is considered sleepy.
       * @default 0.1;
       */
    num? sleepSpeedLimit,
      /**
       * If the body has been sleepy for this sleepTimeLimit seconds, it is considered sleeping.
       * @default 1;
       */
    num? sleepTimeLimit,
      /**
       * World space orientation of the body.
       */
    Quaternion? quaternion,
      /**
       * Angular velocity of the body, in world space. Think of the angular velocity as a vector, which the body rotates around. The length of this vector determines how fast (in radians per second) the body rotates.
       */
    Vec3? angularVelocity,
      /**
       * Set to true if you don't want the body to rotate. Make sure to run .updateMassProperties() if you change this after the body creation.
       * @default false;
       */
    bool? fixedRotation,
      /**
       * How much to damp the body angular velocity each step. It can go from 0 to 1.
       * @default 0.01;
       */
    num? angularDamping,
      /**
       * Use this property to limit the motion along any world axis. (1,1,1) will allow motion along all axes while (0,0,0) allows none.
       */
    Vec3? linearFactor,
      /**
       * Use this property to limit the rotational motion along any world axis. (1,1,1) will allow rotation along all axes while (0,0,0) allows none.
       */
    Vec3? angularFactor,
      /**
       * Add a Shape to the body.
       */
    Shape? shape,
      /**
       * When true the body behaves like a trigger. It does not collide
       * with other bodies but collision events are still triggered.;
       * @default false;
       */
    bool? isTrigger,
    }
  ):super() {
    this.id = Body.idCounter++;
    this.index = -1;
    this.world = null;
    this.vlambda = Vec3();

    this.collisionFilterGroup =  collisionFilterGroup == 'number' ? collisionFilterGroup : 1
    this.collisionFilterMask =  collisionFilterMask == 'number' ? collisionFilterMask : -1
    this.collisionResponse =  collisionResponse == 'boolean' ? collisionResponse : true
    this.position = Vec3();
    this.previousPosition = Vec3();
    this.interpolatedPosition = Vec3();
    this.initPosition = Vec3();

    if (position) {
      this.position.copy(position);
      this.previousPosition.copy(position);
      this.interpolatedPosition.copy(position);
      this.initPosition.copy(position);
    }

    this.velocity = Vec3();

    if (velocity) {
      this.velocity.copy(velocity);
    }

    this.initVelocity = Vec3();
    this.force = Vec3()
    final mass =  mass == 'number' ? mass : 0
    this.mass = mass;
    this.invMass = mass > 0 ? 1.0 / mass : 0
    this.material = material || null;
    this.linearDamping =  linearDamping == 'number' ? linearDamping : 0.01

    this.type = mass <= 0.0 ? Body.STATIC : Body.DYNAMIC

    if ( type ==  Body.STATIC) {
      this.type = type!;
    }

    this.allowSleep =  allowSleep != null ? allowSleep : true
    this.sleepState = Body.AWAKE;
    this.sleepSpeedLimit =  sleepSpeedLimit != null ? sleepSpeedLimit : 0.1
    this.sleepTimeLimit =  sleepTimeLimit != null ? sleepTimeLimit : 1
    this.timeLastSleepy = 0;
    this.wakeUpAfterNarrowphase = false;

    this.torque = Vec3();
    this.quaternion = Quaternion();
    this.initQuaternion = Quaternion();
    this.previousQuaternion = Quaternion();
    this.interpolatedQuaternion = Quaternion();

    if (quaternion) {
      this.quaternion.copy(quaternion);
      this.initQuaternion.copy(quaternion);
      this.previousQuaternion.copy(quaternion);
      this.interpolatedQuaternion.copy(quaternion);
    }

    this.angularVelocity = Vec3();

    if (angularVelocity) {
      this.angularVelocity.copy(angularVelocity);
    }

    this.initAngularVelocity = Vec3();

    this.shapes = [];
    this.shapeOffsets = [];
    this.shapeOrientations = [];

    this.inertia = Vec3();
    this.invInertia = Vec3();
    this.invInertiaWorld = Mat3();
    this.invMassSolve = 0;
    this.invInertiaSolve = Vec3();
    this.invInertiaWorldSolve = Mat3();

    this.fixedRotation =  fixedRotation != null ? fixedRotation : false
    this.angularDamping =  angularDamping != null ? angularDamping : 0.01

    this.linearFactor = Vec3(1, 1, 1);

    if (linearFactor) {
      this.linearFactor.copy(linearFactor);
    }

    this.angularFactor = Vec3(1, 1, 1);

    if (angularFactor) {
      this.angularFactor.copy(angularFactor);
    }

    this.aabb = AABB();
    this.aabbNeedsUpdate = true;
    this.boundingRadius = 0;
    this.wlambda = Vec3();
    this.isTrigger = Boolean(isTrigger);

    if (shape) {
      this.addShape(shape);
    }

    this.updateMassProperties();
  }

  /**
   * Wake the body up.
   */
  void wakeUp(){
    final prevState = this.sleepState;
    this.sleepState = Body.awake;
    this.wakeUpAfterNarrowphase = false;
    if (prevState == Body.sleeping) {
      this.dispatchEvent(Body.wakeupEvent);
    }
  }

  /**
   * Force body sleep
   */
  void sleep(){
    this.sleepState = Body.sleeping;
    this.velocity.set(0, 0, 0);
    this.angularVelocity.set(0, 0, 0);
    this.wakeUpAfterNarrowphase = false;
  }

  /**
   * Called every timestep to update internal sleep timer and change sleep state if needed.
   * @param time The world time in seconds;
   */
  void sleepTick(num? time){
    if (this.allowSleep) {
      final sleepState = this.sleepState;
      final speedSquared = this.velocity.lengthSquared() + this.angularVelocity.lengthSquared();
      final speedLimitSquared = this.sleepSpeedLimit ** 2;
      if (sleepState == Body.awake && speedSquared < speedLimitSquared) {
        this.sleepState = Body.sleepy; // Sleepy
        this.timeLastSleepy = time;
        this.dispatchEvent(Body.sleepyEvent);
      } else if (sleepState == Body.sleepy && speedSquared > speedLimitSquared) {
        this.wakeUp(); // Wake up
      } else if (sleepState == Body.sleepy && time - this.timeLastSleepy > this.sleepTimeLimit) {
        this.sleep(); // Sleeping
        this.dispatchEvent(Body.sleepEvent);
      }
    }
  }

  /**
   * If the body is sleeping, it should be immovable / have infinite mass during solve. We solve it by having a separate "solve mass".
   */
void   updateSolveMassProperties(? ){
    if (this.sleepState == Body.sleeping || this.type == Body.KINEMATIC) {
      this.invMassSolve = 0;
      this.invInertiaSolve.setZero();
      this.invInertiaWorldSolve.setZero();
    } else {
      this.invMassSolve = this.invMass;
      this.invInertiaSolve.copy(this.invInertia);
      this.invInertiaWorldSolve.copy(this.invInertiaWorld);
    }
  }

  /**
   * Convert a world point to local body frame.
   */
  Vec3 pointToLocalFrame(){
    worldPoint.vsub(this.position, result);
    this.quaternion.conjugate().vmult(result, result);
    return result;
  }

  /**
   * Convert a world vector to local body frame.
   */
Vec3   vectorToLocalFrame(){
    this.quaternion.conjugate().vmult(worldVector, result);
    return result;
  }

  /**
   * Convert a local body point to world frame.
   */
  Vec3 pointToWorldFrame(){
    this.quaternion.vmult(localPoint, result);
    result.vadd(this.position, result);
    return result;
  }

  /**
   * Convert a local body point to world frame.
   */
  Vec3 vectorToWorldFrame(){
    this.quaternion.vmult(localVector, result);
    return result;
  }

  /**
   * Add a shape to the body with a local offset and orientation.
   * @return The body object, for chainability.
   */
  Body addShape(Shape? shape,Vec3? _offset,Quaternion? _orientation){
    final offset = Vec3();
    final orientation = Quaternion();

    if (_offset) {
      offset.copy(_offset);
    }
    if (_orientation) {
      orientation.copy(_orientation);
    }

    this.shapes.push(shape);
    this.shapeOffsets.push(offset);
    this.shapeOrientations.push(orientation);
    this.updateMassProperties();
    this.updateBoundingRadius();

    this.aabbNeedsUpdate = true;

    shape.body = this;

    return this;
  }

  /**
   * Remove a shape from the body.
   * @return The body object, for chainability.
   */
  Body removeShape(Shape? shape){
    final index = this.shapes.indexOf(shape);

    if (index == -1) {
      print('Shape does not belong to the body');
      return this;
    }

    this.shapes.splice(index, 1);
    this.shapeOffsets.splice(index, 1);
    this.shapeOrientations.splice(index, 1);
    this.updateMassProperties();
    this.updateBoundingRadius();

    this.aabbNeedsUpdate = true;

    shape.body = null;

    return this;
  }

  /**
   * Update the bounding radius of the body. Should be done if any of the shapes are changed.
   */
  void updateBoundingRadius(? ){
    final shapes = this.shapes;
    final shapeOffsets = this.shapeOffsets;
    final N = shapes.length;
    int radius = 0;

    for (int i = 0; i != N; i++) {
      final shape = shapes[i];
      shape.updateBoundingSphereRadius();
      final offset = shapeOffsets[i].length();
      final r = shape.boundingSphereRadius;
      if (offset + r > radius) {
        radius = offset + r;
      }
    }

    this.boundingRadius = radius;
  }

  /**
   * Updates the .aabb
   */
  void updateAABB(){
    final shapes = this.shapes;
    final shapeOffsets = this.shapeOffsets;
    final shapeOrientations = this.shapeOrientations;
    final N = shapes.length;
    final offset = tmpVec;
    final orientation = tmpQuat;
    final bodyQuat = this.quaternion;
    final aabb = this.aabb;
    final shapeAABB = updateAABB_shapeAABB;

    for (int i = 0; i != N; i++) {
      final shape = shapes[i];

      // Get shape world position
      bodyQuat.vmult(shapeOffsets[i], offset);
      offset.vadd(this.position, offset);

      // Get shape world quaternion
      bodyQuat.mult(shapeOrientations[i], orientation);

      // Get shape AABB
      shape.calculateWorldAABB(offset, orientation, shapeAABB.lowerBound, shapeAABB.upperBound);

      if (i == 0) {
        aabb.copy(shapeAABB);
      } else {
        aabb.extend(shapeAABB);
      }
    }

    this.aabbNeedsUpdate = false;
  }

  /**
   * Update `.inertiaWorld` and `.invInertiaWorld`
   */
  void updateInertiaWorld([bool force = false]) {
    final I = this.invInertia;
    if (I.x == I.y && I.y == I.z && !force) {
      // If inertia M = s*I, where I is identity and s a scalar, then
      //    R*M*R' = R*(s*I)*R' = s*R*I*R' = s*R*R' = s*I = M
      // where R is the rotation matrix.
      // In other words, we don't have to transform the inertia if all
      // inertia diagonal entries are equal.
    } else {
      final m1 = uiw_m1;
      final m2 = uiw_m2;
      final m3 = uiw_m3;
      m1.setRotationFromQuaternion(this.quaternion);
      m1.transpose(m2);
      m1.scale(I, m1);
      m1.mmult(m2, this.invInertiaWorld);
    }
  }

  /**
   * Apply force to a point of the body. This could for example be a point on the Body surface.
   * Applying force this way will add to Body.force and Body.torque.
   * @param force The amount of force to add.
   * @param relativePoint A point relative to the center of mass to apply the force on.
   */
  void applyForce(Vec3 force, [Vec3? relativePoint]) {
    relativePoint ??= Vec3();
    // Needed?
    if (this.type != Body.dynamic) {
      return;
    }

    if (this.sleepState == Body.sleeping) {
      this.wakeUp();
    }

    // Compute produced rotational force
    final rotForce = Body_applyForce_rotForce;
    relativePoint.cross(force, rotForce);

    // Add linear force
    this.force.vadd(force, this.force);

    // Add rotational force
    this.torque.vadd(rotForce, this.torque);
  }

  /**
   * Apply force to a local point in the body.
   * @param force The force vector to apply, defined locally in the body frame.
   * @param localPoint A local point in the body to apply the force on.
   */
  void applyLocalForce(){
    if (this.type != Body.dynamic) {
      return;
    }

    final worldForce = Body_applyLocalForce_worldForce;
    final relativePointWorld = Body_applyLocalForce_relativePointWorld;

    // Transform the force vector to world space
    this.vectorToWorldFrame(localForce, worldForce);
    this.vectorToWorldFrame(localPoint, relativePointWorld);

    this.applyForce(worldForce, relativePointWorld);
  }

  /**
   * Apply torque to the body.
   * @param torque The amount of torque to add.;
   */
  void applyTorque(Vec3 torque){
    if (this.type != Body.dynamic) {
      return;
    }

    if (this.sleepState == Body.sleeping) {
      this.wakeUp();
    }

    // Add rotational force
    this.torque.vadd(torque, this.torque);
  }

  /**
   * Apply impulse to a point of the body. This could for example be a point on the Body surface.
   * An impulse is a force added to a body during a short period of time (impulse = force * time).
   * Impulses will be added to Body.velocity and Body.angularVelocity.;
   * @param impulse The amount of impulse to add.;
   * @param relativePoint A point relative to the center of mass to apply the force on.
   */
  void applyImpulse(){
    if (this.type != Body.dynamic) {
      return;
    }

    if (this.sleepState == Body.sleeping) {
      this.wakeUp();
    }

    // Compute point position relative to the body center
    final Vec3 r = relativePoint;

    // Compute produced central impulse velocity
    final velo = Body_applyImpulse_velo;
    velo.copy(impulse);
    velo.scale(this.invMass, velo);

    // Add linear impulse
    this.velocity.vadd(velo, this.velocity);

    // Compute produced rotational impulse velocity
    final rotVelo = Body_applyImpulse_rotVelo;
    r.cross(impulse, rotVelo);

    /*
     rotVelo.x *= this.invInertia.x;
     rotVelo.y *= this.invInertia.y;
     rotVelo.z *= this.invInertia.z;
     */
    this.invInertiaWorld.vmult(rotVelo, rotVelo);

    // Add rotational Impulse
    this.angularVelocity.vadd(rotVelo, this.angularVelocity);
  }

  /**
   * Apply locally-defined impulse to a local point in the body.
   * @param force The force vector to apply, defined locally in the body frame.
   * @param localPoint A local point in the body to apply the force on.
   */
  void applyLocalImpulse(){
    if (this.type != Body.dynamic) {
      return;
    }

    final worldImpulse = Body_applyLocalImpulse_worldImpulse;
    final relativePointWorld = Body_applyLocalImpulse_relativePoint;

    // Transform the force vector to world space
    this.vectorToWorldFrame(localImpulse, worldImpulse);
    this.vectorToWorldFrame(localPoint, relativePointWorld);

    this.applyImpulse(worldImpulse, relativePointWorld);
  }

  /**
   * Should be called whenever you change the body shape or mass.
   */
  void updateMassProperties(){
    final halfExtents = Body_updateMassProperties_halfExtents;

    this.invMass = this.mass > 0 ? 1.0 / this.mass : 0
    final I = this.inertia;
    final fixed = this.fixedRotation;

    // Approximate with AABB box
    this.updateAABB();
    halfExtents.set(
      (this.aabb.upperBound.x - this.aabb.lowerBound.x) / 2,
      (this.aabb.upperBound.y - this.aabb.lowerBound.y) / 2,
      (this.aabb.upperBound.z - this.aabb.lowerBound.z) / 2
    );
    Box.calculateInertia(halfExtents, this.mass, I);

    this.invInertia.set(
      I.x > 0 && !fixed ? 1.0 / I.x : 0,
      I.y > 0 && !fixed ? 1.0 / I.y : 0,
      I.z > 0 && !fixed ? 1.0 / I.z : 0
    );
    this.updateInertiaWorld(true);
  }

  /**
   * Get world velocity of a point in the body.
   * @param worldPoint;
   * @param result;
   * @return The result vector.;
   */
  Vec3 getVelocityAtWorldPoint(Vec3? worldPoint, Vec3? result){
    final Vec3 r = Vec3();
    worldPoint.vsub(this.position, r);
    this.angularVelocity.cross(r, result);
    this.velocity.vadd(result, result);
    return result;
  }

  /**
   * Move the body forward in time.
   * @param dt Time step;
   * @param quatNormalize Set to true to normalize the body quaternion;
   * @param quatNormalizeFast If the quaternion should be normalized using "fast" quaternion normalization;
   */
  void integrate(double dt,[bool quatNormalize = false,bool quatNormalizeFast = false]){
    // Save previous position
    this.previousPosition.copy(this.position);
    this.previousQuaternion.copy(this.quaternion);

    if (!(this.type == Body.dynamic || this.type == Body.KINEMATIC) || this.sleepState == Body.sleeping) {
      // Only for dynamic
      return;
    }

    final velo = this.velocity;
    final angularVelo = this.angularVelocity;
    final pos = this.position;
    final force = this.force;
    final torque = this.torque;
    final quat = this.quaternion;
    final invMass = this.invMass;
    final invInertia = this.invInertiaWorld;
    final linearFactor = this.linearFactor;

    final iMdt = invMass * dt;
    velo.x += force.x * iMdt * linearFactor.x;
    velo.y += force.y * iMdt * linearFactor.y;
    velo.z += force.z * iMdt * linearFactor.z;

    final e = invInertia.elements;
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

    quat.integrate(this.angularVelocity, dt, this.angularFactor, quat);

    if (quatNormalize) {
      if (quatNormalizeFast) {
        quat.normalizeFast();
      } else {
        quat.normalize();
      }
    }

    this.aabbNeedsUpdate = true;

    // Update world inertia
    this.updateInertiaWorld();
  }
}

final tmpVec = Vec3();
final tmpQuat = Quaternion();

final updateAABB_shapeAABB = AABB();

final uiw_m1 = Mat3();
final uiw_m2 = Mat3();
final uiw_m3 = Mat3();

final Body_applyForce_rotForce = Vec3();

final Body_applyLocalForce_worldForce = Vec3();
final Body_applyLocalForce_relativePointWorld = Vec3();

final Body_applyImpulse_velo = Vec3();
final Body_applyImpulse_rotVelo = Vec3();

final Body_applyLocalImpulse_worldImpulse = Vec3();
final Body_applyLocalImpulse_relativePoint = Vec3();

final Body_updateMassProperties_halfExtents = Vec3();
