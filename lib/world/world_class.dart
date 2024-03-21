import 'dart:math' as math;
import 'package:cannon_physics/utils/logger.dart';

import '../utils/event_target.dart';
import '../utils/utils.dart';
import '../solver/gs_solver.dart';
import '../collision/naive_broadphase.dart';
import '../world/narrow_phase.dart';
import '../math/vec3.dart';
import '../material/material.dart';
import '../material/contact_material.dart';
import '../collision/array_collision_matrix.dart';
import '../collision/overlap_keeper.dart';
import '../utils/tuple_dictionary.dart';
import '../collision/raycast_result.dart';
import '../collision/ray_class.dart';
// import '../collision/aabb.dart';
import '../objects/body.dart';
import '../collision/broadphase.dart';
import '../solver/solver.dart';
import '../equations/contact_equation.dart';
import '../equations/friction_equation.dart';
import '../constraints/constraint_class.dart';
import '../shapes/shape.dart';

class Profile{
  Profile({
    this.solve = 0,
    this.makeContactConstraints = 0,
    this.broadphase = 0,
    this.integrate = 0,
    this.narrowphase = 0
  });

  int solve;
  int makeContactConstraints;
  int broadphase;
  int integrate;
  int narrowphase;
}

/// The physics world
class World extends EventTarget {
  /// Currently / last used timestep. Is set to -1 if not available. This value is updated before each internal step, which means that it is "fresh" inside event callbacks.
  double dt = -1;

  /// Makes bodies go to sleep when they've been inactive.
  late bool allowSleep;

  /// All the current contacts (instances of ContactEquation) in the world.
  List<ContactEquation> contacts = [];

  List<FrictionEquation> frictionEquations = [];

  /// How often to normalize quaternions. Set to 0 for every step, 1 for every second etc.. A larger value increases performance. If bodies tend to explode, set to a smaller value (zero to be sure nothing can go wrong).
  late int quatNormalizeSkip;

  /// Set to true to use fast quaternion normalization. It is often enough accurate to use.
  /// If bodies tend to explode, set to false.
  late bool quatNormalizeFast;

  /// The wall-clock time since simulation start.
  double time = 0.0;

  /// Number of timesteps taken since start.
  int stepnumber = 0;

  /// Default and last timestep sizes.
  double defaultDt = 1/60;

  int nextId = 0;

  /// The gravity of the world.
  Vec3 gravity = Vec3();

  /// Gravity to use when approximating the friction max force (mu*mass*gravity).
  /// If undefined, global gravity will be used.
  /// Use to enable friction in a World with a null gravity vector (no gravity).
  Vec3? frictionGravity;

  /// The broadphase algorithm to use.
  late Broadphase broadphase;

  /// All bodies in this world
  List<Body> bodies = [];

  /// True if any bodies are not sleeping, false if every body is sleeping.
  bool hasActiveBodies = false;

  /// The solver algorithm to use.
  late Solver solver;
  List<Constraint> constraints = [];
  late Narrowphase narrowphase;

  ArrayCollisionMatrix collisionMatrix = ArrayCollisionMatrix();

  /// CollisionMatrix from the previous step.
  ArrayCollisionMatrix collisionMatrixPrevious = ArrayCollisionMatrix();
  OverlapKeeper bodyOverlapKeeper = OverlapKeeper();
  OverlapKeeper shapeOverlapKeeper = OverlapKeeper();

  /// All added contactmaterials.
  List<ContactMaterial> contactmaterials = [];

  /// Used to look up a ContactMaterial given two instances of Material.
  TupleDictionary contactMaterialTable = TupleDictionary();
  /// The default material of the bodies.
  Material defaultMaterial = Material(name:'default');

  /// This contact material is used if no suitable contactmaterial is found for a contact.
  late ContactMaterial defaultContactMaterial;
  bool doProfiling = false;
  
  Profile profile = Profile();

  /// Time accumulator for interpolation.
  /// @see https://gafferongames.com/game-physics/fix-your-timestep/
  double accumulator = 0;

  List subsystems = [];
  final List<int> additions = [];
  final List<int> removals = [];

  /// Dispatched after a body has been added to the world.
  BodyEvent addBodyEvent = BodyEvent(type: 'addBody');

  /// Dispatched after a body has been removed from the world.
  BodyEvent removeBodyEvent = BodyEvent(type: 'removeBody');
  Map<int,Body> idToBodyMap = {};
  double? lastCallTime;

  bool verbose;

  World({
    Vec3? gravity,
    Vec3? frictionGravity,
    this.allowSleep = false,
    Broadphase? broadphase,
    Solver? solver,
    this.quatNormalizeFast = false,
    this.quatNormalizeSkip = 0,
    this.verbose = false
  }):super(){
    logger = NRFLogger(verbose);
    if (gravity != null) {
      this.gravity.copy(gravity);
    }
    if (frictionGravity != null) {
      this.frictionGravity = Vec3();
      this.frictionGravity!.copy(frictionGravity);
    }
    this.broadphase = broadphase ?? NaiveBroadphase();
    this.solver = solver ?? GSSolver();
    defaultContactMaterial = ContactMaterial(defaultMaterial, defaultMaterial,
      friction: 0.3,
      restitution: 0.0,
    );
    narrowphase = Narrowphase(this);
    performance.init();
    this.broadphase.setWorld(this);
  }

  // Temp stuff
  // final _tmpAABB1 = AABB();
  // final _tmpArray1 = [];
  final _tmpRay = Ray();

  // performance.now() fallback on Date.now()
  final Performance performance = Performance();

  // final Vec3 _stepTmp1 = Vec3();

  // Dispatched after the world has stepped forward in time.
  // Reusable event objects to save memory.
  final CollideEvent _worldStepPostStepEvent = CollideEvent( type: 'postStep' );

  // Dispatched before the world steps forward in time.
  final CollideEvent _worldStepPreStepEvent = CollideEvent( type: 'preStep' );

  final CollideEvent _worldStepCollideEvent = CollideEvent();

  // Pools for unused objects
  List<ContactEquation> worldStepOldContacts = [];
  List<FrictionEquation> worldStepFrictionEquationPool = [];

  // Reusable arrays for collision pairs
  List<Body> worldStepP1 = [];
  List<Body> worldStepP2 = [];

  // Stuff for emitContactEvents
  final ContactEvent beginContactEvent = ContactEvent(
    type: 'beginContact',
  );
  final ContactEvent endContactEvent  = ContactEvent(
    type: 'endContact',
  );

  final ShapeContactEvent beginShapeContactEvent  = ShapeContactEvent(
    type: 'beginShapeContact',
  );
  final ShapeContactEvent endShapeContactEvent  = ShapeContactEvent(
    type: 'endShapeContact',
  );

  /// Get the contact material between materials m1 and m2
  /// @return The contact material if it was found.
  ContactMaterial? getContactMaterial(Material m1, Material m2){
    return contactMaterialTable.get(m1.id, m2.id);
  }

  /// Store old collision state info
  void collisionMatrixTick() {
    //final temp = collisionMatrixPrevious;
    collisionMatrixPrevious = collisionMatrix;
    //collisionMatrix = temp;
    collisionMatrix.reset();

    bodyOverlapKeeper.tick();
    shapeOverlapKeeper.tick();
  }

  /// Add a constraint to the simulation.
  void addConstraint(Constraint c) {
    constraints.add(c);
  }

  /// Removes a constraint
  void removeConstraint(Constraint c) {
    constraints.remove(c);
  }

  /// Raycast test
  /// @deprecated Use .raycastAll, .raycastClosest or .raycastAny instead.
  void rayTest(Vec3 from, Vec3 to, dynamic result) {
    if (result is RaycastResult) {
      // Do raycastClosest
      raycastClosest(from, to, RayOptions()..skipBackfaces = true , result);
    } 
    else {
      // Do raycastAll
      raycastAll(from, to, RayOptions()..skipBackfaces = true, result);
    }
  }

  /// Ray cast against all bodies. The provided callback will be executed for each hit with a RaycastResult as single argument.
  /// @return True if any body was hit.
  bool raycastAll([Vec3? from,Vec3? to, RayOptions? options, RaycastCallback? callback]) {
    options ??= RayOptions();
    options.mode = RayMode.all;
    options.from = from;
    options.to = to;
    options.callback = callback;
    return _tmpRay.intersectWorld(this, options);
  }

  /// Ray cast, and stop at the first result. Note that the order is random - but the method is fast.
  /// @return True if any body was hit.
  bool raycastAny([Vec3? from, Vec3? to, RayOptions? options, RaycastResult? result]) {
    options ??= RayOptions();
    options.mode = RayMode.any;
    options.from = from;
    options.to = to;
    options.result = result;
    return _tmpRay.intersectWorld(this, options);
  }

  /// Ray cast, and return information of the closest hit.
  /// @return True if any body was hit.
  bool raycastClosest([Vec3? from, Vec3? to, RayOptions? options, RaycastResult? result]) {
    options ??= RayOptions();
    options.mode = RayMode.closest;
    options.from = from;
    options.to = to;
    options.result = result;
    return _tmpRay.intersectWorld(this, options);
  }

  /// Add a rigid body to the simulation.
  /// @todo If the simulation has not yet started, why recrete and copy arrays for each body? Accumulate in dynamic arrays in this case.
  /// @todo Adding an array of bodies should be possible. This would save some loops too
  void addBody(Body body) {
    if (bodies.contains(body)) {
      return;
    }
    body.index = bodies.length;
    bodies.add(body);
    body.world = this;
    body.initPosition.copy(body.position);
    body.initVelocity.copy(body.velocity);
    body.timeLastSleepy = time;
    //if (body is Body) {
      body.initAngularVelocity.copy(body.angularVelocity);
      body.initQuaternion.copy(body.quaternion);
    //}
    collisionMatrix.setNumObjects(bodies.length);
    addBodyEvent.target = body;
    idToBodyMap[body.id] = body;
    dispatchEvent(addBodyEvent);
  }

  /// Remove a rigid body from the simulation.
  void removeBody(Body body) {
    body.world = null;
    final n = this.bodies.length - 1;
    final bodies = this.bodies;
    final idx = bodies.indexOf(body);
    if (idx != -1) {
      bodies.removeAt(idx);
      // Recompute index
      for (int i = 0; i != bodies.length; i++) {
        bodies[i].index = i;
      }

      collisionMatrix.setNumObjects(n);
      removeBodyEvent.target = body;
      idToBodyMap.remove(body);//delete this.idToBodyMap[body.id];
      dispatchEvent(removeBodyEvent);
    }
  }

  Body getBodyById(int id) {
    return idToBodyMap[id]!;
  }

  /// @todo Make a faster map
  Shape? getShapeById(int id){
    final bodies = this.bodies;
    for (int i = 0; i < bodies.length; i++) {
      final shapes = bodies[i].shapes;
      for (int j = 0; j < shapes.length; j++) {
        final shape = shapes[j];
        if (shape.id == id) {
          return shape;
        }
      }
    }

    return null;
  }

  /// Adds a contact material to the World
  void addContactMaterial(ContactMaterial cmat) {
    // Add contact material
    contactmaterials.add(cmat);
    // Add current contact material to the material table
    contactMaterialTable.set(cmat.materials[0].id, cmat.materials[1].id, cmat);
  }

  /// Removes a contact material from the World.
  void removeContactMaterial(ContactMaterial cmat) {
    final idx = contactmaterials.indexOf(cmat);
    if (idx == -1) {
      return;
    }

    contactmaterials.removeAt(idx);
    contactMaterialTable.delete(cmat.materials[0].id, cmat.materials[1].id);
  }

  /// Step the simulation forward keeping track of last called time
  /// to be able to step the world at a fixed rate, independently of framerate.
  ///
  /// @param dt The fixed time step size to use (default: 1 / 60).
  /// @param maxSubSteps Maximum number of fixed steps to take per function call (default: 10).
  /// @see https://gafferongames.com/post/fix_your_timestep/
  /// @example
  ///     // Run the simulation independently of framerate every 1 / 60 ms
  ///     world.fixedStep()
  void fixedStep([double dt = 1 / 60, int maxSubSteps = 10]) {
    final time = performance.now() / 1000; // seconds
    if (lastCallTime == null) {
      step(dt, null, maxSubSteps);
    } else {
      final timeSinceLastCalled = time - lastCallTime!;
      step(dt, timeSinceLastCalled, maxSubSteps);
    }
    lastCallTime = time;
  }

  /// Step the physics world forward in time.
  ///
  /// There are two modes. The simple mode is fixed timestepping without interpolation. In this case you only use the first argument. The second case uses interpolation. In that you also provide the time since the function was last used, as well as the maximum fixed timesteps to take.
  ///
  /// @param dt The fixed time step size to use.
  /// @param timeSinceLastCalled The time elapsed since the function was last called.
  /// @param maxSubSteps Maximum number of fixed steps to take per function call (default: 10).
  /// @see https://web.archive.org/web/20180426154531/http://bulletphysics.org/mediawiki-1.5.8/index.php/Stepping_The_World#What_do_the_parameters_to_btDynamicsWorld::stepSimulation_mean.3F
  /// @example
  ///     // fixed timestepping without interpolation
  ///     world.step(1 / 60)
  void step(double dt, [double? timeSinceLastCalled, int maxSubSteps = 10]) {
    if (timeSinceLastCalled == null) {
      // Fixed, simple stepping

      internalStep(dt);

      // Increment time
      time += dt;
    } else {
      accumulator += timeSinceLastCalled;

      final t0 = performance.now();
      int substeps = 0;
      while (accumulator >= dt && substeps < maxSubSteps) {
        // Do fixed steps to catch up
        internalStep(dt);
        accumulator -= dt;
        substeps++;
        if (performance.now() - t0 > dt * 1000) {
          // The framerate is not interactive anymore.
          // We are below the target framerate.
          // Better bail out.
          break;
        }
      }

      // Remove the excess accumulator, since we may not
      // have had enough substeps available to catch up
      accumulator = accumulator % dt;

      final t = accumulator / dt;
      for (int j = 0; j != bodies.length; j++) {
        final b = bodies[j];
        b.previousPosition.lerp(b.position, t, b.interpolatedPosition);
        b.previousQuaternion.slerp(b.quaternion, t, b.interpolatedQuaternion);
        b.previousQuaternion.normalize();
      }
      time += timeSinceLastCalled;
    }
  }

  void internalStep(double dt) {
    this.dt = dt;

    //final world = this;
    //final that = this;
    final contacts = this.contacts;
    final p1 = worldStepP1;
    final p2 = worldStepP2;
    int N = this.bodies.length;
    final bodies = this.bodies;
    final solver = this.solver;
    final gravity = this.gravity;
    final doProfiling = this.doProfiling;
    final profile = this.profile;
    int profilingStart = -1;
    final constraints = this.constraints;
    final frictionEquationPool = worldStepFrictionEquationPool;
    //final gnorm = gravity.length();
    final gx = gravity.x;
    final gy = gravity.y;
    final gz = gravity.z;
    int i = 0;

    if (doProfiling) {
      profilingStart = performance.now();
    }
    // Add gravity to all objects
    for (i = 0; i != N; i++) {
      final bi = bodies[i];
      if (bi.type == BodyTypes.dynamic) {
        // Only for dynamic bodies
        final f = bi.force;

        final m = bi.mass;
        f.x += m * gx;
        f.y += m * gy;
        f.z += m * gz;
      }
    }
    // Update subsystems
    for (int i = 0, nSubsystems = subsystems.length; i != nSubsystems; i++) {
      subsystems[i].update();
    }

    // Collision detection
    if (doProfiling) {
      profilingStart = performance.now();
    }
    p1.clear(); // Clean up pair arrays from last step
    p2.clear();
    broadphase.collisionPairs(this, p1, p2);
    if (doProfiling) {
      profile.broadphase = performance.now() - profilingStart;
    }
    // Remove constrained pairs with collideConnected == false
    int nConstraints = constraints.length;
    for (i = 0; i != nConstraints; i++) {
      final c = constraints[i];
      if (!c.collideConnected) {
        for (int j = p1.length - 1; j >= 0; j -= 1) {
          if ((c.bodyA == p1[j] && c.bodyB == p2[j]) || (c.bodyB == p1[j] && c.bodyA == p2[j])) {
            p1.removeAt(j);
            p2.removeAt(j);
          }
        }
      }
    }
    collisionMatrixTick();

    // Generate contacts
    if (doProfiling) {
      profilingStart = performance.now();
    }
    final oldcontacts = worldStepOldContacts;
    final nOldContacts = contacts.length;

    for (i = 0; i != nOldContacts; i++) {
      oldcontacts.add(contacts[i]);
    }
    contacts.clear();

    // Transfer FrictionEquation from current list to the pool for reuse
    final nOldFrictionEquations = frictionEquations.length;
    for (i = 0; i != nOldFrictionEquations; i++) {
      frictionEquationPool.add(frictionEquations[i]);
    }
    frictionEquations.clear();
    narrowphase.getContacts(
      p1,
      p2,
      this,
      contacts,
      oldcontacts, // To be reused
      frictionEquations,
      frictionEquationPool
    );

    if (doProfiling) {
      profile.narrowphase = performance.now() - profilingStart;
    }

    // Loop over all collisions
    if (doProfiling) {
      profilingStart = performance.now();
    }
    // Add all friction eqs
    for (i = 0; i < frictionEquations.length; i++) {
      solver.addEquation(frictionEquations[i]);
    }

    final ncontacts = contacts.length;
    for (int k = 0; k != ncontacts; k++) {
      // Current contact
      final c = contacts[k];

      // Get current collision indeces
      final bi = c.bi;

      final bj = c.bj;
      final si = c.si;
      final sj = c.sj;

      // If friction or restitution were specified in the material, use them
      if (bi.material != null && bj.material != null) {
        if (bi.material!.restitution >= 0 && bj.material!.restitution >= 0) {
          c.restitution = bi.material!.restitution * bj.material!.restitution;
        }
      }

      solver.addEquation(c);

      if (
        bi.allowSleep &&
        bi.type == BodyTypes.dynamic &&
        bi.sleepState == BodySleepStates.sleeping &&
        bj.sleepState == BodySleepStates.awake &&
        bj.type != BodyTypes.static
      ) {
        final speedSquaredB = bj.velocity.lengthSquared() + bj.angularVelocity.lengthSquared();
        final speedLimitSquaredB = math.pow(bj.sleepSpeedLimit,2);
        if (speedSquaredB >= speedLimitSquaredB * 2) {
          bi.wakeUpAfterNarrowphase = true;
        }
      }

      if (
        bj.allowSleep &&
        bj.type == BodyTypes.dynamic &&
        bj.sleepState == BodySleepStates.sleeping &&
        bi.sleepState == BodySleepStates.awake &&
        bi.type != BodyTypes.static
      ) {
        final speedSquaredA = bi.velocity.lengthSquared() + bi.angularVelocity.lengthSquared();
        final speedLimitSquaredA = math.pow(bi.sleepSpeedLimit,2);
        if (speedSquaredA >= speedLimitSquaredA * 2) {
          bj.wakeUpAfterNarrowphase = true;
        }
      }

      // Now we know that i and j are in contact. Set collision matrix state
      collisionMatrix.set(bi, bj, true);

      if (collisionMatrixPrevious.get(bi, bj) != 0) {
        // First contact!
        // We reuse the collideEvent object, otherwise we will end up creating objects for each contact, even if there's no event listener attached.
        _worldStepCollideEvent.body = bj;
        _worldStepCollideEvent.contact = c;
        bi.dispatchEvent(_worldStepCollideEvent);

        _worldStepCollideEvent.body = bi;
        bj.dispatchEvent(_worldStepCollideEvent);
      }

      bodyOverlapKeeper.set(bi.id, bj.id);
      shapeOverlapKeeper.set(si.id, sj.id);
    }

    emitContactEvents();

    if (doProfiling) {
      profile.makeContactConstraints = performance.now() - profilingStart;
      profilingStart = performance.now();
    }

    // Wake up bodies
    for (i = 0; i != N; i++) {
      final bi = bodies[i];
      if (bi.wakeUpAfterNarrowphase) {
        bi.wakeUp();
        bi.wakeUpAfterNarrowphase = false;
      }
    }

    // Add user-added constraints
    nConstraints = constraints.length;
    for (i = 0; i != nConstraints; i++) {
      final c = constraints[i];
      c.update();
      for (int j = 0, neq = c.equations.length; j != neq; j++) {
        final eq = c.equations[j];
        solver.addEquation(eq);
      }
    }

    // Solve the constrained system
    solver.solve(dt, this);

    if (doProfiling) {
      profile.solve = performance.now() - profilingStart;
    }

    // Remove all contacts from solver
    solver.removeAllEquations();

    // Apply damping, see http://code.google.com/p/bullet/issues/detail?id=74 for details
    for (i = 0; i != N; i++) {
      final bi = bodies[i];
      if (bi.type == BodyTypes.dynamic) {
        // Only for dynamic bodies
        final ld = math.pow(1.0 - bi.linearDamping, dt).toDouble();
        final v = bi.velocity;
        v.scale(ld, v);
        final av = bi.angularVelocity;
        final ad = math.pow(1.0 - bi.angularDamping, dt).toDouble();
        av.scale(ad, av);
      }
    }

   dispatchEvent(_worldStepPreStepEvent);

    // Leap frog
    if (doProfiling) {
      profilingStart = performance.now();
    }
    final stepnumber = this.stepnumber;
    final quatNormalize = stepnumber % (quatNormalizeSkip + 1) == 0;
    final quatNormalizeFast = this.quatNormalizeFast;

    for (i = 0; i != N; i++) {
      bodies[i].integrate(dt, quatNormalize, quatNormalizeFast);
    }
    clearForces();

    broadphase.dirty = true;

    if (doProfiling) {
      profile.integrate = performance.now() - profilingStart;
    }

    // Update step number
    this.stepnumber += 1;

    dispatchEvent(_worldStepPostStepEvent);

    // Sleeping update
    bool hasActiveBodies = true;
    if (allowSleep) {
      hasActiveBodies = false;
      for (i = 0; i != N; i++) {
        final bi = bodies[i];
        bi.sleepTick(time);

        if (bi.sleepState != BodySleepStates.sleeping) {
          hasActiveBodies = true;
        }
      }
    }
    this.hasActiveBodies = hasActiveBodies;
  }

  void emitContactEvents() {
    final hasBeginContact = hasAnyEventListener('beginContact');
    final hasEndContact = hasAnyEventListener('endContact');
    if (hasBeginContact || hasEndContact) {
      bodyOverlapKeeper.getDiff(additions, removals);
    }

    if (hasBeginContact) {
      for (int i = 0, l = additions.length; i < l; i += 2) {
        beginContactEvent.bodyA = getBodyById(additions[i]);
        beginContactEvent.bodyB = getBodyById(additions[i + 1]);
        dispatchEvent(beginContactEvent);
      }
      beginContactEvent.bodyA = beginContactEvent.bodyB = null;
    }

    if (hasEndContact) {
      for (int i = 0, l = removals.length; i < l; i += 2) {
        endContactEvent.bodyA = getBodyById(removals[i]);
        endContactEvent.bodyB = getBodyById(removals[i + 1]);
        dispatchEvent(endContactEvent);
      }
      endContactEvent.bodyA = endContactEvent.bodyB = null;
    }

    additions.clear(); 
    removals.clear();

    final hasBeginShapeContact = hasAnyEventListener('beginShapeContact');
    final hasEndShapeContact = hasAnyEventListener('endShapeContact');
    if (hasBeginShapeContact || hasEndShapeContact) {
      shapeOverlapKeeper.getDiff(additions, removals);
    }

    if (hasBeginShapeContact) {
      for (int i = 0, l = additions.length; i < l; i += 2) {
        final shapeA = getShapeById(additions[i]);
        final shapeB = getShapeById(additions[i + 1]);
        beginShapeContactEvent.shapeA = shapeA;
        beginShapeContactEvent.shapeB = shapeB;
        if (shapeA != null) beginShapeContactEvent.bodyA = shapeA.body;
        if (shapeB != null) beginShapeContactEvent.bodyB = shapeB.body;
        dispatchEvent(beginShapeContactEvent);
      }
      beginShapeContactEvent.bodyA =
        beginShapeContactEvent.bodyB =
        beginShapeContactEvent.shapeA =
        beginShapeContactEvent.shapeB =
          null;
    }

    if (hasEndShapeContact) {
      for (int i = 0, l = removals.length; i < l; i += 2) {
        final shapeA = getShapeById(removals[i]);
        final shapeB = getShapeById(removals[i + 1]);
        endShapeContactEvent.shapeA = shapeA;
        endShapeContactEvent.shapeB = shapeB;
        if (shapeA != null) endShapeContactEvent.bodyA = shapeA.body;
        if (shapeB != null) endShapeContactEvent.bodyB = shapeB.body;
        dispatchEvent(endShapeContactEvent);
      }
      endShapeContactEvent.bodyA =
        endShapeContactEvent.bodyB =
        endShapeContactEvent.shapeA =
        endShapeContactEvent.shapeB =
          null;
    }
  }

  /// Sets all body forces in the world to zero.
  void clearForces() {
    final bodies = this.bodies;
    final N = bodies.length;
    for (int i = 0; i != N; i++) {
      final b = bodies[i];
      b.force.set(0, 0, 0);
      b.torque.set(0, 0, 0);
    }
  }
}