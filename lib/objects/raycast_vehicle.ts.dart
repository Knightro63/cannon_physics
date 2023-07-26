import '../objects/body.dart';
import '../math/vec3.dart';
import '../math/quaternion.dart';
import '../collision/ray.dart';
import '../objects/wheelinfo.dart';
import '../objects/wheelinfo.dart';
import '../math/transform.dart';
import '../finalraints/constraint.dart';
import '../world/world.dart';
import 'dart:math' as math;

//export type RaycastVehicleOptions = ConstructorParameters< RaycastVehicle>[0];

/**
 * Vehicle helper class that casts rays from the wheel positions towards the ground and applies forces.
 */
class RaycastVehicle {
  /** The car chassis body. */
Body chassisBody;
  /** The wheels. */
List<WheelInfo> wheelInfos;
  /** Will be set to true if the car is sliding. */
bool sliding;
World? world;
  /** Index of the right axis. x=0, y=1, z=2 */
num indexRightAxis;
  /** Index of the forward axis. x=0, y=1, z=2 */
num indexForwardAxis;
  /** Index of the up axis. x=0, y=1, z=2 */
num indexUpAxis;
  /** The finalraints. */
List<Constraint> finalraints;
  /** Optional pre-step callback. */
  preStepCallback: ()  void
num currentVehicleSpeedKmHour;
  /** Number of wheels on the ground. */
num numWheelsOnGround;

 RaycastVehicle ({
    /** The car chassis body. */
    Body chassisBody,
        /** Index of the right axis. x=0, y=1, z=2 */
    num? indexRightAxis,
        /** Index of the forward axis. x=0, y=1, z=2 */
    num? indexForwardAxis,
        /** Index of the up axis. x=0, y=1, z=2 */
    num? indexUpAxis,
  }) {
    this.chassisBody = chassisBody;
    this.wheelInfos = [];
    this.sliding = false;
    this.world = null;
    this.indexRightAxis =  indexRightAxis != null ? indexRightAxis : 2
    this.indexForwardAxis =  indexForwardAxis != null ? indexForwardAxis : 0
    this.indexUpAxis =  indexUpAxis != null ? indexUpAxis : 1
    this.finalraints = [];
    this.preStepCallback = ()  {}
    this.currentVehicleSpeedKmHour = 0;
    this.numWheelsOnGround = 0;
  }

  /**
   * Add a wheel. For information about the options, see `WheelInfo`.
   */
num   addWheel([WheelInfoOptions? options =  const {}]){
    final info = WheelInfo(options);
    final index = this.wheelInfos.length;
    this.wheelInfos.push(info);

    return index;
  }

  /**
   * Set the steering value of a wheel.
   */
void   setSteeringValue(num? value,num? wheelIndex){
    final wheel = this.wheelInfos[wheelIndex];
    wheel.steering = value;
  }

  /**
   * Set the wheel force to apply on one of the wheels each time step
   */
void   applyEngineForce(num? value,num? wheelIndex){
    this.wheelInfos[wheelIndex].engineForce = value;
  }

  /**
   * Set the braking force of a wheel
   */
void   setBrake(num? brake,num? wheelIndex){
    this.wheelInfos[wheelIndex].brake = brake;
  }

  /**
   * Add the vehicle including its finalraints to the world.
   */
void   addToWorld(World? world){
    world.addBody(this.chassisBody);
    final that = this;
    this.preStepCallback = ()  {
      that.updateVehicle(world.dt);
    }
    world.addEventListener('preStep', this.preStepCallback);
    this.world = world;
  }

  /**
   * Get one of the wheel axles, world-oriented.
   */
void   private getVehicleAxisWorld(num? axisIndex,Vec3? result){
    result.set(axisIndex == 0 ? 1 : 0, axisIndex == 1 ? 1 : 0, axisIndex == 2 ? 1 : 0)
    this.chassisBody.vectorToWorldFrame(result, result);
  }

void   updateVehicle(num? timeStep){
    final wheelInfos = this.wheelInfos;
    final numWheels = wheelInfos.length;
    final chassisBody = this.chassisBody;

    for (int i = 0; i < numWheels; i++) {
      this.updateWheelTransform(i);
    }

    this.currentVehicleSpeedKmHour = 3.6 * chassisBody.velocity.length();

    final forwardWorld = Vec3();
    this.getVehicleAxisWorld(this.indexForwardAxis, forwardWorld);

    if (forwardWorld.dot(chassisBody.velocity) < 0) {
      this.currentVehicleSpeedKmHour *= -1;
    }

    ; // simulate suspension
    for (int i = 0; i < numWheels; i++) {
      this.castRay(wheelInfos[i]);
    }

    this.updateSuspension(timeStep);

    final impulse = Vec3();
    final relpos = Vec3();
    for (int i = 0; i < numWheels; i++) {
      //apply suspension force
      final wheel = wheelInfos[i];
      int suspensionForce = wheel.suspensionForce;
      if (suspensionForce > wheel.maxSuspensionForce) {
        suspensionForce = wheel.maxSuspensionForce;
      }
      wheel.raycastResult.hitNormalWorld.scale(suspensionForce * timeStep, impulse);

      wheel.raycastResult.hitPointWorld.vsub(chassisBody.position, relpos);
      chassisBody.applyImpulse(impulse, relpos);
    }

    this.updateFriction(timeStep);

    final hitNormalWorldScaledWithProj = Vec3();
    final fwd = Vec3();
    final vel = Vec3();
    for (int i = 0; i < numWheels; i++) {
      final wheel = wheelInfos[i];
      //final relpos = Vec3();
      //wheel.chassisConnectionPointWorld.vsub(chassisBody.position, relpos);
      chassisBody.getVelocityAtWorldPoint(wheel.chassisConnectionPointWorld, vel);

      ; // Hack to get the rotation in the correct direction
      int m = 1;
      switch (this.indexUpAxis) {
        case 1:
          m = -1;
          break;
      }

      if (wheel.isInContact) {
        this.getVehicleAxisWorld(this.indexForwardAxis, fwd);
        final proj = fwd.dot(wheel.raycastResult.hitNormalWorld);
        wheel.raycastResult.hitNormalWorld.scale(proj, hitNormalWorldScaledWithProj);

        fwd.vsub(hitNormalWorldScaledWithProj, fwd);

        final proj2 = fwd.dot(vel);
        wheel.deltaRotation = (m * proj2 * timeStep) / wheel.radius;
      }

      if ((wheel.sliding || !wheel.isInContact) && wheel.engineForce != 0 && wheel.useCustomSlidingRotationalSpeed) {
        ; // Apply custom rotation when accelerating and sliding
        wheel.deltaRotation = (wheel.engineForce > 0 ? 1 : -1) * wheel.customSlidingRotationalSpeed * timeStep
      }

      ; // Lock wheels
      if (math.abs(wheel.brake) > math.abs(wheel.engineForce)) {
        wheel.deltaRotation = 0;
      }

      wheel.rotation += wheel.deltaRotation ; // Use the old value
      wheel.deltaRotation *= 0.99 ; // damping of rotation when not in contact
    }
  }

void   updateSuspension(num? deltaTime){
    final chassisBody = this.chassisBody;
    final chassisMass = chassisBody.mass;
    final wheelInfos = this.wheelInfos;
    final numWheels = wheelInfos.length;

    for (int w_it = 0; w_it < numWheels; w_it++) {
      final wheel = wheelInfos[w_it];

      if (wheel.isInContact) {
        int force

        ; // Spring
        final susp_length = wheel.suspensionRestLength;
        final current_length = wheel.suspensionLength;
        final length_diff = susp_length - current_length;

        force = wheel.suspensionStiffness * length_diff * wheel.clippedInvContactDotSuspension

        ; // Damper
        final projected_rel_vel = wheel.suspensionRelativeVelocity;
        int susp_damping;
        if (projected_rel_vel < 0) {
          susp_damping = wheel.dampingCompression;
        } else {
          susp_damping = wheel.dampingRelaxation;
        }
        force -= susp_damping * projected_rel_vel

        wheel.suspensionForce = force * chassisMass
        if (wheel.suspensionForce < 0) {
          wheel.suspensionForce = 0;
        }
      } else {
        wheel.suspensionForce = 0;
      }
    }
  }

  /**
   * Remove the vehicle including its finalraints from the world.
   */
void   removeFromWorld(World? world){
    final finalraints = this.finalraints;
    world.removeBody(this.chassisBody);
    world.removeEventListener('preStep', this.preStepCallback);
    this.world = null;
  }

num   castRay(WheelInfo? wheel){
    final rayvector = castRay_rayvector;
    final target = castRay_target;

    this.updateWheelTransformWorld(wheel)
    final chassisBody = this.chassisBody;

    int depth = -1;

    final raylen = wheel.suspensionRestLength + wheel.radius;

    wheel.directionWorld.scale(raylen, rayvector);
    final source = wheel.chassisConnectionPointWorld;
    source.vadd(rayvector, target);
    final raycastResult = wheel.raycastResult;

    final param = 0;

    raycastResult.reset();
    ; // Turn off ray collision with the chassis temporarily
    final oldState = chassisBody.collisionResponse;
    chassisBody.collisionResponse = false;

    ; // Cast ray against world
    this.world!.rayTest(source, target, raycastResult);
    chassisBody.collisionResponse = oldState;

    final object = raycastResult.body;

    wheel.raycastResult.groundObject = 0;

    if (object) {
      depth = raycastResult.distance;
      wheel.raycastResult.hitNormalWorld = raycastResult.hitNormalWorld;
      wheel.isInContact = true;

      final hitDistance = raycastResult.distance;
      wheel.suspensionLength = hitDistance - wheel.radius;

      ; // clamp on max suspension travel
      final minSuspensionLength = wheel.suspensionRestLength - wheel.maxSuspensionTravel;
      final maxSuspensionLength = wheel.suspensionRestLength + wheel.maxSuspensionTravel;
      if (wheel.suspensionLength < minSuspensionLength) {
        wheel.suspensionLength = minSuspensionLength;
      }
      if (wheel.suspensionLength > maxSuspensionLength) {
        wheel.suspensionLength = maxSuspensionLength;
        wheel.raycastResult.reset();
      }

      final denominator = wheel.raycastResult.hitNormalWorld.dot(wheel.directionWorld);

      final chassis_velocity_at_contactPoint = Vec3();
      chassisBody.getVelocityAtWorldPoint(wheel.raycastResult.hitPointWorld, chassis_velocity_at_contactPoint);

      final projVel = wheel.raycastResult.hitNormalWorld.dot(chassis_velocity_at_contactPoint);

      if (denominator >= -0.1) {
        wheel.suspensionRelativeVelocity = 0;
        wheel.clippedInvContactDotSuspension = 1 / 0.1;
      } else {
        final inv = -1 / denominator;
        wheel.suspensionRelativeVelocity = projVel * inv;
        wheel.clippedInvContactDotSuspension = inv;
      }
    } else {
      ; //put wheel info as in rest position
      wheel.suspensionLength = wheel.suspensionRestLength + 0 * wheel.maxSuspensionTravel;
      wheel.suspensionRelativeVelocity = 0.0;
      wheel.directionWorld.scale(-1, wheel.raycastResult.hitNormalWorld);
      wheel.clippedInvContactDotSuspension = 1.0;
    }

    return depth;
  }

  updateWheelTransformWorld(wheel: WheelInfo): void {
    wheel.isInContact = false;
    final chassisBody = this.chassisBody;
    chassisBody.pointToWorldFrame(wheel.chassisConnectionPointLocal, wheel.chassisConnectionPointWorld);
    chassisBody.vectorToWorldFrame(wheel.directionLocal, wheel.directionWorld);
    chassisBody.vectorToWorldFrame(wheel.axleLocal, wheel.axleWorld);
  }

  /**
   * Update one of the wheel transform.
   * Note when rendering wheels: during each step, wheel transforms are updated BEFORE the chassis; ie. their position becomes invalid after the step. Thus when you render wheels, you must update wheel transforms before rendering them. See raycastVehicle demo for an example.
   * @param wheelIndex The wheel index to update.;
   */
  updateWheelTransform(wheelIndex: number): void {
    final up = tmpVec4;
    final right = tmpVec5;
    final fwd = tmpVec6;

    final wheel = this.wheelInfos[wheelIndex];
    this.updateWheelTransformWorld(wheel)

    wheel.directionLocal.scale(-1, up);
    right.copy(wheel.axleLocal);
    up.cross(right, fwd);
    fwd.normalize();
    right.normalize();

    ; // Rotate around steering over the wheelAxle
    final steering = wheel.steering;
    final steeringOrn = Quaternion();
    steeringOrn.setFromAxisAngle(up, steering);

    final rotatingOrn = Quaternion();
    rotatingOrn.setFromAxisAngle(right, wheel.rotation);

    ; // World rotation of the wheel
    final q = wheel.worldTransform.quaternion
    this.chassisBody.quaternion.mult(steeringOrn, q);
    q.mult(rotatingOrn, q);

    q.normalize();

    ; // world position of the wheel
    final p = wheel.worldTransform.position
    p.copy(wheel.directionWorld);
    p.scale(wheel.suspensionLength, p);
    p.vadd(wheel.chassisConnectionPointWorld, p);
  }

  /**
   * Get the world transform of one of the wheels
   */
  getWheelTransformWorld(wheelIndex: number): Transform {
    return this.wheelInfos[wheelIndex].worldTransform
  }

void   updateFriction(num? timeStep){
    final surfNormalWS_scaled_proj = updateFriction_surfNormalWS_scaled_proj;

    ; //calculate the impulse, so that the wheels don't move sidewards
    final wheelInfos = this.wheelInfos;
    final numWheels = wheelInfos.length;
    final chassisBody = this.chassisBody;
    final forwardWS = updateFriction_forwardWS
    final axle = updateFriction_axle;

    this.numWheelsOnGround = 0;

    for (int i = 0; i < numWheels; i++) {
      final wheel = wheelInfos[i];

      final groundObject = wheel.raycastResult.body;
      if (groundObject) {
        this.numWheelsOnGround++;
      }

      wheel.sideImpulse = 0;
      wheel.forwardImpulse = 0
      if (!forwardWS[i]) {
        forwardWS[i] = Vec3()
      }
      if (!axle[i]) {
        axle[i] = Vec3();
      }
    }

    for (int i = 0; i < numWheels; i++) {
      final wheel = wheelInfos[i];

      final groundObject = wheel.raycastResult.body;

      if (groundObject) {
        final axlei = axle[i];
        final wheelTrans = this.getWheelTransformWorld(i)

        ; // Get world axle
        wheelTrans.vectorToWorldFrame(directions[this.indexRightAxis], axlei);

        final surfNormalWS = wheel.raycastResult.hitNormalWorld;
        final proj = axlei.dot(surfNormalWS);
        surfNormalWS.scale(proj, surfNormalWS_scaled_proj);
        axlei.vsub(surfNormalWS_scaled_proj, axlei);
        axlei.normalize();

        surfNormalWS.cross(axlei, forwardWS[i]);
        forwardWS[i].normalize();

        wheel.sideImpulse = resolveSingleBilateral(
          chassisBody,
          wheel.raycastResult.hitPointWorld,
          groundObject,
          wheel.raycastResult.hitPointWorld,
          axlei
        );

        wheel.sideImpulse *= sideFrictionStiffness2;
      }
    }

    final sideFactor = 1;
    final fwdFactor = 0.5;

    this.sliding = false;
    for (int i = 0; i < numWheels; i++) {
      final wheel = wheelInfos[i];
      final groundObject = wheel.raycastResult.body;

      int rollingFriction = 0;

      wheel.slipInfo = 1;
      if (groundObject) {
        final defaultRollingFrictionImpulse = 0;
        final maxImpulse = wheel.brake ? wheel.brake : defaultRollingFrictionImpulse;

        // btWheelContactPoint contactPt(chassisBody,groundObject,wheelInfraycastInfo.hitPointWorld,forwardWS[wheel],maxImpulse);
        // rollingFriction = calcRollingFriction(contactPt);
        rollingFriction = calcRollingFriction(
          chassisBody,
          groundObject,
          wheel.raycastResult.hitPointWorld,
          forwardWS[i],
          maxImpulse
        );

        rollingFriction += wheel.engineForce * timeStep;

        // rollingFriction = 0;
        final factor = maxImpulse / rollingFriction;
        wheel.slipInfo *= factor;
      }

      //switch between active rolling (throttle), braking and non-active rolling friction (nthrottle/break)

      wheel.forwardImpulse = 0
      wheel.skidInfo = 1;

      if (groundObject) {
        wheel.skidInfo = 1;

        final maximp = wheel.suspensionForce * timeStep * wheel.frictionSlip;
        final maximpSide = maximp;

        final maximpSquared = maximp * maximpSide;

        wheel.forwardImpulse = rollingFriction //wheelInfo.engineForce* timeStep;

        final x = (wheel.forwardImpulse * fwdFactor) / wheel.forwardAcceleration
        final y = (wheel.sideImpulse * sideFactor) / wheel.sideAcceleration;

        final impulseSquared = x * x + y * y;

        wheel.sliding = false;
        if (impulseSquared > maximpSquared) {
          this.sliding = true;
          wheel.sliding = true;

          final factor = maximp / math.sqrt(impulseSquared);

          wheel.skidInfo *= factor;
        }
      }
    }

    if (this.sliding) {
      for (int i = 0; i < numWheels; i++) {
        final wheel = wheelInfos[i];
        if (wheel.sideImpulse != 0) {
          if (wheel.skidInfo < 1) {
            wheel.forwardImpulse *= wheel.skidInfo
            wheel.sideImpulse *= wheel.skidInfo;
          }
        }
      }
    }

    ; // apply the impulses
    for (int i = 0; i < numWheels; i++) {
      final wheel = wheelInfos[i];

      final rel_pos = Vec3();
      wheel.raycastResult.hitPointWorld.vsub(chassisBody.position, rel_pos);
      // cannons applyimpulse is using world coord for the position
      //rel_pos.copy(wheel.raycastResult.hitPointWorld);

      if (wheel.forwardImpulse != 0) {
        final impulse = Vec3();
        forwardWS[i].scale(wheel.forwardImpulse, impulse)
        chassisBody.applyImpulse(impulse, rel_pos);
      }

      if (wheel.sideImpulse != 0) {
        final groundObject = wheel.raycastResult.body!;

        final rel_pos2 = Vec3();
        wheel.raycastResult.hitPointWorld.vsub(groundObject.position, rel_pos2);
        //rel_pos2.copy(wheel.raycastResult.hitPointWorld);
        final sideImp = Vec3();
        axle[i].scale(wheel.sideImpulse, sideImp);

        ; // Scale the relative position in the up direction with rollInfluence.
        // If rollInfluence is 1, the impulse will be applied on the hitPoint (easy to roll over), if it is zero it will be applied in the same plane as the center of mass (not easy to roll over).
        chassisBody.vectorToLocalFrame(rel_pos, rel_pos);
        rel_pos['xyz'[this.indexUpAxis] as 'x' | 'y' | 'z'] *= wheel.rollInfluence;
        chassisBody.vectorToWorldFrame(rel_pos, rel_pos);
        chassisBody.applyImpulse(sideImp, rel_pos);

        ; //apply friction impulse on the ground
        sideImp.scale(-1, sideImp);
        groundObject.applyImpulse(sideImp, rel_pos2);
      }
    }
  }
}

final tmpVec1 = Vec3();
final tmpVec2 = Vec3();
final tmpVec3 = Vec3();
final tmpVec4 = Vec3();
final tmpVec5 = Vec3();
final tmpVec6 = Vec3();
final tmpRay = Ray();

final torque = Vec3();

final castRay_rayvector = Vec3();
final castRay_target = Vec3();

final directions = [Vec3(1, 0, 0), Vec3(0, 1, 0), Vec3(0, 0, 1)];

final updateFriction_surfNormalWS_scaled_proj = Vec3();
List<Vec3> finalupdateFriction_axle =  [];
final updateFriction_forwardWS: Vec3[] = []
final sideFrictionStiffness2 = 1;

final calcRollingFriction_vel1 = Vec3();
final calcRollingFriction_vel2 = Vec3();
final calcRollingFriction_vel = Vec3();

num calcRollingFriction(
Body body0,
Body body1,
Vec3 frictionPosWorld,
Vec3 frictionDirectionWorld,
num maxImpulse,
){
  int j1 = 0;
  final contactPosWorld = frictionPosWorld;

  // final rel_pos1 = Vec3();
  // final rel_pos2 = Vec3();
  final vel1 = calcRollingFriction_vel1;
  final vel2 = calcRollingFriction_vel2;
  final vel = calcRollingFriction_vel;
  // contactPosWorld.vsub(body0.position, rel_pos1);
  // contactPosWorld.vsub(body1.position, rel_pos2);

  body0.getVelocityAtWorldPoint(contactPosWorld, vel1);
  body1.getVelocityAtWorldPoint(contactPosWorld, vel2);
  vel1.vsub(vel2, vel);

  final vrel = frictionDirectionWorld.dot(vel);

  final denom0 = computeImpulseDenominator(body0, frictionPosWorld, frictionDirectionWorld);
  final denom1 = computeImpulseDenominator(body1, frictionPosWorld, frictionDirectionWorld);
  final relaxation = 1;
  final jacDiagABInv = relaxation / (denom0 + denom1);

  ; // calculate j that moves us to zero relative velocity
  j1 = -vrel * jacDiagABInv;

  if (maxImpulse < j1) {
    j1 = maxImpulse;
  }
  if (j1 < -maxImpulse) {
    j1 = -maxImpulse;
  }

  return j1;
}

final computeImpulseDenominator_r0 = Vec3();
final computeImpulseDenominator_c0 = Vec3();
final computeImpulseDenominator_vec = Vec3();
final computeImpulseDenominator_m = Vec3();

num computeImpulseDenominator(Body? body,Vec3? pos,Vec3? normal){
  final r0 = computeImpulseDenominator_r0;
  final c0 = computeImpulseDenominator_c0;
  final vec = computeImpulseDenominator_vec;
  final m = computeImpulseDenominator_m;

  pos.vsub(body.position, r0);
  r0.cross(normal, c0);
  body.invInertiaWorld.vmult(c0, m);
  m.cross(r0, vec);

  return body.invMass + normal.dot(vec);
}

final resolveSingleBilateral_vel1 = Vec3();
final resolveSingleBilateral_vel2 = Vec3();
final resolveSingleBilateral_vel = Vec3();

// bilateral finalraint between two dynamic objects
num resolveSingleBilateral(Body? body1,Vec3? pos1,Body? body2,Vec3? pos2,Vec3? normal){
  final normalLenSqr = normal.lengthSquared();
  if (normalLenSqr > 1.1) {
    return 0 ; // no impulse
  }
  // final rel_pos1 = Vec3();
  // final rel_pos2 = Vec3();
  // pos1.vsub(body1.position, rel_pos1);
  // pos2.vsub(body2.position, rel_pos2);

  final vel1 = resolveSingleBilateral_vel1;
  final vel2 = resolveSingleBilateral_vel2;
  final vel = resolveSingleBilateral_vel;
  body1.getVelocityAtWorldPoint(pos1, vel1);
  body2.getVelocityAtWorldPoint(pos2, vel2);

  vel1.vsub(vel2, vel);

  final rel_vel = normal.dot(vel);

  final contactDamping = 0.2;
  final massTerm = 1 / (body1.invMass + body2.invMass);
  final impulse = -contactDamping * rel_vel * massTerm;

  return impulse;
}
