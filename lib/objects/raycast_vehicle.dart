import '../objects/body.dart';
import '../math/vec3.dart';
import '../math/quaternion.dart';
import '../objects/wheel_info.dart';
import '../math/transform.dart';
import '../constraints/constraint_class.dart';
import '../world/world_class.dart';
import 'dart:math' as math;
import '../utils/utils.dart';

/// Vehicle helper class that casts rays from the wheel positions towards the ground and applies forces.
class RaycastVehicle {

 RaycastVehicle ({
    required this.chassisBody,
    this.indexRightAxis = AxisIndex.z,
    this.indexForwardAxis = AxisIndex.x,
    this.indexUpAxis = AxisIndex.y,
  });

  /// The car chassis body.
  Body chassisBody;
  /// The wheels.
  List<WheelInfo> wheelInfos = [];
  /// Will be set to true if the car is sliding.
  bool sliding = false;
  World? world;
  /// Index of the right axis. x=0, y=1, z=2
  AxisIndex indexRightAxis;
  /// Index of the forward axis. x=0, y=1, z=2
  AxisIndex indexForwardAxis;
  /// Index of the up axis. x=0, y=1, z=2
  AxisIndex indexUpAxis;
  /// The finalraints. */
  List<Constraint> finalraints = [];
  /// Optional pre-step callback
  void  Function(dynamic event) preStepCallback = (event){};
  double currentVehicleSpeedKmHour = 0;
  /// Number of wheels on the ground
  int numWheelsOnGround = 0;


  //final _tmpVec1 = Vec3();
  //final _tmpVec2 = Vec3();
  //final _tmpVec3 = Vec3();
  final _tmpVec4 = Vec3();
  final _tmpVec5 = Vec3();
  final _tmpVec6 = Vec3();
  //final _tmpRay = Ray();

  final torque = Vec3();

  final _castRayRayvector = Vec3();
  final _castRayTarget = Vec3();

  List<Vec3> directions = [Vec3(1, 0, 0), Vec3(0, 1, 0), Vec3(0, 0, 1)];

  final _updateFrictionSurfNormalWSScaledProj = Vec3();
  List<Vec3> updateFrictionAxle =  [];
  List<Vec3> updateFrictionForwardWS = [];
  final _sideFrictionStiffness2 = 1;

  final _calcRollingFrictionVel1 = Vec3();
  final _calcRollingFrictionVel2 = Vec3();
  final _calcRollingFrictionVel = Vec3();

  final _computeImpulseDenominatorR0 = Vec3();
  final _computeImpulseDenominatorC0 = Vec3();
  final _computeImpulseDenominatorVec = Vec3();
  final _computeImpulseDenominatorM = Vec3();

  final _resolveSingleBilateralVel1 = Vec3();
  final _resolveSingleBilateralVel2 = Vec3();
  final _resolveSingleBilateralVel = Vec3();

  /// Add a wheel. For information about the options, see `WheelInfo`.
  int addWheel([WheelInfo? wheelInfo]){
    final info = wheelInfo?.copy() ?? WheelInfo();
    final index = wheelInfos.length;
    wheelInfos.add(info);

    return index;
  }

  /// Set the steering value of a wheel.
  void setSteeringValue(double value, int wheelIndex){
    final wheel = wheelInfos[wheelIndex];
    wheel.steering = value;
  }

  /// Set the wheel force to apply on one of the wheels each time step
  void applyEngineForce(double value,int wheelIndex){
    wheelInfos[wheelIndex].engineForce = value;
  }

  /// Set the braking force of a wheel
  void setBrake(double brake,int wheelIndex){
    wheelInfos[wheelIndex].brake = brake;
  }

  /// Add the vehicle including its finalraints to the world.
  void addToWorld(World world){
    world.addBody(chassisBody);
    final that = this;
    preStepCallback = (event){
      that.updateVehicle(world.dt);
    };
    world.addEventListener('preStep', preStepCallback);
    this.world = world;
  }

  /// Get one of the wheel axles, world-oriented.
  void getVehicleAxisWorld(AxisIndex axisIndex, Vec3 result){
    result.set(axisIndex == AxisIndex.x ? 1 : 0, AxisIndex.y == axisIndex ? 1 : 0, axisIndex == AxisIndex.z ? 1 :0);
    chassisBody.vectorToWorldFrame(result, result);
  }

  void updateVehicle(double timeStep){
    final wheelInfos = this.wheelInfos;
    final numWheels = wheelInfos.length;
    final chassisBody = this.chassisBody;

    for (int i = 0; i < numWheels; i++) {
      updateWheelTransform(i);
    }

    currentVehicleSpeedKmHour = 3.6 * chassisBody.velocity.length();

    final forwardWorld = Vec3();
    getVehicleAxisWorld(indexForwardAxis, forwardWorld);

    if (forwardWorld.dot(chassisBody.velocity) < 0) {
      currentVehicleSpeedKmHour *= -1;
    }

    // simulate suspension
    for (int i = 0; i < numWheels; i++) {
      castRay(wheelInfos[i]);
    }

    updateSuspension(timeStep);

    final impulse = Vec3();
    final relpos = Vec3();
    for (int i = 0; i < numWheels; i++) {
      //apply suspension force
      final wheel = wheelInfos[i];
      double suspensionForce = wheel.suspensionForce;
      if (suspensionForce > wheel.maxSuspensionForce) {
        suspensionForce = wheel.maxSuspensionForce;
      }
      wheel.raycastResult.hitNormalWorld.scale(suspensionForce * timeStep, impulse);

      wheel.raycastResult.hitPointWorld.vsub(chassisBody.position, relpos);
      chassisBody.applyImpulse(impulse, relpos);
    }

    updateFriction(timeStep);

    final hitNormalWorldScaledWithProj = Vec3();
    final fwd = Vec3();
    final vel = Vec3();
    for (int i = 0; i < numWheels; i++) {
      final wheel = wheelInfos[i];
      //final relpos = Vec3();
      //wheel.chassisConnectionPointWorld.vsub(chassisBody.position, relpos);
      chassisBody.getVelocityAtWorldPoint(wheel.chassisConnectionPointWorld, vel);

      // Hack to get the rotation in the correct direction
      int m = 1;
      //switch (indexUpAxis) {
      if (indexUpAxis == AxisIndex.y){
        m = -1;
        break;
      }

      if (wheel.isInContact) {
        getVehicleAxisWorld(indexForwardAxis, fwd);
        final proj = fwd.dot(wheel.raycastResult.hitNormalWorld);
        wheel.raycastResult.hitNormalWorld.scale(proj, hitNormalWorldScaledWithProj);

        fwd.vsub(hitNormalWorldScaledWithProj, fwd);

        final proj2 = fwd.dot(vel);
        wheel.deltaRotation = (m * proj2 * timeStep) / wheel.radius;
      }

      if ((wheel.sliding || !wheel.isInContact) && wheel.engineForce != 0 
      && wheel.useCustomSlidingRotationalSpeed) {
        // Apply custom rotation when accelerating and sliding
        wheel.deltaRotation = (wheel.engineForce > 0 ? 1 : -1) * wheel.customSlidingRotationalSpeed * timeStep;
      }

      // Lock wheels
      if (wheel.brake.abs() > wheel.engineForce.abs()) {
        wheel.deltaRotation = 0;
      }

      wheel.rotation += wheel.deltaRotation ; // Use the old value
      wheel.deltaRotation *= 0.99 ; // damping of rotation when not in contact
    }
  }

  void updateSuspension(num? deltaTime){
    final chassisBody = this.chassisBody;
    final chassisMass = chassisBody.mass;
    final wheelInfos = this.wheelInfos;
    final numWheels = wheelInfos.length;

    for (int wIt = 0; wIt < numWheels; wIt++) {
      final wheel = wheelInfos[wIt];

      if (wheel.isInContact) {
        double force; 
        
        // Spring
        final suspLength = wheel.suspensionRestLength;
        final currentLength = wheel.suspensionLength;
        final lengthDiff = suspLength - currentLength;

        force = wheel.suspensionStiffness * lengthDiff * wheel.clippedInvContactDotSuspension;

        // Damper
        final projectedRelVel = wheel.suspensionRelativeVelocity;
        double suspDamping;
        if (projectedRelVel < 0) {
          suspDamping = wheel.dampingCompression;
        } else {
          suspDamping = wheel.dampingRelaxation;
        }
        force -= suspDamping * projectedRelVel;

        wheel.suspensionForce = force * chassisMass;
        if (wheel.suspensionForce < 0) {
          wheel.suspensionForce = 0;
        }
      } else {
        wheel.suspensionForce = 0;
      }
    }
  }

  /// Remove the vehicle including its finalraints from the world.
  void removeFromWorld(World world){
    //final finalraints = this.finalraints;
    world.removeBody(chassisBody);
    world.removeEventListener('preStep', preStepCallback);
    this.world = null;
  }

  double castRay(WheelInfo wheel){
    final rayvector = _castRayRayvector;
    final target = _castRayTarget;

    updateWheelTransformWorld(wheel);
    final chassisBody = this.chassisBody;

    double depth = -1;

    final raylen = wheel.suspensionRestLength + wheel.radius;

    wheel.directionWorld.scale(raylen, rayvector);
    final source = wheel.chassisConnectionPointWorld;
    source.vadd(rayvector, target);
    final raycastResult = wheel.raycastResult;

    //final param = 0;

    raycastResult.reset();
    // Turn off ray collision with the chassis temporarily
    final oldState = chassisBody.collisionResponse;
    chassisBody.collisionResponse = false;

    // Cast ray against world
    world!.rayTest(source, target, raycastResult);
    chassisBody.collisionResponse = oldState;

    final object = raycastResult.body;

    wheel.raycastResult.groundObject = 0;

    if (object != null) {
      depth = raycastResult.distance;
      wheel.raycastResult.hitNormalWorld = raycastResult.hitNormalWorld;
      wheel.isInContact = true;

      final hitDistance = raycastResult.distance;
      wheel.suspensionLength = hitDistance - wheel.radius;

      // clamp on max suspension travel
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

      //final chassisVelocityAtContactPoint = Vec3();
      chassisBody.getVelocityAtWorldPoint(wheel.raycastResult.hitPointWorld, chassisVelocityAtContactPoint);

      final projVel = wheel.raycastResult.hitNormalWorld.dot(chassisVelocityAtContactPoint);

      if (denominator >= -0.1) {
        wheel.suspensionRelativeVelocity = 0;
        wheel.clippedInvContactDotSuspension = 1 / 0.1;
      } else {
        final inv = -1 / denominator;
        wheel.suspensionRelativeVelocity = projVel * inv;
        wheel.clippedInvContactDotSuspension = inv;
      }
    } else {
      //put wheel info as in rest position
      wheel.suspensionLength = wheel.suspensionRestLength + 0 * wheel.maxSuspensionTravel;
      wheel.suspensionRelativeVelocity = 0.0;
      wheel.directionWorld.scale(-1, wheel.raycastResult.hitNormalWorld);
      wheel.clippedInvContactDotSuspension = 1.0;
    }

    return depth;
  }

  void updateWheelTransformWorld(WheelInfo wheel){
    wheel.isInContact = false;
    final chassisBody = this.chassisBody;
    chassisBody.pointToWorldFrame(wheel.chassisConnectionPointLocal, wheel.chassisConnectionPointWorld);
    chassisBody.vectorToWorldFrame(wheel.directionLocal, wheel.directionWorld);
    chassisBody.vectorToWorldFrame(wheel.axleLocal, wheel.axleWorld);
  }

  /// Update one of the wheel transform.
  /// Note when rendering wheels: during each step, wheel transforms are updated BEFORE the chassis; ie. their position becomes invalid after the step. Thus when you render wheels, you must update wheel transforms before rendering them. See raycastVehicle demo for an example.
  /// @param wheelIndex The wheel index to update.;
  void updateWheelTransform(int wheelIndex){
    final up = _tmpVec4;
    final right = _tmpVec5;
    final fwd = _tmpVec6;

    final wheel = wheelInfos[wheelIndex];
    updateWheelTransformWorld(wheel);

    wheel.directionLocal.scale(-1, up);
    right.copy(wheel.axleLocal);
    up.cross(right, fwd);
    fwd.normalize();
    right.normalize();

    // Rotate around steering over the wheelAxle
    final steering = wheel.steering;
    final steeringOrn = Quaternion();
    steeringOrn.setFromAxisAngle(up, steering);

    final rotatingOrn = Quaternion();
    rotatingOrn.setFromAxisAngle(right, wheel.rotation);

    // World rotation of the wheel
    final q = wheel.worldTransform.quaternion;
    chassisBody.quaternion.mult(steeringOrn, q);
    q.mult(rotatingOrn, q);

    q.normalize();

    // world position of the wheel
    final p = wheel.worldTransform.position;
    p.copy(wheel.directionWorld);
    p.scale(wheel.suspensionLength, p);
    p.vadd(wheel.chassisConnectionPointWorld, p);
  }

  /// Get the world transform of one of the wheels
  Transform getWheelTransformWorld(int wheelIndex){
    return wheelInfos[wheelIndex].worldTransform;
  }

  void updateFriction(double timeStep){
    final surfNormalWSScaledProj = _updateFrictionSurfNormalWSScaledProj;

    //calculate the impulse, so that the wheels don't move sidewards
    final wheelInfos = this.wheelInfos;
    final numWheels = wheelInfos.length;
    final chassisBody = this.chassisBody;
    List<Vec3> forwardWS = updateFrictionForwardWS;
    List<Vec3> axle = updateFrictionAxle;

    numWheelsOnGround = 0;

    for (int i = 0; i < numWheels; i++) {
      final wheel = wheelInfos[i];

      final groundObject = wheel.raycastResult.body;
      if (groundObject != null) {
        numWheelsOnGround++;
      }

      wheel.sideImpulse = 0;
      wheel.forwardImpulse = 0;
      if (forwardWS.length-1 < i){//!forwardWS[i]) {
        forwardWS.add(Vec3());
      }
      if (axle.length-1 < i){//!axle[i]) {
        axle.add(Vec3());
      }
    }

    for (int i = 0; i < numWheels; i++) {
      final wheel = wheelInfos[i];

      final groundObject = wheel.raycastResult.body;

      if (groundObject != null) {
        final axlei = axle[i];
        final wheelTrans = getWheelTransformWorld(i);

        // Get world axle
        wheelTrans.vectorToWorld(directions[indexRightAxis.index], axlei);

        final surfNormalWS = wheel.raycastResult.hitNormalWorld;
        final proj = axlei.dot(surfNormalWS);
        surfNormalWS.scale(proj, surfNormalWSScaledProj);
        axlei.vsub(surfNormalWSScaledProj, axlei);
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

        wheel.sideImpulse *= _sideFrictionStiffness2;
      }
    }

    const sideFactor = 1;
    const fwdFactor = 0.5;
    sliding = false;

    for (int i = 0; i < numWheels; i++) {
      final wheel = wheelInfos[i];
      final groundObject = wheel.raycastResult.body;

      double rollingFriction = 0;

      wheel.slipInfo = 1;
      if (groundObject != null) {
        const double defaultRollingFrictionImpulse = 0;
        final double maxImpulse = wheel.brake != 0 ? wheel.brake : defaultRollingFrictionImpulse;

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

      wheel.forwardImpulse = 0;
      wheel.skidInfo = 1;

      if (groundObject != null) {
        wheel.skidInfo = 1;

        final maximp = wheel.suspensionForce * timeStep * wheel.frictionSlip;
        final maximpSide = maximp;

        final maximpSquared = maximp * maximpSide;

        wheel.forwardImpulse = rollingFriction; //wheelInfo.engineForce* timeStep;

        final x = (wheel.forwardImpulse * fwdFactor) / wheel.forwardAcceleration;
        final y = (wheel.sideImpulse * sideFactor) / wheel.sideAcceleration;

        final impulseSquared = x * x + y * y;

        wheel.sliding = false;
        if (impulseSquared > maximpSquared) {
          sliding = true;
          wheel.sliding = true;

          final factor = maximp / math.sqrt(impulseSquared);

          wheel.skidInfo *= factor;
        }
      }
    }

    if (sliding) {
      for (int i = 0; i < numWheels; i++) {
        final wheel = wheelInfos[i];
        if (wheel.sideImpulse != 0) {
          if (wheel.skidInfo < 1) {
            wheel.forwardImpulse *= wheel.skidInfo;
            wheel.sideImpulse *= wheel.skidInfo;
          }
        }
      }
    }

    // apply the impulses
    for (int i = 0; i < numWheels; i++) {
      final wheel = wheelInfos[i];

      final relPos = Vec3();
      wheel.raycastResult.hitPointWorld.vsub(chassisBody.position, relPos);
      // cannons applyimpulse is using world coord for the position
      //rel_pos.copy(wheel.raycastResult.hitPointWorld);

      if (wheel.forwardImpulse != 0) {
        final impulse = Vec3();
        forwardWS[i].scale(wheel.forwardImpulse, impulse);
        chassisBody.applyImpulse(impulse, relPos);
      }

      if (wheel.sideImpulse != 0) {
        final groundObject = wheel.raycastResult.body!;

        final relPos2 = Vec3();
        wheel.raycastResult.hitPointWorld.vsub(groundObject.position, relPos2);
        //rel_pos2.copy(wheel.raycastResult.hitPointWorld);
        final sideImp = Vec3();
        axle[i].scale(wheel.sideImpulse, sideImp);

        // Scale the relative position in the up direction with rollInfluence.
        // If rollInfluence is 1, the impulse will be applied on the hitPoint (easy to roll over), if it is zero it will be applied in the same plane as the center of mass (not easy to roll over).
        chassisBody.vectorToLocalFrame(relPos, relPos);
        relPos[indexUpAxis.index] *= wheel.rollInfluence;//'xyz'[this.indexUpAxis] as 'x' | 'y' | 'z'
        chassisBody.vectorToWorldFrame(relPos, relPos);
        chassisBody.applyImpulse(sideImp, relPos);

        //apply friction impulse on the ground
        sideImp.scale(-1, sideImp);
        groundObject.applyImpulse(sideImp, relPos2);
      }
    }
  }


double calcRollingFriction(
  Body body0,
  Body body1,
  Vec3 frictionPosWorld,
  Vec3 frictionDirectionWorld,
  double maxImpulse,
){
  double j1 = 0;
  final contactPosWorld = frictionPosWorld;

  // final rel_pos1 = Vec3();
  // final rel_pos2 = Vec3();
  final vel1 = _calcRollingFrictionVel1;
  final vel2 = _calcRollingFrictionVel2;
  final vel = _calcRollingFrictionVel;
  // contactPosWorld.vsub(body0.position, rel_pos1);
  // contactPosWorld.vsub(body1.position, rel_pos2);

  body0.getVelocityAtWorldPoint(contactPosWorld, vel1);
  body1.getVelocityAtWorldPoint(contactPosWorld, vel2);
  vel1.vsub(vel2, vel);

  final vrel = frictionDirectionWorld.dot(vel);

  final denom0 = computeImpulseDenominator(body0, frictionPosWorld, frictionDirectionWorld);
  final denom1 = computeImpulseDenominator(body1, frictionPosWorld, frictionDirectionWorld);
  const relaxation = 1;
  final jacDiagABInv = relaxation / (denom0 + denom1);

  // calculate j that moves us to zero relative velocity
  j1 = -vrel * jacDiagABInv;

  if (maxImpulse < j1) {
    j1 = maxImpulse;
  }
  if (j1 < -maxImpulse) {
    j1 = -maxImpulse;
  }

  return j1;
}

  double computeImpulseDenominator(Body body,Vec3 pos,Vec3 normal){
    final r0 = _computeImpulseDenominatorR0;
    final c0 = _computeImpulseDenominatorC0;
    final vec = _computeImpulseDenominatorVec;
    final m = _computeImpulseDenominatorM;

    pos.vsub(body.position, r0);
    r0.cross(normal, c0);
    body.invInertiaWorld.vmult(c0, m);
    m.cross(r0, vec);

    return body.invMass + normal.dot(vec);
  }

  // bilateral finalraint between two dynamic objects
  double resolveSingleBilateral(Body body1,Vec3 pos1,Body body2,Vec3 pos2,Vec3 normal){
    final normalLenSqr = normal.lengthSquared();
    if (normalLenSqr > 1.1) {
      return 0 ; // no impulse
    }
    // final rel_pos1 = Vec3();
    // final rel_pos2 = Vec3();
    // pos1.vsub(body1.position, rel_pos1);
    // pos2.vsub(body2.position, rel_pos2);

    final vel1 = _resolveSingleBilateralVel1;
    final vel2 = _resolveSingleBilateralVel2;
    final vel = _resolveSingleBilateralVel;
    body1.getVelocityAtWorldPoint(pos1, vel1);
    body2.getVelocityAtWorldPoint(pos2, vel2);

    vel1.vsub(vel2, vel);

    final relVel = normal.dot(vel);

    const contactDamping = 0.2;
    final massTerm = 1 / (body1.invMass + body2.invMass);
    final impulse = -contactDamping * relVel * massTerm;

    return impulse;
  }

}