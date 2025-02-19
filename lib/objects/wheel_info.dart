import '../math/vec3.dart';
import '../math/transform.dart';
import '../collision/raycast_result.dart';
import 'rigid_body.dart';
import 'package:vector_math/vector_math.dart';

final chassisVelocityAtContactPoint = Vector3.zero();

class WheelRaycastResult extends RaycastResult{
  WheelRaycastResult():super();

  double suspensionLength = 0;
  Vector3? directionWorld;
  double groundObject = 0;
}

/// WheelInfo
class WheelInfo {
  /// Max travel distance of the suspension, in meters.
  double maxSuspensionTravel;
  /// Speed to apply to the wheel rotation when the wheel is sliding.
  double customSlidingRotationalSpeed;
  /// If the customSlidingRotationalSpeed should be used.
  bool useCustomSlidingRotationalSpeed;
  bool sliding = false;

  /// Connection point, defined locally in the chassis body frame.
  late Vector3 chassisConnectionPointLocal;

  late Vector3 chassisConnectionPointWorld; 
  late Vector3 directionLocal;
  late Vector3 directionWorld;
  late Vector3 axleLocal;
  late Vector3 axleWorld;

  double suspensionRestLength;
  double suspensionMaxLength;
  double radius;
  double suspensionStiffness;
  double dampingCompression;
  double dampingRelaxation;
  double frictionSlip;
  double forwardAcceleration;
  double sideAcceleration;
  double steering;

  /// Rotation value, in radians.
  double rotation;
  double deltaRotation;
  double rollInfluence;
  double maxSuspensionForce;
  double engineForce = 0;
  double brake = 0;
  bool isFrontWheel;
  double clippedInvContactDotSuspension;
  double suspensionRelativeVelocity;
  double suspensionForce;
  double slipInfo;
  double skidInfo;
  double suspensionLength;
  double sideImpulse = 0;
  double forwardImpulse = 0;
  /// The result from raycasting.
  WheelRaycastResult raycastResult = WheelRaycastResult();
  /// Wheel world transform.
  Transform worldTransform = Transform();
  bool isInContact = false;

  WheelInfo({
      Vector3? chassisConnectionPointLocal,
      Vector3? chassisConnectionPointWorld,
      Vector3? directionLocal,
      Vector3? directionWorld,
      Vector3? axleLocal,
      Vector3? axleWorld,
      this.suspensionRestLength = 1,
      this.suspensionMaxLength = 2,
      this.radius = 1,
      this.suspensionStiffness = 100,
      this.dampingCompression = 10,
      this.dampingRelaxation = 10,
      this.frictionSlip = 10.5,
      this.forwardAcceleration = 1,
      this.sideAcceleration = 1,
      this.steering = 0,
      this.rotation = 0,
      this.deltaRotation = 0,
      this.rollInfluence = 0.01,
      this.maxSuspensionForce = double.infinity,
      this.isFrontWheel = true,
      this.clippedInvContactDotSuspension = 1,
      this.suspensionRelativeVelocity = 0,
      this.suspensionForce = 0,
      this.slipInfo = 0,
      this.skidInfo = 0,
      this.suspensionLength = 0,
      this.maxSuspensionTravel = 1,
      this.useCustomSlidingRotationalSpeed = false,
      this.customSlidingRotationalSpeed = -0.1
    }){
    this.chassisConnectionPointLocal = chassisConnectionPointLocal?.clone() ?? Vector3.zero();
    this.chassisConnectionPointWorld = chassisConnectionPointWorld?.clone() ?? Vector3.zero();
    this.directionLocal = directionLocal?.clone() ?? Vector3.zero();
    this.directionWorld = directionWorld?.clone() ?? Vector3.zero();
    this.axleLocal = axleLocal?.clone() ?? Vector3.zero();
    this.axleWorld = axleWorld?.clone() ?? Vector3.zero();
  }
  final _relpos = Vector3.zero();
  
  WheelInfo copy(){
    return WheelInfo(
      chassisConnectionPointLocal:chassisConnectionPointLocal,
      chassisConnectionPointWorld:chassisConnectionPointWorld,
      directionLocal:directionLocal,
      directionWorld:directionWorld,
      axleLocal:axleLocal,
      axleWorld:axleWorld,
      suspensionRestLength:suspensionRestLength,
      suspensionMaxLength:suspensionMaxLength,
      radius:radius,
      suspensionStiffness:suspensionStiffness,
      dampingCompression:dampingCompression,
      dampingRelaxation:dampingRelaxation,
      frictionSlip:frictionSlip,
      forwardAcceleration:forwardAcceleration,
      sideAcceleration:sideAcceleration,
      steering:steering,
      rotation:rotation,
      deltaRotation:deltaRotation,
      rollInfluence:rollInfluence,
      maxSuspensionForce:maxSuspensionForce,
      isFrontWheel:isFrontWheel,
      clippedInvContactDotSuspension:clippedInvContactDotSuspension,
      suspensionRelativeVelocity:suspensionRelativeVelocity,
      suspensionForce:suspensionForce,
      slipInfo:slipInfo,
      skidInfo:skidInfo,
      suspensionLength:suspensionLength,
      maxSuspensionTravel:maxSuspensionTravel,
      useCustomSlidingRotationalSpeed:useCustomSlidingRotationalSpeed,
      customSlidingRotationalSpeed:customSlidingRotationalSpeed
    );
  }
  void updateWheel(Body chassis) {
    final raycastResult = this.raycastResult;

    if (isInContact) {
      final project = raycastResult.hitNormalWorld.dot(raycastResult.directionWorld!);
      raycastResult.hitPointWorld.sub2(chassis.position, _relpos);
      chassis.getVelocityAtWorldPoint(_relpos, chassisVelocityAtContactPoint);
      final projVel = raycastResult.hitNormalWorld.dot(chassisVelocityAtContactPoint);
      if (project >= -0.1) {
        suspensionRelativeVelocity = 0.0;
        clippedInvContactDotSuspension = 1.0 / 0.1;
      } else {
        final inv = -1 / project;
        suspensionRelativeVelocity = projVel * inv;
        clippedInvContactDotSuspension = inv;
      }
    } else {
      // Not in contact : position wheel in a nice (rest length) position
      raycastResult.suspensionLength = suspensionRestLength;
      suspensionRelativeVelocity = 0.0;
      raycastResult.directionWorld!.scale2(-1, raycastResult.hitNormalWorld);
      clippedInvContactDotSuspension = 1.0;
    }
  }
}
