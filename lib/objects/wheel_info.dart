import '../math/vec3.dart';
import '../math/transform.dart';
import '../collision/raycast_result.dart';
import '../objects/body.dart';


class WheelRaycastResult extends RaycastResult{
  WheelRaycastResult():super();

  double suspensionLength = 0;
  Vec3? directionWorld;
  double groundObject = 0;
}

/**
 * WheelInfo
 */
class WheelInfo {
  /**
   * Max travel distance of the suspension, in meters.
   * @default 1
   */
  double maxSuspensionTravel;
  /**
   * Speed to apply to the wheel rotation when the wheel is sliding.
   * @default -0.1
   */
  double customSlidingRotationalSpeed;
  /**
   * If the customSlidingRotationalSpeed should be used.
   * @default false
   */
  bool useCustomSlidingRotationalSpeed;
  /**
   * sliding
   */
  bool sliding = false;
  /**
   * Connection point, defined locally in the chassis body frame.
   */
  late Vec3 chassisConnectionPointLocal;
  /**
   * chassisConnectionPointWorld
   */
  late Vec3 chassisConnectionPointWorld; 
  /**
   * directionLocal
   */
  late Vec3 directionLocal;
  /**
   * directionWorld
   */
  late Vec3 directionWorld;
  /**
   * axleLocal
   */
  late Vec3 axleLocal;
  /**
   * axleWorld
   */
  late Vec3 axleWorld;
  /**
   * suspensionRestLength
   * @default 1
   */
  double suspensionRestLength;
  /**
   * suspensionMaxLength
   * @default 2
   */
  double suspensionMaxLength;
  /**
   * radius
   * @default 1
   */
  double radius;
  /**
   * suspensionStiffness
   * @default 100
   */
  double suspensionStiffness;
  /**
   * dampingCompression
   * @default 10
   */
  double dampingCompression;
  /**
   * dampingRelaxation
   * @default 10
   */
  double dampingRelaxation;
  /**
   * frictionSlip
   * @default 10.5
   */
  double frictionSlip;
  /** forwardAcceleration */
  double forwardAcceleration;
  /** sideAcceleration */
  double sideAcceleration;
  /**
   * steering
   * @default 0
   */
  double steering;
  /**
   * Rotation value, in radians.
   * @default 0
   */
  double rotation;
  /**
   * deltaRotation
   * @default 0
   */
  double deltaRotation;
  /**
   * rollInfluence
   * @default 0.01
   */
  double rollInfluence;
  /**
   * maxSuspensionForce
   */
  double maxSuspensionForce;
  /**
   * engineForce
   */
  double engineForce = 0;
  /**
   * brake
   */
  double brake = 0;
  /**
   * isFrontWheel
   * @default true
   */
  bool isFrontWheel;
  /**
   * clippedInvContactDotSuspension
   * @default 1
   */
  double clippedInvContactDotSuspension;
  /**
   * suspensionRelativeVelocity
   * @default 0
   */
  double suspensionRelativeVelocity;
  /**
   * suspensionForce
   * @default 0
   */
  double suspensionForce;
  /**
   * slipInfo
   */
  double slipInfo;
  /**
   * skidInfo
   * @default 0
   */
  double skidInfo;
  /**
   * suspensionLength
   * @default 0
   */
  double suspensionLength;
  /**
   * sideImpulse
   */
  double sideImpulse = 0;
  /**
   * forwardImpulse
   */
  double forwardImpulse = 0;
  /**
   * The result from raycasting.
   */
  WheelRaycastResult raycastResult = WheelRaycastResult();
  /**
   * Wheel world transform.
   */
  Transform worldTransform = Transform();
  /**
   * isInContact
   */
  bool isInContact = false;

  WheelInfo({
      Vec3? chassisConnectionPointLocal,
      Vec3? chassisConnectionPointWorld,
      Vec3? directionLocal,
      Vec3? directionWorld,
      Vec3? axleLocal,
      Vec3? axleWorld,
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
    this.chassisConnectionPointLocal = chassisConnectionPointLocal?.clone() ?? Vec3();
    this.chassisConnectionPointWorld = chassisConnectionPointWorld?.clone() ?? Vec3();
    this.directionLocal = directionLocal?.clone() ?? Vec3();
    this.directionWorld = directionWorld?.clone() ?? Vec3();
    this.axleLocal = axleLocal?.clone() ?? Vec3();
    this.axleWorld = axleWorld?.clone() ?? Vec3();
  }

  void updateWheel(Body chassis) {
    final raycastResult = this.raycastResult;

    if (isInContact) {
      final project = raycastResult.hitNormalWorld.dot(raycastResult.directionWorld!);
      raycastResult.hitPointWorld.vsub(chassis.position, relpos);
      chassis.getVelocityAtWorldPoint(relpos, chassis_velocity_at_contactPoint);
      final projVel = raycastResult.hitNormalWorld.dot(chassis_velocity_at_contactPoint);
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
      raycastResult.directionWorld!.scale(-1, raycastResult.hitNormalWorld);
      clippedInvContactDotSuspension = 1.0;
    }
  }
}

final chassis_velocity_at_contactPoint = Vec3();
final relpos = Vec3();
