import '../constraints/point_to_point_constraint.dart';
import '../equations/cone_equation.dart';
import '../equations/rotational_equation.dart';
import '../math/vec3.dart';
import '../objects/body.dart';

//export type ConeTwistConstraintOptions = ConstructorParameters<typeof ConeTwistConstraint>[2]

/// A Cone Twist constraint, useful for ragdolls.
class ConeTwistConstraint extends PointToPointConstraint {
  /// The axis direction for the constraint of the body A.
  late Vec3 axisA;

  /// The axis direction for the constraint of the body B.
  late Vec3 axisB;

  /// The aperture angle of the cone.
  double angle;

  /// The twist angle of the joint.
  double twistAngle;
  late ConeEquation coneEquation;
  late RotationalEquation twistEquation;

  ConeTwistConstraint(
    Body bodyA,
    Body bodyB,
    {
      Vec3? pivotA,
      Vec3? pivotB,
      Vec3? axisA,
      Vec3? axisB,
      this.angle = 0,
      this.twistAngle = 0,
      double maxForce = 1e6,
      bool collideConnected = false
    }
  ):super(bodyA, bodyB, pivotA, pivotB, maxForce){

    // Set pivot point in between
    this.pivotA = pivotA?.clone() ?? Vec3();
    this.pivotB = pivotB?.clone() ?? Vec3();

    this.axisA = axisA?.clone() ?? Vec3();
    this.axisB = axisB?.clone() ?? Vec3();

    coneEquation = ConeEquation(bodyA, bodyB, maxForce: maxForce, angle: angle, axisA: this.axisA, axisB: this.axisB);
    final c = coneEquation;
    twistEquation = RotationalEquation(bodyA, bodyB, maxForce: maxForce, axisA: this.axisA, axisB: this.axisB);
    final t = twistEquation;

    // Make the cone equation push the bodies toward the cone axis, not outward
    c.maxForce = 0;
    c.minForce = -maxForce;

    // Make the twist equation add torque toward the initial position
    t.maxForce = 0;
    t.minForce = -maxForce;

    equations.addAll([c,t]);
  }

  // final _coneTwistConstraintUpdateTmpVec1 = Vec3();
  // final _coneTwistConstraintUpdateTmpVec2 = Vec3();

  @override
  void update() {
    final bodyA = this.bodyA;
    final bodyB = this.bodyB;
    final cone = coneEquation;
    final twist = twistEquation;

    super.update();

    // Update the axes to the cone constraint
    bodyA.vectorToWorldFrame(axisA, cone.axisA);
    bodyB.vectorToWorldFrame(axisB, cone.axisB);

    // Update the world axes in the twist constraint
    axisA.tangents(twist.axisA, twist.axisA);
    bodyA.vectorToWorldFrame(twist.axisA, twist.axisA);

    axisB.tangents(twist.axisB, twist.axisB);
    bodyB.vectorToWorldFrame(twist.axisB, twist.axisB);

    cone.angle = angle;
    twist.maxAngle = twistAngle;
  }
}
