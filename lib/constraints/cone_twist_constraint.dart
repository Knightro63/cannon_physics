import '../constraints/point_to_point_constraint.dart';
import '../equations/cone_equation.dart';
import '../equations/rotational_equation.dart';
import '../math/vec3.dart';
import '../objects/body.dart';

//export type ConeTwistConstraintOptions = ConstructorParameters<typeof ConeTwistConstraint>[2]

/**
 * A Cone Twist constraint, useful for ragdolls.
 */
class ConeTwistConstraint extends PointToPointConstraint {
  /**
   * The axis direction for the constraint of the body A.
   */
  Vec3 axisA;
  /**
   * The axis direction for the constraint of the body B.
   */
  Vec3 axisB;
  /**
   * The aperture angle of the cone.
   */
  double angle;
  /**
   * The twist angle of the joint.
   */
  double twistAngle;
  ConeEquation coneEquation;
  RotationalEquation twistEquation;

  ConeTwistConstraint(
    Body bodyA,
    Body bodyB,
    {
      /**
       * The pivot point for bodyA.
       */
      Vec3? pivotA,
      /**
       * The pivot point for bodyB.
       */
      Vec3? pivotB?,
      /**
       * The axis direction for the constraint of the body A.
       */
      Vec3? axisA?,
      /**
       * The axis direction for the constraint of the body B.
       */
      Vec3? axisB?,
      /**
       * The aperture angle of the cone.
       * @default 0
       */
      this.angle = 0,
      /**
       * The twist angle of the joint.
       * @default 0
       */
      this.twistAngle = 0,
      /**
       * The maximum force that should be applied to constrain the bodies.
       * @default 1e6
       */
      double maxForce = 1e6,
      /**
       * Wether to collide the connected bodies or not.
       * @default false
       */
      bool collideConnected = false
    }
  ) {

    // Set pivot point in between
    const pivotA = options.pivotA ? options.pivotA.clone() :Vec3()
    const pivotB = options.pivotB ? options.pivotB.clone() :Vec3()

    super(bodyA, pivotA, bodyB, pivotB, maxForce)

    this.axisA = options.axisA ? options.axisA.clone() :Vec3()
    this.axisB = options.axisB ? options.axisB.clone() :Vec3()

    const c = (this.coneEquation =ConeEquation(bodyA, bodyB, options))
    const t = (this.twistEquation =RotationalEquation(bodyA, bodyB, options))

    // Make the cone equation push the bodies toward the cone axis, not outward
    c.maxForce = 0
    c.minForce = -maxForce

    // Make the twist equation add torque toward the initial position
    t.maxForce = 0
    t.minForce = -maxForce

    this.equations.push(c, t)
  }

  void update() {
    const bodyA = this.bodyA;
    const bodyB = this.bodyB;
    const cone = this.coneEquation;
    const twist = this.twistEquation;

    super.update();

    // Update the axes to the cone constraint
    bodyA.vectorToWorldFrame(this.axisA, cone.axisA);
    bodyB.vectorToWorldFrame(this.axisB, cone.axisB);

    // Update the world axes in the twist constraint
    this.axisA.tangents(twist.axisA, twist.axisA);
    bodyA.vectorToWorldFrame(twist.axisA, twist.axisA);

    this.axisB.tangents(twist.axisB, twist.axisB);
    bodyB.vectorToWorldFrame(twist.axisB, twist.axisB);

    cone.angle = this.angle;
    twist.maxAngle = this.twistAngle;
  }
}

const ConeTwistConstraint_update_tmpVec1 =Vec3();
const ConeTwistConstraint_update_tmpVec2 =Vec3();
