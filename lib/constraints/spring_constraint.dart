import 'package:cannon_physics/cannon_physics.dart';
import 'package:vector_math/vector_math.dart';

//export type ConeTwistConstraintOptions = ConstructorParameters<typeof ConeTwistConstraint>[2]

/// A Cone Twist constraint, useful for ragdolls.
class SpringConstraint extends Constraint {
  /// The axis direction for the constraint of the body A.
  late Vector3 axisA;

  /// The axis direction for the constraint of the body B.
  late Vector3 axisB;

  /// Rest length of the spring. A number > 0.
  late double _restLength;

  /// Stiffness of the spring. A number >= 0.
  double stiffness;

  /// Damping of the spring. A number >= 0.
  double damping;

  late ContactEquation distanceEquation;

  SpringConstraint(
    Body bodyA,
    Body bodyB,
    {
      this.stiffness = 1,
      this.damping = 1,
    }
  ):super(bodyA, bodyB){
    _restLength = bodyA.position.distanceTo(bodyB.position);
    distanceEquation = ContactEquation(bodyA, bodyB);
    final eq = distanceEquation;
    equations.add(eq);

    // Make it bidirectional
    eq.minForce = -stiffness;
    eq.maxForce = stiffness;
  }

  @override
  void update() {
    final bodyA = this.bodyA;
    final bodyB = this.bodyB;
    final eq = distanceEquation;
    final halfDist = _restLength * 0.5;
    final normal = eq.ni;

    bodyB.position.sub2(bodyA.position, normal);
    normal.normalize();
    normal.scale2(halfDist, eq.ri);
    normal.scale2(-halfDist, eq.rj);
  }
}
