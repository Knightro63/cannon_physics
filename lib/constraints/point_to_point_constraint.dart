import '../constraints/constraint_class.dart';
import '../equations/contact_equation.dart';
import '../objects/rigid_body.dart';
import 'package:vector_math/vector_math.dart';
import '../math/quaternion.dart';
/// Connects two bodies at given offset points.
/// @example
///     const bodyA = Body({ mass: 1 })
///     const bodyB = Body({ mass: 1 })
///     bodyA.position.set(-1, 0, 0)
///     bodyB.position.set(1, 0, 0)
///     bodyA.addShape(shapeA)
///     bodyB.addShape(shapeB)
///     world.addBody(bodyA)
///     world.addBody(bodyB)
///     const localPivotA = Vector3(1, 0, 0)
///     const localPivotB = Vector3(-1, 0, 0)
///     const constraint = PointToPointConstraint(bodyA, localPivotA, bodyB, localPivotB)
///     world.addConstraint(constraint)
class PointToPointConstraint extends Constraint {
  /// Pivot, defined locally in bodyA.
  late Vector3 pivotA;
  /// Pivot, defined locally in bodyB.
  late Vector3 pivotB;

  late ContactEquation equationX;
  late ContactEquation equationY;
  late ContactEquation equationZ;

  /// [pivotA] The point relative to the center of mass of bodyA which bodyA is constrained to.
  /// [bodyB] Body that will be constrained in a similar way to the same point as bodyA. We will therefore get a link between bodyA and bodyB. If not specified, bodyA will be constrained to a static point.
  /// [pivotB] The point relative to the center of mass of bodyB which bodyB is constrained to.
  /// [maxForce] The maximum force that should be applied to constrain the bodies.
  PointToPointConstraint(
    Body bodyA,
    Body bodyB,
    [
      Vector3? pivotA,
      Vector3? pivotB,
      double maxForce = 1e6
    ]
  ):super(bodyA, bodyB) {
    this.pivotA = pivotA?.clone() ?? Vector3.zero();
    this.pivotB = pivotB?.clone() ?? Vector3.zero();
    equationX = ContactEquation(bodyA, bodyB);
    final x = equationX;
    equationY = ContactEquation(bodyA, bodyB);
    final y = equationY;
    equationZ = ContactEquation(bodyA, bodyB);
    final z = equationZ;

    // Equations to be fed to the solver
    equations.addAll([x,y,z]);

    // Make the equations bidirectional
    x.minForce = -maxForce;
    y.minForce = -maxForce;
    z.minForce = -maxForce;
    x.maxForce = maxForce;
    y.maxForce = maxForce;
    z.maxForce = maxForce;

    x.ni.setValues(1, 0, 0);
    y.ni.setValues(0, 1, 0);
    z.ni.setValues(0, 0, 1);
  }
  @override
  void update() {
    final bodyA = this.bodyA;
    final bodyB = this.bodyB;
    final x = equationX;
    final y = equationY;
    final z = equationZ;

    // Rotate the pivots to world space
    bodyA.quaternion.vmult(pivotA, x.ri);
    bodyB.quaternion.vmult(pivotB, x.rj);

    y.ri.setFrom(x.ri);
    y.rj.setFrom(x.rj);
    z.ri.setFrom(x.ri);
    z.rj.setFrom(x.rj);
  }
}
