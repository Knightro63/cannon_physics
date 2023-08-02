import '../constraints/constraint.dart';
import '../equations/contact_equation.dart';
import '../math/vec3.dart';
import '../objects/body.dart';


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
///     const localPivotA = Vec3(1, 0, 0)
///     const localPivotB = Vec3(-1, 0, 0)
///     const constraint = PointToPointConstraint(bodyA, localPivotA, bodyB, localPivotB)
///     world.addConstraint(constraint)
class PointToPointConstraint extends Constraint {
  /// Pivot, defined locally in bodyA.
  late Vec3 pivotA;
  /// Pivot, defined locally in bodyB.
  late Vec3 pivotB;

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
      Vec3? pivotA,
      Vec3? pivotB,
      double maxForce = 1e6
    ]
  ):super(bodyA, bodyB) {
    this.pivotA = pivotA ?? Vec3();
    this.pivotB = pivotB ?? Vec3();
    equationX = ContactEquation(bodyA, bodyB);
    final x = equationX;
    equationY = ContactEquation(bodyA, bodyB);
    final y = equationY;
    equationZ = ContactEquation(bodyA, bodyB);
    final z = equationZ;

    // Equations to be fed to the solver
    equations.add(x);
    equations.add(y);
    equations.add(z);

    // Make the equations bidirectional
    x.minForce = y.minForce = z.minForce = -maxForce;
    x.maxForce = y.maxForce = z.maxForce = maxForce;

    x.ni.set(1, 0, 0);
    y.ni.set(0, 1, 0);
    z.ni.set(0, 0, 1);
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

    y.ri.copy(x.ri);
    y.rj.copy(x.rj);
    z.ri.copy(x.ri);
    z.rj.copy(x.rj);
  }
}
