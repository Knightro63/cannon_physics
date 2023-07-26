import '../constraints/constraint.dart';
import '../equations/contact_equation.dart';
import '../math/vec3.dart';
import '../objects/body.dart';

/**
 * Connects two bodies at given offset points.
 * @example
 *     const bodyA = Body({ mass: 1 })
 *     const bodyB = Body({ mass: 1 })
 *     bodyA.position.set(-1, 0, 0)
 *     bodyB.position.set(1, 0, 0)
 *     bodyA.addShape(shapeA)
 *     bodyB.addShape(shapeB)
 *     world.addBody(bodyA)
 *     world.addBody(bodyB)
 *     const localPivotA = Vec3(1, 0, 0)
 *     const localPivotB = Vec3(-1, 0, 0)
 *     const constraint = PointToPointConstraint(bodyA, localPivotA, bodyB, localPivotB)
 *     world.addConstraint(constraint)
 */
class PointToPointConstraint extends Constraint {
  /**
   * Pivot, defined locally in bodyA.
   */
  Vec3 pivotA;
  /**
   * Pivot, defined locally in bodyB.
   */
  Vec3 pivotB;

  ContactEquation equationX;
  ContactEquation equationY;
  ContactEquation equationZ;

  /**
   * @param pivotA The point relative to the center of mass of bodyA which bodyA is constrained to.
   * @param bodyB Body that will be constrained in a similar way to the same point as bodyA. We will therefore get a link between bodyA and bodyB. If not specified, bodyA will be constrained to a static point.
   * @param pivotB The point relative to the center of mass of bodyB which bodyB is constrained to.
   * @param maxForce The maximum force that should be applied to constrain the bodies.
   */
  PointToPointConstraint(
    Body bodyA,
    this.pivotA,
    Body bodyB,
    this.pivotB,
    double maxForce = 1e6
  ):super(bodyA, bodyB) {
    const x = (this.equationX = ContactEquation(bodyA, bodyB));
    const y = (this.equationY = ContactEquation(bodyA, bodyB));
    const z = (this.equationZ = ContactEquation(bodyA, bodyB));

    // Equations to be fed to the solver
    this.equations.push(x, y, z);

    // Make the equations bidirectional
    x.minForce = y.minForce = z.minForce = -maxForce;
    x.maxForce = y.maxForce = z.maxForce = maxForce;

    x.ni.set(1, 0, 0);
    y.ni.set(0, 1, 0);
    z.ni.set(0, 0, 1);
  }

  void update() {
    const bodyA = this.bodyA;
    const bodyB = this.bodyB;
    const x = this.equationX;
    const y = this.equationY;
    const z = this.equationZ;

    // Rotate the pivots to world space
    bodyA.quaternion.vmult(this.pivotA, x.ri);
    bodyB.quaternion.vmult(this.pivotB, x.rj);

    y.ri.copy(x.ri);
    y.rj.copy(x.rj);
    z.ri.copy(x.ri);
    z.rj.copy(x.rj);
  }
}
