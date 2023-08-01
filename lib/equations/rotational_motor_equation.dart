import '../equations/equation.dart';
import '../math/vec3.dart';
import '../objects/body.dart';

/**
 * Rotational motor constraint. Tries to keep the relative angular velocity of the bodies to a given value.
 */
class RotationalMotorEquation extends Equation {
  /**
   * World oriented rotational axis.
   */
  Vec3 axisA = Vec3();
  /**
   * World oriented rotational axis.
   */
  Vec3 axisB = Vec3();
  /**
   * Motor velocity.
   */
  double targetVelocity = 0;

  RotationalMotorEquation(Body bodyA, Body bodyB, [double maxForce = 1e6]):super(bodyA, bodyB, -maxForce, maxForce);
  @override
  double computeB(double h){
    final b = this.b;
    final axisA = this.axisA;
    final axisB = this.axisB;
    final GA = jacobianElementA;
    final GB = jacobianElementB;

    // g = 0
    // gdot = axisA * wi - axisB * wj
    // gdot = G * W = G * [vi wi vj wj]
    // =>
    // G = [0 axisA 0 -axisB]

    GA.rotational.copy(axisA);
    axisB.negate(GB.rotational);

    final GW = computeGW() - targetVelocity;
    final GiMf = computeGiMf();

    final B = -GW * b - h * GiMf;

    return B;
  }
}
