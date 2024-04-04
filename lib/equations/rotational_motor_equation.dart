import 'equation_class.dart';
import '../math/vec3.dart';
import '../objects/rigid_body.dart';

/// Rotational motor constraint. Tries to keep the relative angular velocity of the bodies to a given value.
class RotationalMotorEquation extends Equation {
  /// World oriented rotational axis.
  Vec3 axisA = Vec3();
  /// World oriented rotational axis.
  Vec3 axisB = Vec3();
  /// Motor velocity.
  double targetVelocity = 0;

  RotationalMotorEquation(Body bodyA, Body bodyB, [double maxForce = 1e6]):super(bodyA, bodyB, -maxForce, maxForce);
  
  @override
  double computeB(double h){
    final b = this.b;
    final axisA = this.axisA;
    final axisB = this.axisB;
    final ga = jacobianElementA;
    final gb = jacobianElementB;

    ga.rotational.copy(axisA);
    axisB.negate(gb.rotational);

    final gw = computeGW() - targetVelocity;
    final giMf = computeGiMf();

    final B = -gw * b - h * giMf;

    return B;
  }
}
