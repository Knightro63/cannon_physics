import 'equation_class.dart';
import '../objects/rigid_body.dart';
import 'package:vector_math/vector_math.dart';

/// Rotational motor constraint. Tries to keep the relative angular velocity of the bodies to a given value.
class RotationalMotorEquation extends Equation {
  /// World oriented rotational axis.
  Vector3 axisA = Vector3.zero();
  /// World oriented rotational axis.
  Vector3 axisB = Vector3.zero();
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

    ga.rotational.setFrom(axisA);
    gb.rotational..setFrom(axisB)..negate();

    final gw = computeGW() - targetVelocity;
    final giMf = computeGiMf();

    final B = -gw * b - h * giMf;

    return B;
  }
}
