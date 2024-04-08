import '../math/vec3.dart';
import '../math/quaternion.dart';
import '../objects/rigid_body.dart';
import '../constraints/point_to_point_constraint.dart';
import '../equations/rotational_equation.dart';
import '../equations/rotational_motor_equation.dart';
import 'package:vector_math/vector_math.dart';

/// Hinge constraint. Think of it as a door hinge. It tries to keep the door in the correct place and with the correct orientation.
class HingeConstraint extends PointToPointConstraint {
  /// Rotation axis, defined locally in bodyA.
  late Vector3 axisA;

  /// Rotation axis, defined locally in bodyB.
  late Vector3 axisB;

  late RotationalEquation rotationalEquation1;
  late RotationalEquation rotationalEquation2;
  late RotationalMotorEquation motorEquation;
  double? motorTargetVelocity;

  HingeConstraint(
    Body bodyA,
    Body bodyB,
    {
      Vector3? pivotA,
      Vector3? pivotB,
      Vector3? axisA,
      Vector3? axisB,
      bool? collideConnected,
      double maxForce = 1e6
    }
  ):super(bodyA, bodyB, pivotA, pivotB, maxForce) {
    this.axisA = axisA ?? Vector3(1, 0, 0);
    this.axisA.normalize();

    this.axisB = axisB ?? Vector3(1, 0, 0);
    this.axisB.normalize();

    this.collideConnected = collideConnected ?? true;
    rotationalEquation1 = RotationalEquation(bodyA, bodyB, axisA: this.axisA,axisB: this.axisB, maxForce: maxForce);
    final rotational1 = rotationalEquation1;
    rotationalEquation2 = RotationalEquation(bodyA, bodyB, axisA: this.axisA,axisB: this.axisB, maxForce: maxForce);
    final rotational2 = rotationalEquation2;
    motorEquation = RotationalMotorEquation(bodyA, bodyB, maxForce);
    final motor = motorEquation;
    motor.enabled = false; // Not enabled by default

    // Equations to be fed to the solver
    equations.addAll([rotational1,rotational2,motor]);
  }

  final _hingeConstraintUpdateTmpVec1 = Vector3.zero();
  final _hingeConstraintUpdateTmpVec2 = Vector3.zero();

  /// enableMotor
  void enableMotor() {
    motorEquation.enabled = true;
  }

  /// disableMotor
  void disableMotor() {
    motorEquation.enabled = false;
  }

  /// setMotorSpeed
  void setMotorSpeed(double speed) {
    motorEquation.targetVelocity = speed;
  }

  /// setMotorMaxForce
  void setMotorMaxForce(double maxForce) {
    motorEquation.maxForce = maxForce;
    motorEquation.minForce = -maxForce;
  }

  /// update
  @override
  void update() {
    final bodyA = this.bodyA;
    final bodyB = this.bodyB;
    final motor = motorEquation;
    final r1 = rotationalEquation1;
    final r2 = rotationalEquation2;
    final worldAxisA = _hingeConstraintUpdateTmpVec1;
    final worldAxisB = _hingeConstraintUpdateTmpVec2;

    final axisA = this.axisA;
    final axisB = this.axisB;

    super.update();

    // Get world axes
    bodyA.quaternion.vmult(axisA, worldAxisA);
    bodyB.quaternion.vmult(axisB, worldAxisB);
    worldAxisA.tangents(r1.axisA, r2.axisA);
    r1.axisB.setFrom(worldAxisB);
    r2.axisB.setFrom(worldAxisB);

    if (motorEquation.enabled) {
      bodyA.quaternion.vmult(this.axisA, motor.axisA);
      bodyB.quaternion.vmult(this.axisB, motor.axisB);
    }
  }
}