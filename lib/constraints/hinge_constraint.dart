import '../math/vec3.dart';
import '../objects/body.dart';
import '../constraints/point_to_point_constraint.dart';
import '../equations/rotational_equation.dart';
import '../equations/rotational_motor_equation.dart';

/// Hinge constraint. Think of it as a door hinge. It tries to keep the door in the correct place and with the correct orientation.
class HingeConstraint extends PointToPointConstraint {
  /// Rotation axis, defined locally in bodyA.
  late Vec3 axisA;

  /// Rotation axis, defined locally in bodyB.
  late Vec3 axisB;

  late RotationalEquation rotationalEquation1;
  late RotationalEquation rotationalEquation2;
  late RotationalMotorEquation motorEquation;
  double? motorTargetVelocity;

  HingeConstraint(
    Body bodyA,
    Body bodyB,
    {
      Vec3? pivotA,
      Vec3? pivotB,
      Vec3? axisA,
      Vec3? axisB,
      bool? collideConnected,
      double maxForce = 1e6
    }
  ):super(bodyA, bodyB, pivotA, pivotB, maxForce) {
    this.axisA = axisA?.clone() ?? Vec3(1, 0, 0);
    this.axisA.normalize();

    this.axisB = axisB?.clone() ?? Vec3(1, 0, 0);
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

  final _hingeConstraintUpdateTmpVec1 = Vec3();
  final _hingeConstraintUpdateTmpVec2 = Vec3();

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
    r1.axisB.copy(worldAxisB);
    r2.axisB.copy(worldAxisB);

    if (motorEquation.enabled) {
      bodyA.quaternion.vmult(this.axisA, motor.axisA);
      bodyB.quaternion.vmult(this.axisB, motor.axisB);
    }
  }
}