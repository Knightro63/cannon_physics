import 'dart:math' as math;
import '../math/vec3.dart';
import 'rigid_body.dart';
import '../rigid_body_shapes/sphere.dart';
import '../rigid_body_shapes/box.dart';
import '../constraints/hinge_constraint.dart';
import '../world/world_class.dart';
import 'package:vector_math/vector_math.dart' hide Sphere;

/// Simple vehicle helper class with spherical rigid body wheels.
class RigidVehicle {
  /// The bodies of the wheels.
  List<Body> wheelBodies = [];
  late Vector3 coordinateSystem;
  /// The chassis body.
  late Body chassisBody;
  /// The constraints.
  List<HingeConstraint> constraints = [];//: (HingeConstraint & { motorTargetVelocity?: number })[];
  /// The wheel axes.
  List<Vector3> wheelAxes = [];
  /// The wheel forces.
  List<double> wheelForces = [];

  RigidVehicle({
    Vector3? coordinateSystem,
    Body? chassisBody
  }){
    this.coordinateSystem = coordinateSystem?.clone() ?? Vector3(1, 2, 3);
    this.chassisBody = chassisBody ?? Body(mass: 1, shape: Box(Vector3(5, 0.5, 2)));
  }

  final _torque = Vector3.zero();
  final _worldAxis = Vector3.zero();

  /// Add a wheel
  int addWheel({
    Body? body,
    Vector3? position,
    Vector3? axis,
    Vector3? direction,
  }){
    Body wheelBody = body ?? Body(mass: 1, shape: Sphere(1.2));

    wheelBodies.add(wheelBody);
    wheelForces.add(0);

    // Position constrain wheels
    final pos = position?.clone() ?? Vector3.zero();

    // Set position locally to the chassis
    final worldPosition = Vector3.zero();
    chassisBody.pointToWorldFrame(pos, worldPosition);
    wheelBody.position.setValues(worldPosition.x, worldPosition.y, worldPosition.z);

    // Constrain wheel
    final ax = axis?.clone() ?? Vector3(0, 0, 1);
    wheelAxes.add(ax);

    final hingeConstraint = HingeConstraint(
      chassisBody, 
      wheelBody, 
      pivotA: pos,
      axisA: ax,
      pivotB: Vector3.zero(),
      axisB: ax,
      collideConnected: false,
    );
    constraints.add(hingeConstraint);

    return wheelBodies.length - 1;
  }

  /// Set the steering value of a wheel.
  /// @todo check coordinateSystem
  void setSteeringValue(double value, int wheelIndex) {
    // Set angle of the hinge axis
    final axis = wheelAxes[wheelIndex];

    final c = math.cos(value);
    final s = math.sin(value);
    final x = axis.x;
    final z = axis.z;
    constraints[wheelIndex].axisA.setValues(-c * x + s * z, 0, s * x + c * z);
  }

  /// Set the target rotational speed of the hinge constraint.
  void setMotorSpeed(double value, int wheelIndex) {
    final hingeConstraint = constraints[wheelIndex];
    hingeConstraint.enableMotor();
    hingeConstraint.motorTargetVelocity = value;
  }

  /// Set the target rotational speed of the hinge constraint.
  void disableMotor(int wheelIndex) {
    final hingeConstraint = constraints[wheelIndex];
    hingeConstraint.disableMotor();
  }

  /// Set the wheel force to apply on one of the wheels each time step
  void setWheelForce(double value, int wheelIndex) {
    wheelForces[wheelIndex] = value;
  }

  /// Apply a torque on one of the wheels.
  void applyWheelForce(double value, int wheelIndex) {
    final axis = wheelAxes[wheelIndex];
    final wheelBody = wheelBodies[wheelIndex];
    final bodyTorque = wheelBody.torque;

    axis.scale2(value, _torque);
    wheelBody.vectorToWorldFrame(_torque, _torque);
    bodyTorque.add2(_torque, bodyTorque);
  }

  /// Add the vehicle including its constraints to the world.
  void addToWorld(World world) {
    final constraints = this.constraints;
     wheelBodies.addAll([chassisBody]);
    final bodies = wheelBodies;

    for (int i = 0; i < bodies.length; i++) {
      world.addBody(bodies[i]);
    }

    for (int i = 0; i < constraints.length; i++) {
      world.addConstraint(constraints[i]);
    }

    world.addEventListener('preStep', _update);//_update.bind(this));
  }

  void _update(event){
    final wheelForces = this.wheelForces;
    for (int i = 0; i < wheelForces.length; i++) {
      applyWheelForce(wheelForces[i], i);
    }
  }

  /// Remove the vehicle including its constraints from the world.
  void removeFromWorld(World world) {
    final constraints = this.constraints;
    wheelBodies.addAll([chassisBody]);
    final bodies = wheelBodies;

    for (int i = 0; i < bodies.length; i++) {
      world.removeBody(bodies[i]);
    }

    for (int i = 0; i < constraints.length; i++) {
      world.removeConstraint(constraints[i]);
    }
  }

  /// Get current rotational velocity of a wheel
  double getWheelSpeed(int wheelIndex){
    final axis = wheelAxes[wheelIndex];
    final wheelBody = wheelBodies[wheelIndex];
    final w = wheelBody.angularVelocity;
    chassisBody.vectorToWorldFrame(axis, _worldAxis);
    return w.dot(_worldAxis);
  }
}
