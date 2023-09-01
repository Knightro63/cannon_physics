import 'dart:math' as math;
import '../math/vec3.dart';
import '../objects/body.dart';
import '../shapes/sphere.dart';
import '../shapes/box.dart';
import '../constraints/hinge_constraint.dart';
import '../world/world_class.dart';

/// Simple vehicle helper class with spherical rigid body wheels.
class RigidVehicle {
  /// The bodies of the wheels.
  List<Body> wheelBodies = [];
  late Vec3 coordinateSystem;
  /// The chassis body.
  late Body chassisBody;
  /// The constraints.
  List<HingeConstraint> constraints = [];//: (HingeConstraint & { motorTargetVelocity?: number })[];
  /// The wheel axes.
  List<Vec3> wheelAxes = [];
  /// The wheel forces.
  List<double> wheelForces = [];

  RigidVehicle({
    Vec3? coordinateSystem,
    Body? chassisBody
  }){
    this.coordinateSystem = coordinateSystem?.clone() ?? Vec3(1, 2, 3);
    this.chassisBody = chassisBody ?? Body(mass: 1, shape: Box(Vec3(5, 0.5, 2)));
  }

  final _torque = Vec3();
  final _worldAxis = Vec3();

  /// Add a wheel
  int addWheel({
    Body? body,
    Vec3? position,
    Vec3? axis,
    Vec3? direction,
  }){
    Body wheelBody = body ?? Body(mass: 1, shape: Sphere(1.2));

    wheelBodies.add(wheelBody);
    wheelForces.add(0);

    // Position constrain wheels
    final pos = position?.clone() ?? Vec3();

    // Set position locally to the chassis
    final worldPosition = Vec3();
    chassisBody.pointToWorldFrame(pos, worldPosition);
    wheelBody.position.set(worldPosition.x, worldPosition.y, worldPosition.z);

    // Constrain wheel
    final ax = axis?.clone() ?? Vec3(0, 0, 1);
    wheelAxes.add(ax);

    final hingeConstraint = HingeConstraint(
      chassisBody, 
      wheelBody, 
      pivotA: pos,
      axisA: ax,
      pivotB: Vec3.zero,
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
    constraints[wheelIndex].axisA.set(-c * x + s * z, 0, s * x + c * z);
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

    axis.scale(value, _torque);
    wheelBody.vectorToWorldFrame(_torque, _torque);
    bodyTorque.vadd(_torque, bodyTorque);
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
