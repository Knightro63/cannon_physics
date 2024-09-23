import 'package:flutter/material.dart';
import 'dart:math' as math;
import '../src/demo.dart';
import 'package:cannon_physics/cannon_physics.dart' as cannon;
import 'package:vector_math/vector_math.dart' as vmath;

class Impulses extends StatefulWidget {
  const Impulses({
    Key? key,
    this.offset = const Offset(0,0),
    this.settings
  }) : super(key: key);

  final Offset offset;
  final DemoSettings? settings;

  @override
  _ImpulsesState createState() => _ImpulsesState();
}

class _ImpulsesState extends State<Impulses> {
  late Demo demo;
  final radius = 1.0;
  final mass = 2.0;
  final strength = 500.0;
  final dt = 1 / 60;
  final damping = 0.5;

  @override
  void initState() {
    demo = Demo(
      onSetupComplete: (){setState(() {});},
    );
    setupWorld();
    super.initState();
  }
  @override
  void dispose() {
    demo.dispose();
    super.dispose();
  }
  void centerImpulse(){
    final world = demo.world;

    final shape = cannon.Sphere(radius);
    final body = cannon.Body(mass:mass);
    body.addShape(shape);
    body.linearDamping = damping;
    body.angularDamping = damping;
    world.addBody(body);
    demo.addVisual(body);

    final impulse = vmath.Vector3(-strength * dt, 0, 0);
    body.applyImpulse(impulse);
  }
  void topImpulse(){
    final world = demo.world;

    final shape = cannon.Sphere(radius);
    final body = cannon.Body( mass:mass);
    body.addShape(shape);
    body.linearDamping = damping;
    body.angularDamping = damping;
    world.addBody(body);
    demo.addVisual(body);

    // The top of the sphere, relative to the sphere center
    final topPoint = vmath.Vector3(0, radius, 0);
    final impulse = vmath.Vector3(-strength * dt, 0, 0);
    body.applyImpulse(impulse, topPoint);
  }
  void centerForce(){
    final world = demo.world;

    final shape = cannon.Sphere(radius);
    final body = cannon.Body(mass:mass);
    body.addShape(shape);
    body.linearDamping = damping;
    body.angularDamping = damping;
    world.addBody(body);
    demo.addVisual(body);

    final force = vmath.Vector3(-strength, 0, 0);
    body.applyForce(force);
  }
  void topForce(){
    final world = demo.world;

    final shape = cannon.Sphere(radius);
    final body = cannon.Body(mass:mass);
    body.addShape(shape);
    body.linearDamping = damping;
    body.angularDamping = damping;
    world.addBody(body);
    demo.addVisual(body);

    // The top of the sphere, relative to the sphere center
    final topPoint = vmath.Vector3(0, radius, 0);
    final force = vmath.Vector3(-strength, 0, 0);
    body.applyForce(force, topPoint);
  }
  void localForce(){
    final world = demo.world;

    final shape = cannon.Sphere(radius);
    final body = cannon.Body(mass:mass);
    body.addShape(shape);
    body.linearDamping = damping;
    body.angularDamping = damping;
    body.quaternion.setFromEuler(0, 0, math.pi);
    world.addBody(body);
    demo.addVisual(body);

    // it's the top point, but since the sphere is rotated
    // by 180 degrees, it is the bottom point to the right
    final topPoint = vmath.Vector3(0, radius, 0);
    final force = vmath.Vector3(-strength, 0, 0);
    body.applyLocalForce(force, topPoint);
  }
  void torque(){
    final world = demo.world;

    final shape = cannon.Sphere(radius);
    final body = cannon.Body(mass:mass);
    body.addShape(shape);
    body.linearDamping = damping;
    body.angularDamping = damping;
    world.addBody(body);
    demo.addVisual(body);

    // add a positive rotation in the z-axis
    final torque = vmath.Vector3(0, 0, strength);
    body.applyTorque(torque);
  }
  void setupWorld(){
    demo.addScene('Center Impulse',centerImpulse);
    demo.addScene('Top Impulse',topImpulse);
    demo.addScene('Center Force',centerForce);
    demo.addScene('Top Force',topForce);
    demo.addScene('Local Force',localForce);
    demo.addScene('Torque',torque);
  }
  @override
  Widget build(BuildContext context) {
    return demo.threeDart();
  }
}