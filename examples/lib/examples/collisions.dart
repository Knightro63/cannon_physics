import 'package:flutter/material.dart';
import '../src/demo.dart';
import 'package:cannon_physics/cannon_physics.dart' as cannon;
import 'dart:math' as math;
import 'package:vector_math/vector_math.dart' as vmath;

class Collisions extends StatefulWidget {
  const Collisions({
    Key? key,
    this.offset = const Offset(0,0),
    this.settings
  }) : super(key: key);

  final Offset offset;
  final DemoSettings? settings;

  @override
  _CollisionsState createState() => _CollisionsState();
}

class _CollisionsState extends State<Collisions> {
  late Demo demo;

  @override
  void initState() {
    demo = Demo(
      onSetupComplete: (){setState(() {});},
      settings: DemoSettings(
        gx: 0,
        gy: 0,
        gz: 0,
      )
    );
    setupWorld();
    super.initState();
  }
  @override
  void dispose() {
    demo.dispose();
    super.dispose();
  }
  void setScene(){
    demo.addScene('Sphere Sphere',setScene0);
    demo.addScene('Box Sphere',setScene1);
    demo.addScene('Box Edge Sphere',setScene2);
    demo.addScene('Box Point Sphere',setScene3);
  }
  void setScene0(){
    final world = demo.world;
    final sphereShape = cannon.Sphere(1);

    // Sphere 1
    final body1 = cannon.Body(mass: 5 );
    body1.addShape(sphereShape);
    body1.position.setValues(-5, 0, 0);
    body1.velocity.setValues(5, 0, 0);
    body1.linearDamping = 0;
    world.addBody(body1);
    demo.addVisual(body1);

    // Sphere 2
    final body2 = cannon.Body(mass: 5 );
    body2.addShape(sphereShape);
    body2.linearDamping = 0;
    body2.position.setValues(5, 0, 0);
    body2.velocity.setValues(-5, 0, 0);
    world.addBody(body2);
    demo.addVisual(body2);
  }
  void setScene1(){
    final world = demo.world;

    final boxShape = cannon.Box(vmath.Vector3(1, 1, 1));
    final sphereShape = cannon.Sphere(1);
    // Box
    final body1 = cannon.Body(mass: 5 );
    body1.addShape(boxShape);
    body1.position.setValues(-5, 0, 0);
    body1.velocity.setValues(5, 0, 0);
    body1.linearDamping = 0;
    world.addBody(body1);
    demo.addVisual(body1);

    // Sphere
    final body2 = cannon.Body(mass: 5 );
    body2.addShape(sphereShape);
    body2.position.setValues(5, 0, 0);
    body2.velocity.setValues(-5, 0, 0);
    body2.linearDamping = 0;
    world.addBody(body2);
    demo.addVisual(body2);
  }
  void setScene2(){
    final world = demo.world;

    final boxShape = cannon.Box(vmath.Vector3(1, 1, 1));
    final sphereShape = cannon.Sphere(1);
    final quaternion = vmath.Quaternion(0,0,0,1);
    quaternion.setFromEuler(0, math.pi * 0.25, 0);

    // Box
    final body1 = cannon.Body(
      mass: 5,
      position: vmath.Vector3(-5,0,0),
      velocity: vmath.Vector3(5,0,0),
      linearDamping: 0,
      quaternion: quaternion,
      shape: boxShape
    );
    world.addBody(body1);
    demo.addVisual(body1);

    // Sphere
    final body2 = cannon.Body(
      mass: 5,
      position: vmath.Vector3(5,0,0),
      velocity: vmath.Vector3(-5,0,0),
      linearDamping: 0,
      shape: sphereShape
    );
    world.addBody(body2);
    demo.addVisual(body2);
  }
  void setScene3(){
    final world = demo.world;
    final boxShape = cannon.Box(vmath.Vector3(1, 1, 1));
    final sphereShape = cannon.Sphere(1);

    // Box
    final body1 = cannon.Body(mass: 5);
    body1.addShape(boxShape);
    body1.position.setValues(-5, 0, 0);
    body1.velocity.setValues(5, 0, 0);
    body1.linearDamping = 0;
    final quaternion1 = vmath.Quaternion(0,0,0,1);
    quaternion1.setFromEuler(0, math.pi * 0.25, 0);
    final quaternion2 = vmath.Quaternion(0,0,0,1);
    quaternion2.setFromEuler(0, 0, math.pi * 0.25);
    final quaternion = quaternion1.multiply(quaternion2);
    body1.quaternion.setFrom(quaternion);
    world.addBody(body1);
    demo.addVisual(body1);

    // Sphere
    final body2 = cannon.Body(mass: 5);
    body2.addShape(sphereShape);
    body2.position.setValues(5, 0, 0);
    body2.velocity.setValues(-5, 0, 0);
    body2.linearDamping = 0;
    world.addBody(body2);
    demo.addVisual(body2);
  }
  void setupWorld(){
    setScene();
  }
  @override
  Widget build(BuildContext context) {
    return demo.threeDart();
  }
}