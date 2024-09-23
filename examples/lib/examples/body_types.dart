import 'package:flutter/material.dart';
import '../src/demo.dart';
import 'dart:math' as math;
import 'package:cannon_physics/cannon_physics.dart' as cannon;
import 'package:vector_math/vector_math.dart' as vmath;

class BodyTypes extends StatefulWidget {
  const BodyTypes({
    Key? key,
    this.offset = const Offset(0,0),
    this.settings
  }) : super(key: key);

  final Offset offset;
  final DemoSettings? settings;

  @override
  _BodyTypesState createState() => _BodyTypesState();
}

class _BodyTypesState extends State<BodyTypes> {
  late Demo demo;

  @override
  void initState() {
    demo = Demo(
      onSetupComplete: (){setState(() {});},
      settings: DemoSettings(
        gx: 0,
        gy: -40,
        gz: 0,
        k: 1e8,
        d: 10
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
    final world = demo.world;
    // Static ground plane
    final groundShape = cannon.Plane();
    final groundBody = cannon.Body(mass: 0 );
    groundBody.addShape(groundShape);
    groundBody.quaternion.setFromEuler(-math.pi / 2, 0, 0);
    world.addBody(groundBody);
    demo.addVisual(groundBody);

    const size = 2.0;

    // Kinematic Box
    // Does only collide with dynamic bodies, but does not respond to any force.
    // Its movement can be controlled by setting its velocity.
    final boxShape = cannon.Box(vmath.Vector3(size, size, size));
    final boxBody = cannon.Body(
      mass: 0,
      type: cannon.BodyTypes.kinematic,
      position: vmath.Vector3(0, size * 0.5, 0),
    );
    boxBody.addShape(boxShape);
    world.addBody(boxBody);
    demo.addVisual(boxBody);

    // To control the box movement we must set its velocity
    boxBody.velocity.setValues(0, 5, 0);
    double secs = 0;
    demo.addAnimationEvent((dt){
      secs += dt;
      if(secs > 1){
        if (boxBody.velocity.y < 0) {
          boxBody.velocity.setValues(0, 5, 0);
        } else {
          boxBody.velocity.setValues(0, -5, 0);
        }
        secs = 0;
      }
    });
    // Dynamic Sphere
    // Dynamic bodies can collide with bodies of all other types.
    final sphereShape = cannon.Sphere(size);
    final sphereBody = cannon.Body(
      mass: 5,
      position: vmath.Vector3(0, size * 3, 0),
    );
    sphereBody.addShape(sphereShape);
    world.addBody(sphereBody);
    demo.addVisual(sphereBody);
}

  void setupWorld(){
    setScene();
  }
  @override
  Widget build(BuildContext context) {
    return demo.threeDart();
  }
}