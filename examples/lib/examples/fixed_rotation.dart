import 'package:flutter/material.dart';
import 'package:three_dart/three_dart.dart';
import '../src/demo.dart';
import 'package:cannon_physics/cannon_physics.dart' as cannon;
import 'package:vector_math/vector_math.dart' as vmath;

class FixedRotation extends StatefulWidget {
  const FixedRotation({
    Key? key,
    this.offset = const Offset(0,0),
    this.settings
  }) : super(key: key);

  final Offset offset;
  final DemoSettings? settings;

  @override
  _FixedRotationState createState() => _FixedRotationState();
}

class _FixedRotationState extends State<FixedRotation> {
  late Demo demo;

  @override
  void initState() {
    demo = Demo(
      onSetupComplete: (){setState(() {});},
      settings: DemoSettings(
        gx: 0,
        gy: -10,
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
    final world = demo.world;

    final groundShape = cannon.Plane();
    final groundBody = cannon.Body(mass: 0);
    groundBody.addShape(groundShape);
    groundBody.quaternion.setFromEuler(-Math.PI / 2, 0, 0);
    world.addBody(groundBody);
    demo.addVisual(groundBody);

    const size = 1.0;

    // Create a box with fixed rotation
    final shape1 = cannon.Box(vmath.Vector3(size, size, size));
    final boxBody1 = cannon.Body(mass: 1);
    boxBody1.addShape(shape1);
    boxBody1.position.setValues(0, size, 0);
    boxBody1.fixedRotation = true;
    boxBody1.updateMassProperties();
    world.addBody(boxBody1);
    demo.addVisual(boxBody1);

    // Another one
    final shape2 = cannon.Box(vmath.Vector3(size, size, size));
    final boxBody2 = cannon.Body(mass: 1);
    boxBody2.addShape(shape2);
    boxBody2.position.setValues(-(size * 3) / 2, size * 4, 0);
    boxBody2.fixedRotation = true;
    boxBody2.updateMassProperties();
    world.addBody(boxBody2);
    demo.addVisual(boxBody2);
  }

  void setupWorld(){
    setScene();
  }
  @override
  Widget build(BuildContext context) {
    return demo.threeDart();
  }
}