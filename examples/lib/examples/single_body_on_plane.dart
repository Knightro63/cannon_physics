import 'package:flutter/material.dart';
import 'dart:math' as math;
import '../src/demo.dart';
import 'package:cannon_physics/cannon_physics.dart' as cannon;
import 'package:vector_math/vector_math.dart' as vmath;

class SBOP extends StatefulWidget {
  const SBOP({
    Key? key,
    this.offset = const Offset(0,0),
    this.settings
  }) : super(key: key);

  final Offset offset;
  final DemoSettings? settings;

  @override
  _SBOPState createState() => _SBOPState();
}

class _SBOPState extends State<SBOP> {
  late Demo demo;

  @override
  void initState() {
    demo = Demo(
      onSetupComplete: (){setState(() {});},
      settings: DemoSettings(
        gx: 0,
        gy: -10,
        gz: 0,
        k: 1e7,
        d:4
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
  void plane(){
    final world = demo.world;

    final groundShape = cannon.Plane();
    final groundBody = cannon.Body(mass: 0);
    groundBody.addShape(groundShape);
    groundBody.quaternion.setFromEuler(-math.pi / 2, 0, 0);
    world.addBody(groundBody);
    demo.addVisual(groundBody);
  }
  void box(){
    plane();
    final world = demo.world;

    const size = 2.0;

    final boxShape = cannon.Box(vmath.Vector3(size, size, size));

    final body = cannon.Body(mass: 30);
    body.addShape(boxShape);
    body.position.setValues(0, size * 2, size);
    world.addBody(body);
    demo.addVisual(body);
  }
  void sphere(){
    plane();
    final world = demo.world;

    const size = 2.0;

    final sphereShape = cannon.Sphere(size);

    final body = cannon.Body(mass: 30);
    body.addShape(sphereShape);
    body.position.setValues(0, size * 2, size);
    world.addBody(body);
    demo.addVisual(body);
  }
  void setupWorld(){
    demo.addScene('Box',box);
    demo.addScene('Sphere',sphere);
  }
  @override
  Widget build(BuildContext context) {
    return demo.threeDart();
  }
}