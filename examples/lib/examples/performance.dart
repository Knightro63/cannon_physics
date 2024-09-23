import 'package:flutter/material.dart';
import 'dart:math' as math;
import '../src/demo.dart';
import 'package:cannon_physics/cannon_physics.dart' as cannon;
import 'package:vector_math/vector_math.dart' as vmath;

class Performance extends StatefulWidget {
  const Performance({
    Key? key,
    this.offset = const Offset(0,0),
    this.settings
  }) : super(key: key);

  final Offset offset;
  final DemoSettings? settings;

  @override
  _PerformanceState createState() => _PerformanceState();
}

class _PerformanceState extends State<Performance> {
  late Demo demo;

  @override
  void initState() {
    demo = Demo(
      onSetupComplete: (){setState(() {});},
      settings: DemoSettings(
        gx: 0,
        gy: -50,
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
  void setupFallingBoxes(int N) {
    final world = demo.world;

    final groundShape = cannon.Plane();
    final groundBody = cannon.Body(mass: 0 );
    groundBody.addShape(groundShape);
    groundBody.quaternion.setFromEuler(-math.pi / 2, 0, 0);
    world.addBody(groundBody);
    demo.addVisual(groundBody);

    const size = 0.25;
    const mass = 1.0;

    final boxShape = cannon.Box(vmath.Vector3(size, size, size));

    for (int i = 0; i < N; i++) {
      // start with random positions
      final position = vmath.Vector3(
        (math.Random().nextDouble() * 2 - 1) * 2.5,
        math.Random().nextDouble() * 10,
        (math.Random().nextDouble() * 2 - 1) * 2.5
      );

      final boxBody = cannon.Body(
        position: position,
        mass: mass,
      );
      boxBody.addShape(boxShape);
      world.addBody(boxBody);
      demo.addVisual(boxBody);
    }
  }

  void setupWorld(){
    setupFallingBoxes(500);
  }
  @override
  Widget build(BuildContext context) {
    return demo.threeDart();
  }
}