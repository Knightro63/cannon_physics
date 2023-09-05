import 'package:flutter/material.dart';
import 'package:three_dart/three_dart.dart';
import '../src/demo.dart';
import 'package:cannon_physics/cannon_physics.dart' as cannon;

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
    groundBody.quaternion.setFromEuler(-Math.PI / 2, 0, 0);
    world.addBody(groundBody);
    demo.addVisual(groundBody);

    const size = 0.25;
    const mass = 1.0;

    final boxShape = cannon.Box(cannon.Vec3(size, size, size));

    List<cannon.Body> boxes = [];
    for (int i = 0; i < N; i++) {
      // start with random positions
      final position = cannon.Vec3(
        (Math.random() * 2 - 1) * 2.5,
        Math.random() * 10,
        (Math.random() * 2 - 1) * 2.5
      );

      final boxBody = cannon.Body(
        position: position,
        mass: mass,
      );
      boxBody.addShape(boxShape);
      world.addBody(boxBody);
      demo.addVisual(boxBody);
      boxes.add(boxBody);
    }
  }

  void setupWorld(){
    setupFallingBoxes(500);
    demo.start();
  }
  @override
  Widget build(BuildContext context) {
    return demo.threeDart();
  }
}