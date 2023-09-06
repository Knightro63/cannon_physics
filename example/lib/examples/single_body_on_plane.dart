import 'package:flutter/material.dart';
import 'package:three_dart/three_dart.dart';
import '../src/demo.dart';
import 'package:cannon_physics/cannon_physics.dart' as cannon;

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
    groundBody.quaternion.setFromEuler(-Math.PI / 2, 0, 0);
    world.addBody(groundBody);
    demo.addVisual(groundBody);
  }
  void box(){
    final world = demo.world;

    const size = 2.0;

    final boxShape = cannon.Box(cannon.Vec3(size, size, size));

    final body = cannon.Body(mass: 30);
    body.addShape(boxShape);
    body.position.set(0, size * 2, size);
    world.addBody(body);
    demo.addVisual(body);
  }
  void sphere(){
    final world = demo.world;

    const size = 2.0;

    final sphereShape = cannon.Sphere(size);

    final body = cannon.Body(mass: 30);
    body.addShape(sphereShape);
    body.position.set(0, size * 2, size);
    world.addBody(body);
    demo.addVisual(body);
  }
  void setupWorld(){
    plane();
    sphere();
    demo.start();
  }
  @override
  Widget build(BuildContext context) {
    return demo.threeDart();
  }
}