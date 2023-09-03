import 'package:flutter/material.dart';
import 'package:three_dart/three_dart.dart';
import '../src/demo.dart';
import 'package:cannon_physics/cannon_physics.dart' as cannon;

class Jenga extends StatefulWidget {
  const Jenga({
    Key? key,
    this.offset = const Offset(0,0),
    this.settings
  }) : super(key: key);

  final Offset offset;
  final DemoSettings? settings;

  @override
  _JengaState createState() => _JengaState();
}

class _JengaState extends State<Jenga> {
  late Demo demo;

  @override
  void initState() {
    demo = Demo(
      onSetupComplete: (){setState(() {});},
      settings: DemoSettings(
        gx: 0,
        gy: -5,
        gz: 0,
        k: 5e6,
        iterations: 50
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

    const size = 0.5;
    const mass = 1.0;
    const gap = 0.02;

    // Layers
    for (int i = 0; i < 10; i++) {
      for (int j = 0; j < 3; j++) {
        cannon.Body body = cannon.Body(mass: mass );

        cannon.Vec3 halfExtents;
        int dx;
        int dz;
        if (i % 2 == 0) {
          halfExtents = cannon.Vec3(size, size, size * 3);
          dx = 1;
          dz = 0;
        } else {
          halfExtents = cannon.Vec3(size * 3, size, size);
          dx = 0;
          dz = 1;
        }

        cannon.Box shape = cannon.Box(halfExtents);
        body.addShape(shape);
        body.position.set(
          2 * (size + gap) * (j - 1) * dx,
          2 * (size + gap) * (i + 1),
          2 * (size + gap) * (j - 1) * dz
        );

        world.addBody(body);
        demo.addVisual(body);
      }
    }

    cannon.Plane groundShape = cannon.Plane();
    cannon.Body groundBody = cannon.Body(mass: 0 );
    groundBody.addShape(groundShape);
    groundBody.quaternion.setFromEuler(-Math.PI / 2, 0, 0);
    world.addBody(groundBody);
    demo.addVisual(groundBody);
  }

  void setupWorld(){
    setScene();
  }
  @override
  Widget build(BuildContext context) {
    return demo.threeDart();
  }
}