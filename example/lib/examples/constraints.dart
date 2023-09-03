import 'package:flutter/material.dart';
import 'package:three_dart/three_dart.dart';
import '../src/demo.dart';
import 'package:cannon_physics/cannon_physics.dart' as cannon;

class Constraints extends StatefulWidget {
  const Constraints({
    Key? key,
    this.offset = const Offset(0,0),
    this.settings
  }) : super(key: key);

  final Offset offset;
  final DemoSettings? settings;

  @override
  _ConstraintsState createState() => _ConstraintsState();
}

class _ConstraintsState extends State<Constraints> {
  late Demo demo;

  @override
  void initState() {
    demo = Demo(
      onSetupComplete: (){setState(() {});},
      settings: DemoSettings(
        gx: 0,
        gy: -40,
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
    groundBody.position.set(0, 1, 0);
    groundBody.quaternion.setFromEuler(-Math.PI / 2, 0, 0);
    world.addBody(groundBody);
    demo.addVisual(groundBody);
  }

  void lockScene(){
    final world = demo.world;

    world.gravity.set(0, -10, 0);
    world.solver.iterations = 20;

    const size = 0.5;
    const mass = 1.0;
    const space = size * 0.1;

    final boxShape = cannon.Box(cannon.Vec3(size, size, size));

    const N = 10;
    cannon.Body? previous;
    for (int i = 0; i < N; i++) {
      // Create a box
      final boxBody = cannon.Body(
        mass: mass,
        shape: boxShape,
        position: cannon.Vec3(-(N - i - N / 2) * (size * 2 + 2 * space), size * 6 + space, 0),
      );
      world.addBody(boxBody);
      demo.addVisual(boxBody);

      if (previous != null) {
        // Connect the current body to the last one
        final lockConstraint = cannon.LockConstraint(boxBody, previous);
        world.addConstraint(lockConstraint);
      }

      // To keep track of which body was added last
      previous = boxBody;
    }

    // Create stands
    final body1 = cannon.Body(
      mass: 0,
      shape: boxShape,
      position: cannon.Vec3(-(-N / 2 + 1) * (size * 2 + 2 * space), size * 3, 0),
    );
    world.addBody(body1);
    demo.addVisual(body1);

    final body2 = cannon.Body(
      mass: 0,
      shape: boxShape,
      position: cannon.Vec3(-(N / 2) * (size * 2 + space * 2), size * 3, 0),
    );
    world.addBody(body2);
    demo.addVisual(body2);
  }

  void setupWorld(){
    setScene();
    lockScene();
    demo.start();
  }
  @override
  Widget build(BuildContext context) {
    return demo.threeDart();
  }
}