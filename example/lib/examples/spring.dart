import 'package:flutter/material.dart';
import 'package:three_dart/three_dart.dart';
import '../src/demo.dart';
import 'package:cannon_physics/cannon_physics.dart' as cannon;
import 'dart:math' as math;

extension on cannon.Quaternion{
  Quaternion toQuaternion(){
    return Quaternion(x,y,z,w);
  }
}
extension on cannon.Vec3{
  Vector3 toVector3(){
    return Vector3(x,y,z);
  }
}

class Spring extends StatefulWidget {
  const Spring({
    Key? key,
    this.offset = const Offset(0,0),
    this.settings
  }) : super(key: key);

  final Offset offset;
  final DemoSettings? settings;

  @override
  _SpringState createState() => _SpringState();
}

class _SpringState extends State<Spring> {
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

    const size = 1.0;

    // Create a static sphere
    cannon.Sphere sphereShape = cannon.Sphere(0.1);
    cannon.Body sphereBody = cannon.Body( mass: 0 );
    sphereBody.addShape(sphereShape);
    world.addBody(sphereBody);
    demo.addVisual(sphereBody);

    // Create a box body
    cannon.Box boxShape = cannon.Box(cannon.Vec3(size, size, size * 0.3));
    cannon.Body boxBody = cannon.Body( mass: 5 );
    boxBody.addShape(boxShape);
    boxBody.position.set(size, -size, 0);
    world.addBody(boxBody);
    demo.addVisual(boxBody);

    cannon.Spring spring = cannon.Spring(boxBody, sphereBody, 
      localAnchorA: cannon.Vec3(-size, size, 0),
      localAnchorB: cannon.Vec3(0, 0, 0),
      restLength: 0,
      stiffness: 50,
      damping: 1,
    );

    // Compute the force after each step
    demo.addEventListener((event) => {spring.applyForce()});
  }

  void setupWorld(){
    setScene();
  }
  @override
  Widget build(BuildContext context) {
    return demo.threeDart();
  }
}