import 'package:cannon_physics_example/src/conversion_utils.dart';
import 'package:flutter/material.dart';
import 'package:flutter_gl/native-array/index.dart';
import 'package:three_dart/three_dart.dart';
import 'package:three_dart_jsm/three_dart_jsm.dart';
import '../src/demo.dart';
import 'package:cannon_physics/cannon_physics.dart' as cannon;

class SoftBody extends StatefulWidget {
  const SoftBody({
    Key? key,
    this.offset = const Offset(0,0),
    this.settings
  }) : super(key: key);

  final Offset offset;
  final DemoSettings? settings;

  @override
  _SPHState createState() => _SPHState();
}

class _SPHState extends State<SoftBody> {
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
    groundBody.position.setValues(0, 1, 0);
    groundBody.quaternion.setFromEuler(-Math.PI / 2, 0, 0);
    // world.addBody(groundBody);
    // demo.addVisual(groundBody);
  }
  void setBody(){
    setScene();
    final world = demo.world;

    // Max solver iterations: Use more for better force propagation, but keep in mind that it's not very computationally cheap!
    world.solver.iterations = 10;

    final sb = cannon.SphereSoftBody(
      radius: 6,
      mass: 0.1,
      stiffness: 1000
    );

    for (final key in sb.particleBodies.keys) {
      demo.addVisual(sb.particleBodies[key]!);
    }

    for(final cons in sb.constraints){
      BufferGeometry line = BufferGeometry()
        ..setFromPoints([
            cons.bodyA.position.toVector3(),
            cons.bodyB.position.toVector3()
        ]);
      demo.scene.add(
        Line(
          line, 
          LineBasicMaterial()
        )
      );
    }

    sb.addToWorld(world);

    // Sphere moving towards right
    // cannon.Sphere sphereShape = cannon.Sphere(1);
    // cannon.Body sphereBody = cannon.Body(mass: 1);
    // sphereBody.addShape(sphereShape);
    // sphereBody.position.set(-3, 20, 10);
    // cannon.Vec3 impulse = cannon.Vec3(0, 0, 0);
    // cannon.Vec3 topPoint = cannon.Vec3(0, 1, 0);
    // sphereBody.applyImpulse(impulse, topPoint);
    // sphereBody.linearDamping = 0.3;
    // sphereBody.angularDamping = 0.3;
    // world.addBody(sphereBody);
    // demo.addVisual(sphereBody);
  }

  void setupWorld(){
    demo.addScene('Lock',setBody);
  }
  @override
  Widget build(BuildContext context) {
    return demo.threeDart();
  }
}