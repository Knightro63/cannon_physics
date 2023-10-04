import 'package:flutter/material.dart';
import 'package:three_dart/three_dart.dart';
import '../src/demo.dart';
import 'package:cannon_physics/cannon_physics.dart' as cannon;

class Compound extends StatefulWidget {
  const Compound({
    Key? key,
    this.offset = const Offset(0,0),
    this.settings
  }) : super(key: key);

  final Offset offset;
  final DemoSettings? settings;

  @override
  _CompoundState createState() => _CompoundState();
}

class _CompoundState extends State<Compound> {
  late Demo demo;

  @override
  void initState() {
    demo = Demo(
      onSetupComplete: (){setState(() {});},
      settings: DemoSettings(
        gx: 0,
        gy: -30,
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
  void sceneBox(){
    final world = demo.world;

    // Create the compound shape
    // const compoundShape = cannon.Compound();
    const size = 1.5;

    // Now create a Body for our Compound
    const mass = 10.0;
    final body = cannon.Body(mass:mass);
    body.position.set(0, 6, 0);
    body.quaternion.setFromEuler(0, 0, Math.PI * 0.03);

    // Use a box shape as child shape
    final shape = cannon.Box(cannon.Vec3(size * 0.5, size * 0.5, size * 0.5));

    // We can add the same shape several times to position child shapes within the Compound.
    body.addShape(shape, cannon.Vec3(-size, -size, 0));
    body.addShape(shape, cannon.Vec3(-size, size, 0));
    body.addShape(shape, cannon.Vec3(size, -size, 0));
    body.addShape(shape, cannon.Vec3(size, size, 0));
    // Note: we only use translational offsets. The third argument could be a CANNON.Quaternion to rotate the child shape.
    body.addShape(shape, cannon.Vec3(size, 0, 0));
    body.addShape(shape, cannon.Vec3(0, -size, 0));
    body.addShape(shape, cannon.Vec3(0, size, 0));

    world.addBody(body);
    demo.addVisual(body);
  }

  // Here we create a compound made out of spheres
  void sceneSpheres(){
    final world = demo.world;

    const mass = 10.0;
    final body = cannon.Body(mass:mass);

    // Compound shape
    final sphereShape = cannon.Sphere(1);
    body.addShape(sphereShape, cannon.Vec3(-1, -1, 0));
    body.addShape(sphereShape, cannon.Vec3(-1, 1, 0));
    body.addShape(sphereShape, cannon.Vec3(1, -1, 0));
    body.addShape(sphereShape, cannon.Vec3(1, 1, 0));

    body.position.set(0, 6, 0);
    body.quaternion.setFromEuler(0, 0, -Math.PI * 0.03);
    world.addBody(body);
    demo.addVisual(body);
  }

  void setupWorld(){
    final groundShape = cannon.Plane();
    final groundBody = cannon.Body(mass: 0);
    groundBody.addShape(groundShape);
    groundBody.quaternion.setFromEuler(-Math.PI / 2, 0, 0);
    demo.world.addBody(groundBody);
    demo.addVisual(groundBody);

    sceneBox();
    //sceneSpheres();
    demo.start();
  }
  @override
  Widget build(BuildContext context) {
    return demo.threeDart();
  }
}