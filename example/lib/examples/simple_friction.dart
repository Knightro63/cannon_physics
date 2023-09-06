import 'package:flutter/material.dart';
import 'package:three_dart/three_dart.dart';
import '../src/demo.dart';
import 'package:cannon_physics/cannon_physics.dart' as cannon;

class SimpleFriction extends StatefulWidget {
  const SimpleFriction({
    Key? key,
    this.offset = const Offset(0,0),
    this.settings
  }) : super(key: key);

  final Offset offset;
  final DemoSettings? settings;

  @override
  _SimpleFrictionState createState() => _SimpleFrictionState();
}

class _SimpleFrictionState extends State<SimpleFriction> {
  late Demo demo;

  @override
  void initState() {
    demo = Demo(
      onSetupComplete: (){setState(() {});},
      settings: DemoSettings(
        gx: 3,
        gy: -60,
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
  void friction(){
    final world = demo.world;

    const size = 1.0;

    // Static ground plane
    final groundMaterial = cannon.Material(name: 'ground');
    groundMaterial.friction = 0.3;
    final groundShape = cannon.Plane();
    final groundBody = cannon.Body(mass: 0, material: groundMaterial);
    groundBody.addShape(groundShape);
    groundBody.quaternion.setFromEuler(-Math.PI / 2, 0, 0);
    world.addBody(groundBody);
    demo.addVisual(groundBody);

    // Create a slippery material (friction coefficient = 0.0)
    final slipperyMaterial = cannon.Material(name: 'slippery');
    slipperyMaterial.friction = 0;

    // Create slippery box
    final shape = cannon.Box(cannon.Vec3(size, size, size));
    final boxBody1 = cannon.Body(mass: 1, material: slipperyMaterial);
    boxBody1.addShape(shape);
    boxBody1.position.set(0, 5, 0);
    world.addBody(boxBody1);
    demo.addVisual(boxBody1);

    // Create box made of groundMaterial
    final boxBody2 = cannon.Body(mass: 10, material: groundMaterial);
    boxBody2.addShape(shape);
    boxBody2.position.set(-size * 4, 5, 0);
    world.addBody(boxBody2);
    demo.addVisual(boxBody2);
  }
  void perShape(){
    final world = demo.world;

    const size = 1.0;

    // Static ground plane
    final groundMaterial = cannon.Material(name: 'ground');
    groundMaterial.friction = 0.3;
    final groundShape = cannon.Plane();
    groundShape.material = groundMaterial;
    final groundBody = cannon.Body(mass: 0);
    groundBody.addShape(groundShape);
    groundBody.quaternion.setFromEuler(-Math.PI / 2, 0, 0);
    world.addBody(groundBody);
    demo.addVisual(groundBody);

    // Create a slippery material (friction coefficient = 0.0)
    final slipperyMaterial = cannon.Material(name: 'slippery');
    slipperyMaterial.friction = 0;

    // Create slippery box - will slide on the plane
    final shape1 = cannon.Box(cannon.Vec3(size, size, size));
    shape1.material = slipperyMaterial;
    final boxBody1 = cannon.Body(mass: 1);
    boxBody1.addShape(shape1);
    boxBody1.position.set(0, 5, 0);
    world.addBody(boxBody1);
    demo.addVisual(boxBody1);

    // Create box made of groundMaterial - will not slide on the plane
    final shape2 = cannon.Box(cannon.Vec3(size, size, size));
    shape2.material = groundMaterial;
    final boxBody2 = cannon.Body(mass: 10);
    boxBody2.addShape(shape2);
    boxBody2.position.set(-size * 4, 5, 0);
    world.addBody(boxBody2);
    demo.addVisual(boxBody2);
  }

  void setupWorld(){
    friction();
    demo.start();
  }
  @override
  Widget build(BuildContext context) {
    return demo.threeDart();
  }
}