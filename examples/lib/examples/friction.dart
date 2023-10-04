import 'package:flutter/material.dart';
import 'package:three_dart/three_dart.dart';
import '../src/demo.dart';
import 'package:cannon_physics/cannon_physics.dart' as cannon;

class Friction extends StatefulWidget {
  const Friction({
    Key? key,
    this.offset = const Offset(0,0),
    this.settings
  }) : super(key: key);

  final Offset offset;
  final DemoSettings? settings;

  @override
  _FrictionState createState() => _FrictionState();
}

class _FrictionState extends State<Friction> {
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
  void setScene(){
    final world = demo.world;

    const size = 1.0;

    // Static ground plane
    final groundMaterial = cannon.Material(name: 'ground');
    final groundShape = cannon.Plane();
    final groundBody = cannon.Body(mass: 0, material: groundMaterial);
    groundBody.addShape(groundShape);
    groundBody.quaternion.setFromEuler(-Math.PI / 2, 0, 0);
    world.addBody(groundBody);
    demo.addVisual(groundBody);

    // Create a slippery material (friction coefficient = 0.0)
    final slipperyMaterial = cannon.Material(name: 'slippery');

    // Create slippery box
    final boxShape = cannon.Box(cannon.Vec3(size, size, size));
    final boxBody1 = cannon.Body( mass: 1, material: slipperyMaterial );
    boxBody1.addShape(boxShape);
    boxBody1.position.set(0, 5, 0);
    world.addBody(boxBody1);
    demo.addVisual(boxBody1);

    // Create box made of groundMaterial
    final boxBody2 = cannon.Body(mass: 10, material: groundMaterial);
    boxBody2.addShape(boxShape);
    boxBody2.position.set(-size * 4, 5, 0);
    world.addBody(boxBody2);
    demo.addVisual(boxBody2);

    // Adjust constraint equation parameters for ground/ground contact
    final ground_ground = cannon.ContactMaterial(groundMaterial, groundMaterial,
      friction: 0.4,
      restitution: 0.3,
      contactEquationStiffness: 1e8,
      contactEquationRelaxation: 3,
      frictionEquationStiffness: 1e8,
      frictionEquationRelaxation: 3,
    );

    // Add contact material to the world
    world.addContactMaterial(ground_ground);

    // The ContactMaterial defines what happens when two materials meet.
    // In this case we want friction coefficient = 0.0 when the slippery material touches ground.
    final slippery_ground = cannon.ContactMaterial(groundMaterial, slipperyMaterial,
      friction: 0,
      restitution: 0.3,
      contactEquationStiffness: 1e8,
      contactEquationRelaxation: 3,
    );

    // We must add the contact materials to the world
    world.addContactMaterial(slippery_ground);

  }

  void setupWorld(){
    setScene();
    demo.start();
  }
  @override
  Widget build(BuildContext context) {
    return demo.threeDart();
  }
}