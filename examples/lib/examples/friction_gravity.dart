import 'package:flutter/material.dart';
import 'dart:math' as math;
import '../src/demo.dart';
import 'package:cannon_physics/cannon_physics.dart' as cannon;
import 'package:vector_math/vector_math.dart' as vmath;

class FrictionGravity extends StatefulWidget {
  const FrictionGravity({
    Key? key,
    this.offset = const Offset(0,0),
    this.settings
  }) : super(key: key);

  final Offset offset;
  final DemoSettings? settings;

  @override
  _FrictionGravityState createState() => _FrictionGravityState();
}

class _FrictionGravityState extends State<FrictionGravity> {
  late Demo demo;

  @override
  void initState() {
    demo = Demo(
      onSetupComplete: (){setState(() {});},
      settings: DemoSettings(
        gx: 0,
        gy: 0,
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
  void setScene(cannon.World world){
    late cannon.Body boxBody1;
    late cannon.Body boxBody2;
    const size = 1.0;

    // Static ground plane
    final groundMaterial = cannon.Material(name: 'ground');
    final groundShape = cannon.Plane();
    final groundBody = cannon.Body(mass: 0, material: groundMaterial);
    groundBody.addShape(groundShape);
    groundBody.quaternion.setFromEuler(-math.pi / 2, 0, 0);
    world.addBody(groundBody);
    demo.addVisual(groundBody);

    // Create a slippery material (friction coefficient = 0.0)
    final slipperyMaterial = cannon.Material(name: 'slippery');

    // Create slippery box
    final boxShape = cannon.Box(vmath.Vector3(size, size, size));
    boxBody1 = cannon.Body(mass: 1, material: slipperyMaterial);
    boxBody1.addShape(boxShape);
    boxBody1.position.setValues(0, 5, 0);
    world.addBody(boxBody1);
    demo.addVisual(boxBody1);

    // Create box made of groundMaterial
    boxBody2 = cannon.Body(mass: 10, material: groundMaterial);
    boxBody2.addShape(boxShape);
    boxBody2.position.setValues(-size * 4, 5, 0);
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
      //frictionEquationRegularizationTime: 3,
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

    final gravityForce = vmath.Vector3(3, -60, 0);

    demo.addAnimationEvent((dt){
      boxBody1.applyForce(gravityForce);
      boxBody2.applyForce(gravityForce);
    });
  }

  void setupWorld(){
    final world = demo.world;
    world.gravity.setValues(0, 0, 0);
    world.frictionGravity = vmath.Vector3(3, -60, 0);

    setScene(world);
  }
  @override
  Widget build(BuildContext context) {
    return demo.threeDart();
  }
}