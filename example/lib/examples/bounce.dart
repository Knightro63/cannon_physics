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

class Bounce extends StatefulWidget {
  const Bounce({
    Key? key,
    this.offset = const Offset(0,0),
    this.settings
  }) : super(key: key);

  final Offset offset;
  final DemoSettings? settings;

  @override
  _BounceState createState() => _BounceState();
}

class _BounceState extends State<Bounce> {
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

    // Static ground plane
    final groundMaterial = cannon.Material(name:'ground');
    final groundShape = cannon.Plane();
    final groundBody = cannon.Body(mass: 0, material: groundMaterial);
    groundBody.addShape(groundShape);
    groundBody.quaternion.setFromEuler(-Math.PI / 2, 0, 0);
    world.addBody(groundBody);
    demo.addVisual(groundBody);

    const double mass = 10;
    const double size = 1;
    const double height = 5;
    const double damping = 0.01;

    final sphereShape = cannon.Sphere(size);

    // Shape on plane
    final mat1 = cannon.Material();
    final shapeBody1 = cannon.Body(
      mass: mass,
      material: mat1,
      position: cannon.Vec3(-size * 3, height, size),
    );
    shapeBody1.addShape(sphereShape);
    shapeBody1.linearDamping = damping;
    world.addBody(shapeBody1);
    demo.addVisual(shapeBody1);

    final mat2 = cannon.Material();
    final shapeBody2 = cannon.Body(
      mass: mass,
      material: mat2,
      position: cannon.Vec3(0, height, size),
    );
    shapeBody2.addShape(sphereShape);
    shapeBody2.linearDamping = damping;
    world.addBody(shapeBody2);
    demo.addVisual(shapeBody2);

    final mat3 = cannon.Material();
    final shapeBody3 = cannon.Body(
      mass: mass,
      material: mat3,
      position: cannon.Vec3(size * 3, height, size),
    );
    shapeBody3.addShape(sphereShape);
    shapeBody3.linearDamping = damping;
    world.addBody(shapeBody3);
    demo.addVisual(shapeBody3);

    // Create contact material behaviour
    final mat1_ground = cannon.ContactMaterial(groundMaterial, mat1, friction: 0.0, restitution: 0.0 );
    final mat2_ground = cannon.ContactMaterial(groundMaterial, mat2, friction: 0.0, restitution: 0.7 );
    final mat3_ground = cannon.ContactMaterial(groundMaterial, mat3, friction: 0.0, restitution: 0.9 );

    world.addContactMaterial(mat1_ground);
    world.addContactMaterial(mat2_ground);
    world.addContactMaterial(mat3_ground);
  }

  void setupWorld(){
    setScene();
  }
  @override
  Widget build(BuildContext context) {
    return demo.threeDart();
  }
}