import 'package:flutter/material.dart';
import 'package:three_dart/three_dart.dart';
import '../src/demo.dart';
import 'package:cannon_physics/cannon_physics.dart' as cannon;

class Callback extends StatefulWidget {
  const Callback({
    Key? key,
    this.offset = const Offset(0,0),
    this.settings
  }) : super(key: key);

  final Offset offset;
  final DemoSettings? settings;

  @override
  _CallbackState createState() => _CallbackState();
}

class _CallbackState extends State<Callback> {
  late Demo demo;

  @override
  void initState() {
    demo = Demo(
      onSetupComplete: (){setState(() {});},
      settings: DemoSettings()
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

    final moonShape = cannon.Sphere(0.5);
    final moon = cannon.Body(
      mass: 5,
      position: cannon.Vec3(-5, 0, 0),
    );
    moon.addShape(moonShape);

    moon.velocity.set(0, 8, 0);
    moon.linearDamping = 0.0;

    final planetShape = cannon.Sphere(3.5);
    final planet = cannon.Body(mass: 0);
    planet.addShape(planetShape);

    world.addEventListener('preStep', (d){
      final moon_to_planet = cannon.Vec3();
      moon.position.negate(moon_to_planet);

      final distance = moon_to_planet.length();

      moon_to_planet.normalize();
      moon_to_planet.scale(1500 / Math.pow(distance, 2), moon.force);
    });

    world.addBody(moon);
    world.addBody(planet);

    demo.addVisual(moon);
    demo.addVisual(planet);
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