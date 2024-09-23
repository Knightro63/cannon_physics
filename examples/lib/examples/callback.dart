import 'package:flutter/material.dart';
import 'dart:math' as math;
import '../src/demo.dart';
import 'package:cannon_physics/cannon_physics.dart' as cannon;
import 'package:vector_math/vector_math.dart' as vmath;

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
      position: vmath.Vector3(-5, 0, 0),
    );
    moon.addShape(moonShape);

    moon.velocity.setValues(0, 8, 0);
    moon.linearDamping = 0.0;

    final planetShape = cannon.Sphere(3.5);
    final planet = cannon.Body(mass: 0);
    planet.addShape(planetShape);

    world.addEventListener('preStep', (d){
      final moon_to_planet = vmath.Vector3.zero();
      moon_to_planet..setFrom(moon.position)..negate();

      final distance = moon_to_planet.length;

      moon_to_planet.normalize();
      moon_to_planet.scale2(1500 / math.pow(distance, 2), moon.force);
    });

    world.addBody(moon);
    world.addBody(planet);

    demo.addVisual(moon);
    demo.addVisual(planet);
  }

  void setupWorld(){
    setScene();
  }
  @override
  Widget build(BuildContext context) {
    return demo.threeDart();
  }
}