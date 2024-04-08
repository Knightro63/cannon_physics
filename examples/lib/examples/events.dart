import 'package:flutter/material.dart';
import 'package:three_dart/three_dart.dart';
import '../src/demo.dart';
import 'package:cannon_physics/cannon_physics.dart' as cannon;

class Events extends StatefulWidget {
  const Events({
    Key? key,
    this.offset = const Offset(0,0),
    this.settings
  }) : super(key: key);

  final Offset offset;
  final DemoSettings? settings;

  @override
  _EventsState createState() => _EventsState();
}

class _EventsState extends State<Events> {
  late Demo demo;

  @override
  void initState() {
    demo = Demo(
      onSetupComplete: (){setState(() {});},
      settings: DemoSettings(
        gx: 0,
        gy: -20,
        gz: 0,
        k: 5e7,
        d: 4
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
    groundBody.quaternion.setFromEuler(-Math.PI / 2, 0, 0);
    world.addBody(groundBody);
    demo.addVisual(groundBody);

    const size = 1.0;

    // Sphere
    final sphere = cannon.Sphere(size);
    final sphereBody = cannon.Body(mass: 30);
    sphereBody.addShape(sphere);
    sphereBody.position.setValues(0, size * 6, 0);
    world.addBody(sphereBody);
    demo.addVisual(sphereBody);

    // When a body collides with another body, they both dispatch the "collide" event.
    sphereBody.addEventListener('collide', (event){
      print('The sphere just collided with the ground!');
      print('Collided with body: ${event.body}');
      print('Contact between bodies: ${event.contact}');
    });
  }

  void setupWorld(){
    setScene();
  }
  @override
  Widget build(BuildContext context) {
    return demo.threeDart();
  }
}