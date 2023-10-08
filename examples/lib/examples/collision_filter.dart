import 'package:flutter/material.dart';
import '../src/demo.dart';
import 'package:cannon_physics/cannon_physics.dart' as cannon;

class CollisionFilter extends StatefulWidget {
  const CollisionFilter({
    Key? key,
    this.offset = const Offset(0,0),
    this.settings
  }) : super(key: key);

  final Offset offset;
  final DemoSettings? settings;

  @override
  _CollisionFilterState createState() => _CollisionFilterState();
}

class _CollisionFilterState extends State<CollisionFilter> {
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
    // Max solver iterations: Use more for better force propagation, but keep in mind that it's not very computationally cheap!
    world.solver.iterations = 5;

    // Collision filter groups - must be powers of 2!
    const g1 = 1;
    const g2 = 2;
    const g3 = 4;

    const size = 1.0;
    const mass = 1.0;

    // Sphere
    final sphereShape = cannon.Sphere(size);
    final sphereBody = cannon.Body(
      mass: mass,
      position: cannon.Vec3(-5, 0, 0),
      velocity: cannon.Vec3(5, 0, 0),
      collisionFilterGroup: g1, // Put the sphere in group 1
      collisionFilterMask: g2 | g3, // It can only collide with group 2 and 3
      shape: sphereShape,
    );

    // Box
    final boxBody = cannon.Body(
      mass: mass,
      shape: cannon.Box(cannon.Vec3(size, size, size)),
      collisionFilterGroup: g2, // Put the box in group 2
      collisionFilterMask: g1, // It can only collide with group 1 (the sphere)
    );

    // Cylinder
    final cylinderShape = cannon.Cylinder(
      radiusTop: size, 
      radiusBottom: size, 
      height: size * 2.2, 
      numSegments: 10
    );
    final cylinderBody = cannon.Body(
      mass: mass,
      shape: cylinderShape,
      position: cannon.Vec3(5, 0, 0),
      collisionFilterGroup: g3, // Put the cylinder in group 3
      collisionFilterMask: g1, // It can only collide with group 1 (the sphere)
    );

    // Add everything to the world and demo
    world.addBody(sphereBody);
    world.addBody(boxBody);
    world.addBody(cylinderBody);

    demo.addVisual(sphereBody);
    demo.addVisual(boxBody);
    demo.addVisual(cylinderBody);
  }

  void setupWorld(){
    setScene();
  }
  @override
  Widget build(BuildContext context) {
    return demo.threeDart();
  }
}