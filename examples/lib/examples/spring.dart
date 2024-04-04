import 'package:flutter/material.dart';
import '../src/demo.dart';
import 'package:cannon_physics/cannon_physics.dart' as cannon;

class Spring extends StatefulWidget {
  const Spring({
    Key? key,
    this.offset = const Offset(0,0),
    this.settings
  }) : super(key: key);

  final Offset offset;
  final DemoSettings? settings;

  @override
  _SpringState createState() => _SpringState();
}

class _SpringState extends State<Spring> {
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
  void doubleSpringScene(){
    final world = demo.world;

    const size = 1.0;

    // Create a static sphere
    cannon.Sphere sphereShape = cannon.Sphere(0.1);
    cannon.Body sphereBody = cannon.Body( mass: 0 );
    sphereBody.addShape(sphereShape);
    world.addBody(sphereBody);
    demo.addVisual(sphereBody);

    // Create a box body
    cannon.Box boxShape = cannon.Box(cannon.Vec3(size, size, size * 0.3));
    cannon.Body boxBody = cannon.Body( mass: 5 );
    boxBody.addShape(boxShape);
    boxBody.position.set(size, -size, 0);
    world.addBody(boxBody);
    demo.addVisual(boxBody);

    cannon.Spring spring = cannon.Spring(
      boxBody, 
      sphereBody, 
      localAnchorA: cannon.Vec3(-size, size, 0),
      localAnchorB: cannon.Vec3(0, 0, 0),
      restLength: 0,
      stiffness: 50,
      damping: 1,
    );

    // Create a box body
    cannon.Box boxShape1 = cannon.Box(cannon.Vec3(size, size, size * 0.3));
    cannon.Body boxBody1 = cannon.Body( mass: 5 );
    boxBody1.addShape(boxShape1);
    boxBody1.position.set(size, -size*8, 0);
    world.addBody(boxBody1);
    demo.addVisual(boxBody1);

    cannon.Spring spring2 = cannon.Spring(
      boxBody1, 
      boxBody, 
      localAnchorA: cannon.Vec3(-size, size, 0),
      localAnchorB: cannon.Vec3(size, -size, 0),
      restLength: 0,
      stiffness: 50,
      damping: 1,
    );

    // Compute the force after each step
    world.addEventListener('postStep',(event) => {spring.applyForce(),spring2.applyForce()});
  }
  void singleSpringScene(){
    final world = demo.world;

    const size = 1.0;

    // Create a static sphere
    cannon.Sphere sphereShape = cannon.Sphere(0.1);
    cannon.Body sphereBody = cannon.Body( mass: 0 );
    sphereBody.addShape(sphereShape);
    world.addBody(sphereBody);
    demo.addVisual(sphereBody);

    // Create a box body
    cannon.Box boxShape = cannon.Box(cannon.Vec3(size, size, size * 0.3));
    cannon.Body boxBody = cannon.Body( mass: 5 );
    boxBody.addShape(boxShape);
    boxBody.position.set(size, -size, 0);
    world.addBody(boxBody);
    demo.addVisual(boxBody);

    cannon.Spring spring = cannon.Spring(
      boxBody, 
      sphereBody, 
      localAnchorA: cannon.Vec3(-size, size, 0),
      localAnchorB: cannon.Vec3(0, 0, 0),
      restLength: 0,
      stiffness: 50,
      damping: 1,
    );

    // Compute the force after each step
    world.addEventListener('postStep',(event) => {spring.applyForce()});
  }
  void setupWorld(){
    demo.addScene('Single',singleSpringScene);
    demo.addScene('Double',doubleSpringScene);
  }
  @override
  Widget build(BuildContext context) {
    return demo.threeDart();
  }
}