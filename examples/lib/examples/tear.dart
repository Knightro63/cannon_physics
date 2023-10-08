import 'package:flutter/material.dart';
import 'package:three_dart/three_dart.dart';
import '../src/demo.dart';
import 'package:cannon_physics/cannon_physics.dart' as cannon;

class Tear extends StatefulWidget {
  const Tear({
    Key? key,
    this.offset = const Offset(0,0),
    this.settings
  }) : super(key: key);

  final Offset offset;
  final DemoSettings? settings;

  @override
  _TearState createState() => _TearState();
}

class _TearState extends State<Tear> {
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

    const size = 0.45;
    double mass = 1;

    // The number of chain links
    const N = 15;
    // The distance constraint between those links
    const distance = size * 2 + 0.12;

    // To be able to propagate force throw the chain of N spheres, we need at least N solver iterations.
    world.solver.iterations = N;

    cannon.Sphere sphereShape = cannon.Sphere(size);

    List<cannon.Constraint> constraints = [];
    cannon.Body? lastBody;
    for (int i = 0; i < N; i++) {
      // First body is static (mass = 0) to support the other bodies
      cannon.Body sphereBody = cannon.Body(mass: (i == 0 ? 0 : mass));
      sphereBody.addShape(sphereShape);
      sphereBody.position.set(0, (N - i) * distance - 9, 0);
      world.addBody(sphereBody);
      demo.addVisual(sphereBody);

      // Connect this body to the last one added
      if (lastBody != null) {
        cannon.DistanceConstraint constraint = cannon.DistanceConstraint(sphereBody, lastBody, distance);
        world.addConstraint(constraint);
        constraints.add(constraint);
      }

      // Keep track of the last added body
      lastBody = sphereBody;
    }

    world.addEventListener('postStep',(event){
      for (int i = constraints.length - 1; i >= 0; i--) {
        // The multiplier is proportional to how much force that is added to the bodies by the constraint.
        // If this exceeds a limit we remove the constraint.
        num multiplier = Math.abs(constraints[i].equations[0].multiplier);
        if (multiplier > 1000) {
          world.removeConstraint(constraints[i]);
        }
      }
    });

    // Throw a body on the chain to break it!
    cannon.Body sphereBody = cannon.Body( mass: mass * 2 );
    sphereBody.addShape(sphereShape);
    sphereBody.position.set(-20, 3, 0);
    sphereBody.velocity.x = 30;
    world.addBody(sphereBody);
    demo.addVisual(sphereBody);
  }

  void setupWorld(){
    setScene();
  }
  @override
  Widget build(BuildContext context) {
    return demo.threeDart();
  }
}