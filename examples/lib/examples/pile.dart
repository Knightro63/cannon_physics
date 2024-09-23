import 'package:flutter/material.dart';
import 'dart:math' as math;
import '../src/demo.dart';
import 'package:cannon_physics/cannon_physics.dart' as cannon;
import 'package:vector_math/vector_math.dart' as vmath;

class Pile extends StatefulWidget {
  const Pile({
    Key? key,
    this.offset = const Offset(0,0),
    this.settings
  }) : super(key: key);

  final Offset offset;
  final DemoSettings? settings;

  @override
  _PileState createState() => _PileState();
}

class _PileState extends State<Pile> {
  late Demo demo;

  @override
  void initState() {
    demo = Demo(
      onSetupComplete: (){setState(() {});},
      settings: DemoSettings(
        gx: 0,
        gy: -50,
        gz: 0,
        iterations: 5,
        k: 5e6,
        d: 10,
        quatNormalizeFast: true,
        quatNormalizeSkip: 3
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
  void setScene() {
    final world = demo.world;

    final groundShape = cannon.Plane();
    final groundBody = cannon.Body(mass: 0 );
    groundBody.addShape(groundShape);
    groundBody.quaternion.setFromEuler(-math.pi / 2, 0, 0);
    world.addBody(groundBody);
    demo.addVisual(groundBody);

    final planeShapeXmin = cannon.Plane();
    final planeXmin = cannon.Body(mass: 0);
    planeXmin.addShape(planeShapeXmin);
    planeXmin.quaternion.setFromEuler(0, math.pi / 2, 0);
    planeXmin.position.setValues(-5, 0, 0);
    world.addBody(planeXmin);

    // Plane +x
    final planeShapeXmax = cannon.Plane();
    final planeXmax = cannon.Body(mass: 0 );
    planeXmax.addShape(planeShapeXmax);
    planeXmax.quaternion.setFromEuler(0, -math.pi / 2, 0);
    planeXmax.position.setValues(5, 0, 0);
    world.addBody(planeXmax);

    // Plane -z
    final planeShapeZmin = cannon.Plane();
    final planeZmin = cannon.Body(mass: 0);
    planeZmin.addShape(planeShapeZmin);
    planeZmin.quaternion.setFromEuler(0, 0, 0);
    planeZmin.position.setValues(0, 0, -5);
    world.addBody(planeZmin);

    // Plane +z
    final planeShapeZmax = cannon.Plane();
    final planeZmax = cannon.Body(mass: 0);
    planeZmax.addShape(planeShapeZmax);
    planeZmax.quaternion.setFromEuler(0, math.pi, 0);
    planeZmax.position.setValues(0, 0, 5);
    world.addBody(planeZmax);

    const size = 1.0;
    List<cannon.Body> bodies = [];
    int i = 0;

    demo.addAnimationEvent((dt){
      i++;
      final sphereShape = cannon.Sphere(size);
      final sphereBody = cannon.Body(
        mass: 5,
        position: vmath.Vector3(-size * 2 * math.sin(i), size * 2 * 7, size * 2 * math.cos(i)),
      );
      sphereBody.addShape(sphereShape);
      world.addBody(sphereBody);
      demo.addVisual(sphereBody);
      bodies.add(sphereBody);

      if (bodies.length > 80) {
        final bodyToKill = bodies.removeAt(0);
        demo.removeVisual(bodyToKill);
        world.removeBody(bodyToKill);
      }
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