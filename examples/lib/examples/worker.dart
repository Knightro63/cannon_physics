import 'package:flutter/material.dart';
import '../src/demo.dart';
import 'dart:math' as math;
import 'package:cannon_physics/cannon_physics.dart' as cannon;
import 'package:vector_math/vector_math.dart' as vmath;

class Worker extends StatefulWidget {
  const Worker({
    Key? key,
    this.offset = const Offset(0,0),
    this.settings
  }) : super(key: key);

  final Offset offset;
  final DemoSettings? settings;

  @override
  _WorkerState createState() => _WorkerState();
}

class _WorkerState extends State<Worker> {
  late Demo demo;

  @override
  void initState() {
    demo = Demo(
      onSetupComplete: (){setState(() {});},
      settings: DemoSettings(
        gx: 0,
        gy: -10,
        gz: 0,
        tolerance: 0.001
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
    groundBody.quaternion.setFromEuler(-math.pi / 2, 0, 0);
    world.addBody(groundBody);
    demo.addVisual(groundBody);

    int N = 50;
    const mass = 1.0;
    const size = 0.25;
    final boxShape = cannon.Sphere(size);
    //final torusGeometry = TorusKnotGeometry(size,size*0.4,8);
    //final torusMaterial = MeshStandardMaterial({'color': 0x2b4c7f });
    //final torusMesh = Mesh(torusGeometry, torusMaterial);
    final torus = cannon.Particle();//ConversionUtils.geometryToShape(torusGeometry);

    final cylinderShape = cannon.Cylinder(
      radiusTop: size, 
      radiusBottom: size, 
      height: size * 2, 
      numSegments: 10
    );
    final cylinderBody = cannon.Body(mass:mass);
    cylinderBody.addShape(cylinderShape);
    cylinderBody.position.setValues(size * 2, size + 1, size * 2);
    world.addBody(cylinderBody);
    demo.addVisual(cylinderBody);

    for (int i = 0; i < N; i++) {
      final position = vmath.Vector3(
        (math.Random().nextDouble() * 2 - 1) * 2.5,
       math.Random().nextDouble() * 10,
        (math.Random().nextDouble() * 2 - 1) * 2.5
      );

      final body = cannon.Body(
        position: position,
        mass: mass,
      );
      body.linearDamping = 0.3;
      body.angularDamping = 0.3;
      if(i%2 == 0){
        body.addShape(torus);
        demo.addVisual(
          body,
          //mesh:torusMesh,
          //material: torusMaterial
        );
      }
      else{
        body.addShape(boxShape);
        demo.addVisual(body);
      }
      world.addBody(body);
    }
  }

  void setupWorld(){
    setScene();
  }
  @override
  Widget build(BuildContext context) {
    return demo.threeDart();
  }
}