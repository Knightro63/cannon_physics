import 'package:cannon_physics_example/src/conversion_utils.dart';
import 'package:flutter/material.dart';
import 'package:three_dart/three_dart.dart';
import '../src/demo.dart';
import 'package:cannon_physics/cannon_physics.dart' as cannon;

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
    groundBody.quaternion.setFromEuler(-Math.PI / 2, 0, 0);
    world.addBody(groundBody);
    demo.addVisual(groundBody);

    int N = 50;
    const mass = 1.0;
    const size = 0.25;
    final boxShape = cannon.Box(cannon.Vec3(size, size, size));
    final torusGeometry = TorusKnotGeometry(size*2,size*2*0.4);
    final torusMaterial = MeshStandardMaterial({'color': 0x2b4c7f });
    final torusMesh = Mesh(torusGeometry, torusMaterial);
    final torus = ConversionUtils.geometryToShape(torusGeometry);

    for (int i = 0; i < N; i++) {
      final position = cannon.Vec3(
        (Math.random() * 2 - 1) * 2.5,
        Math.random() * 10,
        (Math.random() * 2 - 1) * 2.5
      );

      final body = cannon.Body(
        position: position,
        mass: mass,
      );
      if(i%5 == 0){
        body.addShape(torus);
        demo.addVisual(
          body,
          mesh:torusMesh,
          material: torusMaterial
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
    demo.start();
  }
  @override
  Widget build(BuildContext context) {
    return demo.threeDart();
  }
}