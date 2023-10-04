import 'package:flutter/material.dart';
import 'package:three_dart/three_dart.dart';
import '../src/demo.dart';
import 'package:cannon_physics/cannon_physics.dart' as cannon;

class ContainerCP extends StatefulWidget {
  const ContainerCP({
    Key? key,
    this.offset = const Offset(0,0),
    this.settings
  }) : super(key: key);

  final Offset offset;
  final DemoSettings? settings;

  @override
  _ContainerState createState() => _ContainerState();
}

class _ContainerState extends State<ContainerCP> {
  late Demo demo;

  @override
  void initState() {
    demo = Demo(
      onSetupComplete: (){setState(() {});},
      settings: DemoSettings(
        gx: 0,
        gy: -30,
        gz: 0,
        k: 1e11,
        d: 2,
        quatNormalizeSkip: 8,
        quatNormalizeFast: true,
        iterations: 10
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

  void createContainer(nx, ny, nz) {
    final world = demo.world;

    world.broadphase = cannon.SAPBroadphase(world);

    // Materials
    final stone = cannon.Material(name: 'stone');
    final stone_stone = cannon.ContactMaterial(
      stone, 
      stone,
      friction: 0.3,
      restitution: 0.2,
    );
    world.addContactMaterial(stone_stone);

    // Ground plane
    final groundShape = cannon.Plane();
    final groundBody = cannon.Body(mass: 0, material: stone);
    groundBody.addShape(groundShape);
    groundBody.quaternion.setFromEuler(-Math.PI / 2, 0, 0);
    world.addBody(groundBody);
    demo.addVisual(groundBody);

    // Plane -x
    final planeShapeXmin = cannon.Plane();
    final planeXmin = cannon.Body(mass: 0, material: stone);
    planeXmin.addShape(planeShapeXmin);
    planeXmin.quaternion.setFromEuler(0, Math.PI / 2, 0);
    planeXmin.position.set(-5, 0, 0);
    world.addBody(planeXmin);

    // Plane +x
    final planeShapeXmax = cannon.Plane();
    final planeXmax = cannon.Body( mass: 0, material: stone);
    planeXmax.addShape(planeShapeXmax);
    planeXmax.quaternion.setFromEuler(0, -Math.PI / 2, 0);
    planeXmax.position.set(5, 0, 0);
    world.addBody(planeXmax);

    // Plane -z
    final planeShapeZmin = cannon.Plane();
    final planeZmin = cannon.Body(mass: 0, material: stone);
    planeZmin.addShape(planeShapeZmin);
    planeZmin.quaternion.setFromEuler(0, 0, 0);
    planeZmin.position.set(0, 0, -5);
    world.addBody(planeZmin);

    // Plane +z
    final planeShapeZmax = cannon.Plane();
    final planeZmax = cannon.Body(mass: 0, material: stone );
    planeZmax.addShape(planeShapeZmax);
    planeZmax.quaternion.setFromEuler(0, Math.PI, 0);
    planeZmax.position.set(0, 0, 5);
    world.addBody(planeZmax);

    // Create spheres
    const randRange = 0.1;
    const heightOffset = 0;
    final sphereShape = cannon.Sphere(1); // Sharing shape saves memory

    world.allowSleep = true;
    for (int i = 0; i < nx; i++) {
      for (int j = 0; j < ny; j++) {
        for (int k = 0; k < nz; k++) {
          final sphereBody = cannon.Body(mass: 5, material: stone );
          sphereBody.addShape(sphereShape);
          sphereBody.position.set(
            -(i * 2 - nx * 0.5 + (Math.random() - 0.5) * randRange),
            1 + k * 2.1 + heightOffset,
            j * 2 - ny * 0.5 + (Math.random() - 0.5) * randRange
          );
          sphereBody.allowSleep = true;
          sphereBody.sleepSpeedLimit = 1;
          sphereBody.sleepTimeLimit = 5;

          world.addBody(sphereBody);
          demo.addVisual(sphereBody);
        }
      }
    }
  }

  void setupWorld(){
    createContainer(4, 4, 30);
  }
  @override
  Widget build(BuildContext context) {
    return demo.threeDart();
  }
}