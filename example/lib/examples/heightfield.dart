import 'package:flutter/material.dart';
import 'package:three_dart/three_dart.dart';
import '../src/demo.dart';
import 'package:cannon_physics/cannon_physics.dart' as cannon;

class Heightfield extends StatefulWidget {
  const Heightfield({
    Key? key,
    this.offset = const Offset(0,0),
    this.settings
  }) : super(key: key);

  final Offset offset;
  final DemoSettings? settings;

  @override
  _HeightfieldState createState() => _HeightfieldState();
}

class _HeightfieldState extends State<Heightfield> {
  late Demo demo;

  @override
  void initState() {
    demo = Demo(
      onSetupComplete: (){setState(() {});},
      settings: DemoSettings(
        gx: 0,
        gy: 0,
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

    // Create a matrix of height values
    List<List<double>> matrix = [];
    const sizeX = 15;
    const sizeZ = 15;
    for (int i = 0; i < sizeX; i++) {
      matrix.add([]);
      for (int j = 0; j < sizeZ; j++) {
        if (i == 0 || i == sizeX - 1 || j == 0 || j == sizeZ - 1) {
          const height = 3.0;
          matrix[i].add(height);
          continue;
        }

        final height = Math.cos((i / sizeX) * Math.PI * 2) * Math.cos((j / sizeZ) * Math.PI * 2) + 2;
        matrix[i].add(height);
      }
    }

    // Create the heightfield
    final heightfieldShape = cannon.Heightfield(
      matrix,
      elementSize: 1,
    );
    final heightfieldBody = cannon.Body(mass: 0);
    heightfieldBody.addShape(heightfieldShape);
    heightfieldBody.position.set(
      -((sizeX - 1) * heightfieldShape.elementSize) / 2,
      -4,
      ((sizeZ - 1) * heightfieldShape.elementSize) / 2
    );
    heightfieldBody.quaternion.setFromEuler(-Math.PI / 2, 0, 0);
    world.addBody(heightfieldBody);
    demo.addVisual(heightfieldBody);

    // Add spheres
    const mass = 1.0;
    for (int i = 0; i < sizeX - 1; i++) {
      for (int j = 0; j < sizeZ - 1; j++) {
        if (i == 0 || i >= sizeX - 2 || j == 0 || j >= sizeZ - 2) {
          continue;
        }

        final sphereShape = cannon.Sphere(0.1);
        final sphereBody = cannon.Body(mass:mass );
        sphereBody.addShape(sphereShape);
        sphereBody.position.set(i + 0.25, 3, -j + 0.25);
        sphereBody.position.vadd(heightfieldBody.position, sphereBody.position);
        world.addBody(sphereBody);
        demo.addVisual(sphereBody);
      }
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