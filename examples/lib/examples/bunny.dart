import 'package:flutter/material.dart';
import 'dart:math' as math;
import '../src/demo.dart';
import 'package:cannon_physics/cannon_physics.dart' as cannon;
import '../src/conversion_utils.dart';
import 'package:vector_math/vector_math.dart' as vmath;

class Bunny extends StatefulWidget {
  const Bunny({
    Key? key,
    this.offset = const Offset(0,0),
    this.settings
  }) : super(key: key);

  final Offset offset;
  final DemoSettings? settings;

  @override
  _BunnyState createState() => _BunnyState();
}

class _BunnyState extends State<Bunny> {
  late Demo demo;

  @override
  void initState() {
    demo = Demo(
      onSetupComplete: (){setState(() {});},
      settings: DemoSettings(
        gx: 0,
        gy: -20,
        gz: 0,
        iterations: 20,
        k: 1e10,
        d: 10 
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
    final groundBody = cannon.Body(
      mass: 0,
      position: vmath.Vector3(0, -5, 0),
    );
    groundBody.addShape(groundShape);
    groundBody.quaternion.setFromEuler(-math.pi / 2, 0, 0);
    world.addBody(groundBody);
    demo.addVisual(groundBody);

    final bunnyBody = cannon.Body(mass: 1);

    for (int i = 0; i < bunny.length; i++) {
      final rawVertices = bunny[i]['vertices']!;
      final rawFaces = bunny[i]['faces']!;
      final rawOffset = bunny[i]['offset']!;

      // Get vertices
      final List<vmath.Vector3> vertices = [];
      for (int j = 0; j < rawVertices.length; j += 3) {
        vertices.add(vmath.Vector3(rawVertices[j], rawVertices[j + 1], rawVertices[j + 2]));
      }

      // Get faces
      final List<List<int>> faces = [];
      for (int j = 0; j < rawFaces.length; j += 3) {
        faces.add([rawFaces[j], rawFaces[j + 1], rawFaces[j + 2]]);
      }

      // Get offset
      final offset = vmath.Vector3(rawOffset[0], rawOffset[1], rawOffset[2]);
      // Construct polyhedron
      final bunnyPart = cannon.ConvexPolyhedron(vertices:vertices, faces:faces);

      // Add to compound
      bunnyBody.addShape(bunnyPart, offset);
    }

    // Create body
    bunnyBody.quaternion.setFromEuler(math.pi, 0, 0);
    world.addBody(bunnyBody);
    demo.addVisual(bunnyBody);
  }

  void setupWorld(){
    setScene();
  }
  @override
  Widget build(BuildContext context) {
    return demo.threeDart();
  }
}