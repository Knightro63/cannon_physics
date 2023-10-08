import 'package:flutter/material.dart';
import 'package:three_dart/three_dart.dart';
import '../src/demo.dart';
import 'package:cannon_physics/cannon_physics.dart' as cannon;

class Convex extends StatefulWidget {
  const Convex({
    Key? key,
    this.offset = const Offset(0,0),
    this.settings
  }) : super(key: key);

  final Offset offset;
  final DemoSettings? settings;

  @override
  _ConvexState createState() => _ConvexState();
}

class _ConvexState extends State<Convex> {
  late Demo demo;

  @override
  void initState() {
    demo = Demo(
      onSetupComplete: (){setState(() {});},
      settings: DemoSettings(
        gx: 0,
        gy: -30,
        gz: 0,
        k: 5e6,
        d: 3
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

  cannon.ConvexPolyhedron createTetra() {
    final vertices = [
      cannon.Vec3(0, 0, 0),
      cannon.Vec3(2, 0, 0),
      cannon.Vec3(0, 2, 0),
      cannon.Vec3(0, 0, 2),
    ];
    const offset = -0.35;
    for (int i = 0; i < vertices.length; i++) {
      final v = vertices[i];
      v.x += offset;
      v.y += offset;
      v.z += offset;
    }
    return cannon.ConvexPolyhedron(
      vertices: vertices,
      faces: [
        [0, 3, 2], // -x
        [0, 1, 3], // -y
        [0, 2, 1], // -z
        [1, 2, 3], // +xyz
      ],
    );
  }

  cannon.ConvexPolyhedron createBoxPolyhedron([double size = 1]) {
    final box = cannon.Box(cannon.Vec3(size, size, size));
    return box.convexPolyhedronRepresentation;
  }

  void setScene(){
    final world = demo.world;
    // Static ground plane
    final groundShape = cannon.Plane();
    final groundBody = cannon.Body(mass: 0 );
    groundBody.addShape(groundShape);
    groundBody.quaternion.setFromEuler(-Math.PI / 2, 0, 0);
    // groundBody.position.set(-10, 0, 0)
    world.addBody(groundBody);
    demo.addVisual(groundBody);
  }

  void convexWall(){
    setScene();
    final world = demo.world;

    const size = 1.0;
    const mass = 10.0;

    // ConvexPolyhedron box shape
    final convexShape = createBoxPolyhedron(size);
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        final boxbody = cannon.Body(mass:mass );
        boxbody.addShape(convexShape);
        boxbody.position.set(-(size * 2 * i + 0.01), size * 2 * j + size * 1.2, 0);
        world.addBody(boxbody);
        demo.addVisual(boxbody);
      }
    }
  }

  void convexOnConvex(){
    setScene();
    final world = demo.world;

    const size = 2.0;
    const mass = 10.0;

    final convexShape = createBoxPolyhedron(size);

    // ConvexPolyhedron box shape
    final boxBody1 = cannon.Body(mass:mass);
    boxBody1.addShape(convexShape);
    boxBody1.position.set(0, size + 1, 0);
    world.addBody(boxBody1);
    demo.addVisual(boxBody1);

    final boxBody2 = cannon.Body(mass:mass);
    boxBody2.addShape(convexShape);
    boxBody2.position.set(-1.5, size * 4 + 1, 0);
    world.addBody(boxBody2);
    demo.addVisual(boxBody2);
  }
  void various(){
    setScene();
    final world = demo.world;

    const size = 0.5;
    const mass = 10.0;

    // ConvexPolyhedron box shape
    final convexShape = createBoxPolyhedron(size);
    final boxbody = cannon.Body(mass:mass );
    boxbody.addShape(convexShape);
    boxbody.position.set(-1, size + 1, 0);
    world.addBody(boxbody);
    demo.addVisual(boxbody);

    // ConvexPolyhedron tetra shape
    final tetraShape = createTetra();
    final tetraBody = cannon.Body(mass:mass);
    tetraBody.addShape(tetraShape);
    tetraBody.position.set(-5, size + 1, -3);
    world.addBody(tetraBody);
    demo.addVisual(tetraBody);

    // The Cylinder is a ConvexPolyhedron under the hood
    const height = 2.0;
    const radius = 0.5;
    const detail = 20;

    final cylinderShape = cannon.Cylinder(
      radiusTop: radius, 
      radiusBottom: radius, 
      height: height, 
      numSegments: detail
    );
    final cylinderBody = cannon.Body(mass:mass);
    cylinderBody.addShape(cylinderShape);
    cylinderBody.position.set(0, size * 4 + 1, 0);
    cylinderBody.quaternion.setFromEuler(0, 0, Math.PI / 3);
    world.addBody(cylinderBody);
    demo.addVisual(cylinderBody);
  }

  void setupWorld(){
    demo.addScene('Convex Wall',convexWall);
    demo.addScene('Convex On Convex',convexOnConvex);
    demo.addScene('Various',various);
  }
  @override
  Widget build(BuildContext context) {
    return demo.threeDart();
  }
}