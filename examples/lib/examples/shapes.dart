import 'package:flutter/material.dart';
import 'package:three_dart/three_dart.dart';
import '../src/demo.dart';
import 'package:cannon_physics/cannon_physics.dart' as cannon;
import 'package:vector_math/vector_math.dart' as vmath;

class Shapes extends StatefulWidget {
  const Shapes({
    Key? key,
    this.offset = const Offset(0,0),
    this.settings
  }) : super(key: key);

  final Offset offset;
  final DemoSettings? settings;

  @override
  _ShapesState createState() => _ShapesState();
}

class _ShapesState extends State<Shapes> {
  late Demo demo;

  @override
  void initState() {
    demo = Demo(
      onSetupComplete: (){setState(() {});},
      settings: DemoSettings(
        gx: 0,
        gy: -30,
        gz: 0,
        iterations: 17,
        k: 1e6,
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
      vmath.Vector3(0, 0, 0),
      vmath.Vector3(2, 0, 0),
      vmath.Vector3(0, 2, 0),
      vmath.Vector3(0, 0, 2),
    ];
    const offset = -0.35;
    for (int i = 0; i < vertices.length; i++) {
      final v = vertices[i];
      v.x += offset;
      v.y += offset;
      v.z += offset;
    }
    return cannon.ConvexPolyhedron(
      vertices:vertices,
      faces: [
        [0, 3, 2], // -x
        [0, 1, 3], // -y
        [0, 2, 1], // -z
        [1, 2, 3], // +xyz
      ],
    );
  }
  void setScene(){
    final world = demo.world;

    final groundShape = cannon.Plane();
    final groundBody = cannon.Body(mass: 0 );
    groundBody.addShape(groundShape);
    groundBody.quaternion.setFromEuler(-Math.PI / 2, 0, 0);
    world.addBody(groundBody);
    demo.addVisual(groundBody);

    const mass = 1.0;
    const size = 1.0;

    // Sphere shape
    final sphereShape = cannon.Sphere(size);
    final sphereBody = cannon.Body(mass:mass);
    sphereBody.addShape(sphereShape);
    sphereBody.position.setValues(-size * 2, size + 1, size * 2);
    world.addBody(sphereBody);
    demo.addVisual(sphereBody);

    // Cylinder shape
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

    // Cylinder shape 2
    final cylinderShape2 = cannon.Cylinder(
      radiusTop: size, 
      radiusBottom: size, 
      height: size * 2, 
      numSegments: 10
    );
    final cylinderBody2 = cannon.Body(mass:mass);
    cylinderBody2.addShape(cylinderShape2);
    cylinderBody2.position.setValues(size * 2, size * 4 + 1, size * 2);
    cylinderBody2.quaternion.setFromEuler(Math.PI / 2, Math.PI / 2, 0);
    world.addBody(cylinderBody2);
    demo.addVisual(cylinderBody2);

    // Box shape
    final boxShape = cannon.Box(vmath.Vector3(size, size, size));
    final boxBody = cannon.Body(mass:mass);
    boxBody.addShape(boxShape);
    boxBody.position.setValues(size * 2, size + 1, -size * 2);
    world.addBody(boxBody);
    demo.addVisual(boxBody);

    // Particle - not a shape but still here to show how to use it.
    final particle = cannon.Body(mass:mass);
    particle.addShape(cannon.Particle());
    particle.position.setValues(size * 2, size + 1, size * 4);
    world.addBody(particle);
    demo.addVisual(particle);

    final particle2 = cannon.Body(mass:mass);
    particle2.addShape(cannon.Particle());
    particle2.position.setValues(size *2 , size *12, -size-0.5);
    world.addBody(particle2);
    demo.addVisual(particle2);
    // Compound
    final compoundBody = cannon.Body(mass:mass);
    final shape = cannon.Box(vmath.Vector3(size * 0.5, size * 0.5, size * 0.5));
    compoundBody.addShape(shape, vmath.Vector3(0, size, 0));
    compoundBody.addShape(shape, vmath.Vector3(0, 0, 0));
    compoundBody.addShape(shape, vmath.Vector3(0, -size, 0));
    compoundBody.addShape(shape, vmath.Vector3(size, -size, 0));
    compoundBody.position.setValues(size * 4, size + 1, size * 4);
    world.addBody(compoundBody);
    demo.addVisual(compoundBody);

    // ConvexPolyhedron tetra shape
    final polyhedronShape = createTetra();
    final polyhedronBody = cannon.Body(mass:mass);
    polyhedronBody.addShape(polyhedronShape);
    polyhedronBody.position.setValues(-size * 2, size + 1, -size * 2);
    world.addBody(polyhedronBody);
    demo.addVisual(polyhedronBody);
  }

  void setupWorld(){
    setScene();
  }
  @override
  Widget build(BuildContext context) {
    return demo.threeDart();
  }
}