import 'dart:math' as math;
import 'package:cannon_physics_example/src/conversion_utils.dart';
import 'package:flutter/material.dart';
import 'package:three_js/three_js.dart';
import 'package:three_js_geometry/three_js_geometry.dart';
import '../src/demo.dart';
import 'package:cannon_physics/cannon_physics.dart' as cannon;
import 'package:vector_math/vector_math.dart' as vmath;

class TriMesh extends StatefulWidget {
  const TriMesh({
    Key? key,
    this.offset = const Offset(0,0),
    this.settings
  }) : super(key: key);

  final Offset offset;
  final DemoSettings? settings;

  @override
  _TriMeshState createState() => _TriMeshState();
}

class _TriMeshState extends State<TriMesh> {
  late Demo demo;

  @override
  void initState() {
    demo = Demo(
      onSetupComplete: (){setState(() {});},
      settings: DemoSettings(
        gx: 0,
        gy: -10,
        gz: 0,
        k: 1e7,
        d:4
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
  void triMeshRay(){
    final world = demo.world;

    world.gravity.setValues(0, 0, 0);

    // Particle as marker for the raycast hit
    const N = 10;
    final particleShape = cannon.Particle();
    List<cannon.Body> particleBodies = [];
    for (int i = 0; i < N * N; i++) {
      final particleBody = cannon.Body(
        mass: 1,
        shape: particleShape,
        collisionResponse: false,
        angularDamping: 0
      );
      world.addBody(particleBody);
      demo.addVisual(particleBody);
      particleBodies.add(particleBody);
    }

    // Torus
    //final torusGeometry = TorusGeometry(4, 3.5, 16, 16);
    final torusGeometry = IcosahedronGeometry(2,2);
    final torusMaterial = MeshStandardMaterial.fromMap({'color': 0x2b4c7f });
    final torusMesh = Mesh(torusGeometry, torusMaterial);
    final torusShape = ConversionUtils.geometryToShape(torusGeometry);

    //final torusShape = cannon.Trimesh.createTorus(cannon.TorusGeometry(5, 3.5, 16,16));
    final torusBody = cannon.Body(mass: 1,angularDamping: 0 );
    torusBody.addShape(torusShape);
    torusBody.position.setValues(0.01, 0.01, 0.01);
    torusBody.quaternion.setFromEuler(-math.pi / 2, 0, 0);
    torusBody.angularVelocity.setValues(0, 0, 1);
    world.addBody(torusBody);
    demo.addVisual(torusBody, mesh: torusMesh, material: torusMaterial);

    // Do the raycasting
    final from = vmath.Vector3(-10, 0, 1);
    final to = vmath.Vector3(10, 0, 1);
    final result = cannon.RaycastResult();
    final raycastOptions = cannon.RayOptions();
    void postStepListener(event) {
      for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
          from.setValues(-10, i * 0.1, j * 0.1);
          to.setValues(10, i * 0.1, j * 0.1);
          result.reset();
          world.raycastClosest(from, to, raycastOptions, result);
          particleBodies[i * N + j].position.setFrom(result.hitPointWorld);
        }
      }
    }

    world.addEventListener('postStep', postStepListener);
  }
  void triMeshSphere(){
    final world = demo.world;

    final groundShape = cannon.Plane();
    final groundBody = cannon.Body(mass: 0);
    groundBody.addShape(groundShape);
    groundBody.quaternion.setFromEuler(-math.pi / 2, 0, 0);
    demo.world.addBody(groundBody);
    demo.addVisual(groundBody);

    // Sphere
    final sphereShape = cannon.Sphere();
    final sphereBody = cannon.Body(
      mass: 1,
      shape: sphereShape,
      position: vmath.Vector3(-3, 11, 3),
    );
    world.addBody(sphereBody);
    demo.addVisual(sphereBody);

    // Torus
    final torusGeometry = TorusGeometry(4, 3.5, 16, 16);
    //final torusGeometry = IcosahedronGeometry(2,2);
    final torusMaterial = MeshStandardMaterial();//{'color': 0x2b4c7f }
    final torusMesh = Mesh(torusGeometry, torusMaterial);
    final torusShape = ConversionUtils.geometryToShape(torusGeometry);

    final torusBody = cannon.Body(mass: 1 );
    torusBody.addShape(torusShape);
    torusBody.position.setValues(0, 4, 0);
    torusBody.quaternion.setFromEuler(math.pi / 2, 0, 0);
    torusBody.velocity.setValues(0, 1, 1);
    world.addBody(torusBody);
    demo.addVisual(torusBody, mesh: torusMesh, material: torusMaterial);
  }
  void triMeshSphere1(){
    final world = demo.world;

    final groundShape = cannon.Plane();
    final groundBody = cannon.Body(mass: 0);
    groundBody.addShape(groundShape);
    groundBody.quaternion.setFromEuler(-math.pi / 2, 0, 0);
    demo.world.addBody(groundBody);
    demo.addVisual(groundBody);

    // Sphere
    final sphereShape = cannon.Sphere();
    final sphereBody = cannon.Body(
      mass: 1,
      shape: sphereShape,
      position: vmath.Vector3(-3, 11, 3),
    );
    world.addBody(sphereBody);
    demo.addVisual(sphereBody);

    // Torus
    final torusGeometry = TorusGeometry(4, 3.5, 16, 16);
    //final torusGeometry = IcosahedronGeometry(2,2);
    final torusMaterial = MeshStandardMaterial();//{'color': 0x2b4c7f }
    final torusMesh = Mesh(torusGeometry, torusMaterial);
    final torusShape = ConversionUtils.geometryToShape(torusGeometry);

    final torusBody = cannon.ConvexPolyhedron.trimeshToPolyhedron(torusShape as cannon.Trimesh,cannon.Body(mass: 1 ));//cannon.Body(mass: 1 );
    torusBody.mass = 1;
    torusBody.addShape(torusShape);
    torusBody.position.setValues(0, 4, 0);
    torusBody.quaternion.setFromEuler(math.pi / 2, 0, 0);
    torusBody.velocity.setValues(0, 1, 1);
    world.addBody(torusBody);
    demo.addVisual(torusBody, mesh: torusMesh, material: torusMaterial);
  }
  void triMeshPartical(){
    final world = demo.world;

    final groundShape = cannon.Plane();
    final groundBody = cannon.Body(mass: 0);
    groundBody.addShape(groundShape);
    groundBody.quaternion.setFromEuler(-math.pi / 2, 0, 0);
    demo.world.addBody(groundBody);
    demo.addVisual(groundBody);

    final shape = cannon.Particle();
    final body = cannon.Body(
      mass: 1,
      shape: shape,
      position: vmath.Vector3(-3, 11, 4),
    );
    world.addBody(body);
    demo.addVisual(body);

    // Torus
    final torusGeometry = TorusGeometry(4, 3.5, 16, 16);
    //final torusGeometry = IcosahedronGeometry(2,2);
    final torusMaterial = MeshStandardMaterial({});//{'color': 0x2b4c7f }'wireframe':true
    final torusMesh = Mesh(torusGeometry, torusMaterial);
    final torusShape = ConversionUtils.geometryToShape(torusGeometry);

    final torusBody = cannon.ConvexPolyhedron.trimeshToPolyhedron(torusShape as cannon.Trimesh,cannon.Body(mass: 1000 ));//
    //final torusBody = cannon.Body(mass: 1 );
    torusBody.addShape(torusShape);
    torusBody.position.setValues(0, 4, 0);
    torusBody.quaternion.setFromEuler(math.pi / 2, 0, 0);
    torusBody.velocity.setValues(0, 1, 1);
    world.addBody(torusBody);
    demo.addVisual(torusBody, mesh: torusMesh, material: torusMaterial);
  }
  void triMeshTriMesh(){
    final world = demo.world;

    final groundShape = cannon.Plane();
    final groundBody = cannon.Body(mass: 0);
    groundBody.addShape(groundShape);
    groundBody.quaternion.setFromEuler(-math.pi / 2, 0, 0);
    demo.world.addBody(groundBody);
    demo.addVisual(groundBody);

   final torusGeometry1 = TorusGeometry();
    //final torusGeometry = IcosahedronGeometry(2,2);
    final torusMaterial1 = MeshStandardMaterial.fromMap({'color': 0x2b4c7f});//{'color': 0x2b4c7f }'wireframe':true
    final torusMesh1 = Mesh(torusGeometry1, torusMaterial1);
    final shape = ConversionUtils.geometryToShape(torusGeometry1);
    final body = cannon.Body(
      mass: 1,
      shape: shape,
      position: vmath.Vector3(-3, 11, 4),
    );
    world.addBody(body);
    demo.addVisual(body, mesh: torusMesh1, material: torusMaterial1);

    // Torus
    final torusGeometry = TorusGeometry(4, 3.5, 16, 16);
    //final torusGeometry = IcosahedronGeometry(2,2);
    final torusMaterial = MeshStandardMaterial({});//{'color': 0x2b4c7f }'wireframe':true
    final torusMesh = Mesh(torusGeometry, torusMaterial);
    final torusShape = ConversionUtils.geometryToShape(torusGeometry);

    //final torusBody = cannon.ConvexPolyhedron.trimeshToPolyhedron(torusShape as cannon.Trimesh,cannon.Body(mass: 1000 ));//
    final torusBody = cannon.Body(mass: 1 );
    torusBody.addShape(torusShape);
    torusBody.position.setValues(0, 4, 0);
    torusBody.quaternion.setFromEuler(math.pi / 2, 0, 0);
    torusBody.velocity.setValues(0, 1, 1);
    world.addBody(torusBody);
    demo.addVisual(torusBody, mesh: torusMesh, material: torusMaterial);
  }
  void triMeshBox(){
    final world = demo.world;

    final groundShape = cannon.Plane();
    final groundBody = cannon.Body(mass: 0);
    groundBody.addShape(groundShape);
    groundBody.quaternion.setFromEuler(-math.pi / 2, 0, 0);
    demo.world.addBody(groundBody);
    demo.addVisual(groundBody);

    final shape = cannon.Box(vmath.Vector3( 0.5,0.5,0.5));
    final body = cannon.Body(
      mass: 1,
      shape: shape,
      position: vmath.Vector3(-3, 11, 0),
    );
    world.addBody(body);
    demo.addVisual(body);

    // Torus
    final torusGeometry = TorusGeometry(4, 3.5, 16, 16);
    //final torusGeometry = IcosahedronGeometry(2,2);
    final torusMaterial = MeshStandardMaterial();//{'color': 0x2b4c7f }
    final torusMesh = Mesh(torusGeometry, torusMaterial);
    final torusShape = ConversionUtils.geometryToShape(torusGeometry);

    final torusBody = cannon.ConvexPolyhedron.trimeshToPolyhedron(torusShape as cannon.Trimesh,cannon.Body(mass: 1 ));//
    torusBody.mass = 1;
    torusBody.addShape(torusShape);
    torusBody.position.setValues(0, 4, 0);
    torusBody.quaternion.setFromEuler(math.pi / 2, 0, 0);
    torusBody.velocity.setValues(0, 1, 1);
    world.addBody(torusBody);
    demo.addVisual(torusBody, mesh: torusMesh, material: torusMaterial);
  }
  void triMeshCylinder(){
    final world = demo.world;

    final groundShape = cannon.Plane();
    final groundBody = cannon.Body(mass: 0);
    groundBody.addShape(groundShape);
    groundBody.quaternion.setFromEuler(-math.pi / 2, 0, 0);
    demo.world.addBody(groundBody);
    demo.addVisual(groundBody);

    final shape = cannon.Cylinder(radiusTop: 0.5,radiusBottom:0.5);
    final body = cannon.Body(
      mass: 1,
      shape: shape,
      position: vmath.Vector3(0, 11, 3),
    );
    world.addBody(body);
    demo.addVisual(body);

    // Torus
    final torusGeometry = TorusGeometry(4, 3.5, 16, 16);
    //final torusGeometry = IcosahedronGeometry(2,2);
    final torusMaterial = MeshStandardMaterial();//{'color': 0x2b4c7f }
    final torusMesh = Mesh(torusGeometry, torusMaterial);
    final torusShape = ConversionUtils.geometryToShape(torusGeometry);

    final torusBody = cannon.ConvexPolyhedron.trimeshToPolyhedron(torusShape as cannon.Trimesh,cannon.Body(mass: 1 ));//cannon.Body(mass: 1 );
    torusBody.mass = 1;
    torusBody.addShape(torusShape);
    torusBody.position.setValues(0, 4, 0);
    torusBody.quaternion.setFromEuler(math.pi / 2, 0, 0);
    torusBody.velocity.setValues(0, 1, 1);
    world.addBody(torusBody);
    demo.addVisual(torusBody, mesh: torusMesh, material: torusMaterial);
  }
  void setupWorld(){
    demo.addScene('Sphere',triMeshSphere);
    demo.addScene('Sphere Convex',triMeshSphere1);    
    demo.addScene('Box',triMeshBox);
    demo.addScene('Cylinder',triMeshCylinder);
    demo.addScene('Partical',triMeshPartical);
    demo.addScene('TriMesh',triMeshTriMesh);
    demo.addScene('Ray',triMeshRay);
  }
  @override
  Widget build(BuildContext context) {
    return demo.threeDart();
  }
}