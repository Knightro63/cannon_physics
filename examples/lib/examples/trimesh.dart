import 'dart:math' as math;
import 'package:cannon_physics_example/src/conversion_utils.dart';
import 'package:flutter/material.dart';
import 'package:three_dart/three_dart.dart';
import '../src/demo.dart';
import 'package:cannon_physics/cannon_physics.dart' as cannon;

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

    world.gravity.set(0, 0, 0);

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
    final torusGeometry = TorusGeometry(4, 3.5, 16, 16);
    //final torusGeometry = IcosahedronGeometry(2,2);
    final torusMaterial = MeshStandardMaterial({'wireframe':true});//{'color': 0x2b4c7f }
    final torusMesh = Mesh(torusGeometry, torusMaterial);
    final torusShape = ConversionUtils.geometryToShape(torusGeometry);

    //final torusShape = cannon.Trimesh.createTorus(cannon.TorusGeometry(5, 3.5, 16,16));
    final torusBody = cannon.Body(mass: 1,angularDamping: 0 );
    torusBody.addShape(torusShape);
    torusBody.position.set(0.01, 0.01, 0.01);
    torusBody.quaternion.setFromEuler(-Math.PI / 2, 0, 0);
    torusBody.angularVelocity.set(0, 0, 1);
    world.addBody(torusBody);
    demo.addVisual(torusBody, mesh: torusMesh, material: torusMaterial);

    // Do the raycasting
    final from = cannon.Vec3(-10, 0, 1);
    final to = cannon.Vec3(10, 0, 1);
    final result = cannon.RaycastResult();
    final raycastOptions = cannon.RayOptions();
    void postStepListener(event) {
      for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
          from.set(-10, i * 0.1, j * 0.1);
          to.set(10, i * 0.1, j * 0.1);
          result.reset();
          world.raycastClosest(from, to, raycastOptions, result);
          particleBodies[i * N + j].position.copy(result.hitPointWorld);
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
    groundBody.quaternion.setFromEuler(-Math.PI / 2, 0, 0);
    demo.world.addBody(groundBody);
    demo.addVisual(groundBody);

    // Sphere
    final sphereShape = cannon.Sphere();
    final sphereBody = cannon.Body(
      mass: 1,
      shape: sphereShape,
      position: cannon.Vec3(-3, 11, 3),
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
    torusBody.position.set(0, 4, 0);
    torusBody.quaternion.setFromEuler(math.pi / 2, 0, 0);
    torusBody.velocity.set(0, 1, 1);
    world.addBody(torusBody);
    demo.addVisual(torusBody, mesh: torusMesh, material: torusMaterial);
  }
  void triMeshSphere1(){
    final world = demo.world;

    final groundShape = cannon.Plane();
    final groundBody = cannon.Body(mass: 0);
    groundBody.addShape(groundShape);
    groundBody.quaternion.setFromEuler(-Math.PI / 2, 0, 0);
    demo.world.addBody(groundBody);
    demo.addVisual(groundBody);

    // Sphere
    final sphereShape = cannon.Sphere();
    final sphereBody = cannon.Body(
      mass: 1,
      shape: sphereShape,
      position: cannon.Vec3(-3, 11, 3),
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
    torusBody.position.set(0, 4, 0);
    torusBody.quaternion.setFromEuler(math.pi / 2, 0, 0);
    torusBody.velocity.set(0, 1, 1);
    world.addBody(torusBody);
    demo.addVisual(torusBody, mesh: torusMesh, material: torusMaterial);
  }
  void triMeshPartical(){
    final world = demo.world;

    final groundShape = cannon.Plane();
    final groundBody = cannon.Body(mass: 0);
    groundBody.addShape(groundShape);
    groundBody.quaternion.setFromEuler(-Math.PI / 2, 0, 0);
    demo.world.addBody(groundBody);
    demo.addVisual(groundBody);

    final shape = cannon.Particle();
    final body = cannon.Body(
      mass: 1,
      shape: shape,
      position: cannon.Vec3(-3, 11, 4),
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
    torusBody.position.set(0, 4, 0);
    torusBody.quaternion.setFromEuler(math.pi / 2, 0, 0);
    torusBody.velocity.set(0, 1, 1);
    world.addBody(torusBody);
    demo.addVisual(torusBody, mesh: torusMesh, material: torusMaterial);
  }
  void triMeshTriMesh(){
    final world = demo.world;

    final groundShape = cannon.Plane();
    final groundBody = cannon.Body(mass: 0);
    groundBody.addShape(groundShape);
    groundBody.quaternion.setFromEuler(-Math.PI / 2, 0, 0);
    demo.world.addBody(groundBody);
    demo.addVisual(groundBody);

   final torusGeometry1 = TorusGeometry();
    //final torusGeometry = IcosahedronGeometry(2,2);
    final torusMaterial1 = MeshStandardMaterial({'color': 0x2b4c7f});//{'color': 0x2b4c7f }'wireframe':true
    final torusMesh1 = Mesh(torusGeometry1, torusMaterial1);
    final shape = ConversionUtils.geometryToShape(torusGeometry1);
    final body = cannon.Body(
      mass: 1,
      shape: shape,
      position: cannon.Vec3(-3, 11, 4),
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
    torusBody.position.set(0, 4, 0);
    torusBody.quaternion.setFromEuler(math.pi / 2, 0, 0);
    torusBody.velocity.set(0, 1, 1);
    world.addBody(torusBody);
    demo.addVisual(torusBody, mesh: torusMesh, material: torusMaterial);
  }
  void triMeshBox(){
    final world = demo.world;

    final groundShape = cannon.Plane();
    final groundBody = cannon.Body(mass: 0);
    groundBody.addShape(groundShape);
    groundBody.quaternion.setFromEuler(-Math.PI / 2, 0, 0);
    demo.world.addBody(groundBody);
    demo.addVisual(groundBody);

    final shape = cannon.Box(cannon.Vec3( 0.5,0.5,0.5));
    final body = cannon.Body(
      mass: 1,
      shape: shape,
      position: cannon.Vec3(-3, 11, 0),
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
    torusBody.position.set(0, 4, 0);
    torusBody.quaternion.setFromEuler(math.pi / 2, 0, 0);
    torusBody.velocity.set(0, 1, 1);
    world.addBody(torusBody);
    demo.addVisual(torusBody, mesh: torusMesh, material: torusMaterial);
  }
  void triMeshCylinder(){
    final world = demo.world;

    final groundShape = cannon.Plane();
    final groundBody = cannon.Body(mass: 0);
    groundBody.addShape(groundShape);
    groundBody.quaternion.setFromEuler(-Math.PI / 2, 0, 0);
    demo.world.addBody(groundBody);
    demo.addVisual(groundBody);

    final shape = cannon.Cylinder(radiusTop: 0.5,radiusBottom:0.5);
    final body = cannon.Body(
      mass: 1,
      shape: shape,
      position: cannon.Vec3(0, 11, 3),
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
    torusBody.position.set(0, 4, 0);
    torusBody.quaternion.setFromEuler(math.pi / 2, 0, 0);
    torusBody.velocity.set(0, 1, 1);
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