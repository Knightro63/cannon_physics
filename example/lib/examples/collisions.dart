import 'package:flutter/material.dart';
import 'package:three_dart/three_dart.dart';
import '../src/demo.dart';
import 'package:cannon_physics/cannon_physics.dart' as cannon;
import 'dart:math' as math;

extension on cannon.Quaternion{
  Quaternion toQuaternion(){
    return Quaternion(x,y,z,w);
  }
}
extension on cannon.Vec3{
  Vector3 toVector3(){
    return Vector3(x,y,z);
  }
}

class Collisions extends StatefulWidget {
  const Collisions({
    Key? key,
    this.offset = const Offset(0,0),
    this.settings
  }) : super(key: key);

  final Offset offset;
  final DemoSettings? settings;

  @override
  _CollisionsState createState() => _CollisionsState();
}

class _CollisionsState extends State<Collisions> {
  late Demo demo;

  @override
  void initState() {
    demo = Demo(
      onSetupComplete: (){setState(() {});},
      updatePhysics: (){updateCannonPhysics();},
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
    //setScene0();
    // setScene1();
    setScene2();
    // setScene3();
  }
  void setScene0(){
    final world = demo.world;
    final sphereShape = cannon.Sphere(1);

    // Sphere 1
    final body1 = cannon.Body(mass: 5 );
    body1.addShape(sphereShape);
    body1.position.set(-5, 0, 0);
    body1.velocity.set(5, 0, 0);
    body1.linearDamping = 0;
    world.addBody(body1);
    demo.addVisual(body1);

    // Sphere 2
    final body2 = cannon.Body(mass: 5 );
    body2.addShape(sphereShape);
    body2.linearDamping = 0;
    body2.position.set(5, 0, 0);
    body2.velocity.set(-5, 0, 0);
    world.addBody(body2);
    demo.addVisual(body2);
  }
  void setScene1(){
    final world = demo.world;

    final boxShape = cannon.Box(cannon.Vec3(1, 1, 1));
    final sphereShape = cannon.Sphere(1);
    // Box
    final body1 = cannon.Body(mass: 5 );
    body1.addShape(boxShape);
    body1.position.set(-5, 0, 0);
    body1.velocity.set(5, 0, 0);
    body1.linearDamping = 0;
    world.addBody(body1);
    demo.addVisual(body1);

    // Sphere
    final body2 = cannon.Body(mass: 5 );
    body2.addShape(sphereShape);
    body2.position.set(5, 0, 0);
    body2.velocity.set(-5, 0, 0);
    body2.linearDamping = 0;
    world.addBody(body2);
    demo.addVisual(body2);
  }
  void setScene2(){
    final world = demo.world;

    final boxShape = cannon.Box(cannon.Vec3(1, 1, 1));
    final sphereShape = cannon.Sphere(1);

    // Box
    final body1 = cannon.Body(
      mass: 5,
      position: cannon.Vec3(-5,0,0),
      velocity: cannon.Vec3(5,0,0),
      linearDamping: 0,
      shape: boxShape
    );

    final quaternion = cannon.Quaternion();
    quaternion.setFromEuler(0, math.pi * 0.25, 0);
    body1.quaternion.copy(quaternion);
    world.addBody(body1);
    demo.addVisual(body1);

    // Sphere
    final body2 = cannon.Body(
      mass: 5,
      position: cannon.Vec3(5,0,0),
      velocity: cannon.Vec3(-5,0,0),
      linearDamping: 0,
      shape: sphereShape
    );
    world.addBody(body2);
    demo.addVisual(body2);
  }
  void setScene3(){
    final world = demo.world;
    final boxShape = cannon.Box(cannon.Vec3(1, 1, 1));
    final sphereShape = cannon.Sphere(1);

    // Box
    final body1 = cannon.Body(mass: 5);
    body1.addShape(boxShape);
    body1.position.set(-5, 0, 0);
    body1.velocity.set(5, 0, 0);
    body1.linearDamping = 0;
    final quaternion1 = cannon.Quaternion();
    quaternion1.setFromEuler(0, math.pi * 0.25, 0);
    final quaternion2 = cannon.Quaternion();
    quaternion2.setFromEuler(0, 0, math.pi * 0.25);
    final quaternion = quaternion1.mult(quaternion2);
    body1.quaternion.copy(quaternion);
    world.addBody(body1);
    demo.addVisual(body1);

    // Sphere
    final body2 = cannon.Body(mass: 5);
    body2.addShape(sphereShape);
    body2.position.set(5, 0, 0);
    body2.velocity.set(-5, 0, 0);
    body2.linearDamping = 0;
    world.addBody(body2);
    demo.addVisual(body2);
  }
  void setupWorld(){
    final world = demo.world;
    world.broadphase = cannon.NaiveBroadphase();
    setScene();
  }
  void updateCannonPhysics() {
    demo.world.fixedStep();
    double x, y, z;
    Object3D mesh; 
    cannon.Body body;

    for(int i = 0; i < demo.bodies.length;i++){
      body = demo.bodies[i];
      mesh = demo.visuals[i];

      if(body.sleepState != cannon.BodySleepStates.sleeping){
        
        mesh.position.copy(body.position.toVector3());
        mesh.quaternion.copy(body.quaternion.toQuaternion());

        // reset position
        if(mesh.position.y<-100){
          x = -100 + Math.random()*200;
          z = -100 + Math.random()*200;
          y = 100 + Math.random()*1000;
          body.position = cannon.Vec3(x,y,z);
        }
      } 
    }
  }
  @override
  Widget build(BuildContext context) {
    return demo.threeDart();
  }
}