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

class Trigger extends StatefulWidget {
  const Trigger({
    Key? key,
    this.offset = const Offset(0,0),
    this.settings
  }) : super(key: key);

  final Offset offset;
  final DemoSettings? settings;

  @override
  _TriggerState createState() => _TriggerState();
}

class _TriggerState extends State<Trigger> {
  late Demo demo;

  @override
  void initState() {
    demo = Demo(
      onSetupComplete: (){setState(() {});},
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

    const radius = 1.0;

    // Sphere moving towards right
    cannon.Sphere sphereShape = cannon.Sphere(radius);
    cannon.Body sphereBody = cannon.Body(mass: 1);
    sphereBody.addShape(sphereShape);
    sphereBody.position.set(-5, 0, 0);
    cannon.Vec3 impulse = cannon.Vec3(5.5, 0, 0);
    cannon.Vec3 topPoint = cannon.Vec3(0, radius, 0);
    sphereBody.applyImpulse(impulse, topPoint);
    sphereBody.linearDamping = 0.3;
    sphereBody.angularDamping = 0.3;
    world.addBody(sphereBody);
    demo.addVisual(sphereBody);

    // Trigger body
    cannon.Box boxShape = cannon.Box(cannon.Vec3(2, 2, 5));
    cannon.Body triggerBody = cannon.Body(isTrigger: true );
    triggerBody.addShape(boxShape);
    triggerBody.position.set(5, radius, 0);
    world.addBody(triggerBody);
    demo.addVisual(triggerBody);

    // It is possible to run code on the exit/enter
    // of the trigger.
    triggerBody.addEventListener('collide', (event){
      if (event.body == sphereBody) {
        print('The sphere entered the trigger! $event');
      }
    });
    world.addEventListener('endContact', (event){
      if (
        (event.bodyA == sphereBody && event.bodyB == triggerBody) ||
        (event.bodyB == sphereBody && event.bodyA == triggerBody)
      ) {
        print('The sphere exited the trigger! $event');
      }
    });
  }

  void setupWorld(){
    setScene();
  }
  @override
  Widget build(BuildContext context) {
    return demo.threeDart();
  }
}