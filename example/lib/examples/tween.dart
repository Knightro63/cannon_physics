import 'package:flutter/material.dart';
import '../src/demo.dart';
import 'package:cannon_physics/cannon_physics.dart' as cannon;

class Tween extends StatefulWidget {
  const Tween({
    Key? key,
    this.offset = const Offset(0,0),
    this.settings
  }) : super(key: key);

  final Offset offset;
  final DemoSettings? settings;

  @override
  _TweenState createState() => _TweenState();
}

class _TweenState extends State<Tween> {
  late Demo demo;

  @override
  void initState() {
    demo = Demo(
      onSetupComplete: (){setState(() {});},
      settings: DemoSettings(
        gx: 0,
        gy: -5,
        gz: 0,
        k: 5e6,
        iterations: 50
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

    cannon.Vec3 startPosition = cannon.Vec3(-5, 2, 0);
    cannon.Vec3 endPosition = cannon.Vec3(5, 2, 0);
    const tweenTime = 3; // seconds

    cannon.Box boxShape = cannon.Box(cannon.Vec3(1, 1, 1));
    cannon.Body body = cannon.Body(
      mass: 0,
      type: cannon.BodyTypes.kinematic,
      position: startPosition,
    );
    body.addShape(boxShape);
    world.addBody(body);
    demo.addVisual(body);

    // Compute direction vector and get total length of the path
    cannon.Vec3 direction = cannon.Vec3();
    endPosition.vsub(startPosition, direction);
    double totalLength = direction.length();
    direction.normalize();

    double speed = totalLength / tweenTime;
    direction.scale(speed, body.velocity);

    // Save the start time
    double startTime = world.time;

    cannon.Vec3 offset = cannon.Vec3();

    void postStepListener() {
      // Progress is a number where 0 is at start position and 1 is at end position
      double progress = (world.time - startTime) / tweenTime;

      if (progress < 1) {
        direction.scale(progress * totalLength, offset);
        startPosition.vadd(offset, body.position);
      } else {
        body.velocity.set(0, 0, 0);
        body.position.copy(endPosition);
        world.removeEventListener('postStep', postStepListener);
      }
    }

    demo.addEventListener((event){postStepListener();});
  }

  void setupWorld(){
    setScene();
  }
  @override
  Widget build(BuildContext context) {
    return demo.threeDart();
  }
}