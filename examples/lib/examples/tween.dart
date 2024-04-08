import 'package:flutter/material.dart';
import '../src/demo.dart';
import 'package:cannon_physics/cannon_physics.dart' as cannon;
import 'package:vector_math/vector_math.dart' as vmath;

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

    vmath.Vector3 startPosition = vmath.Vector3(-5, 2, 0);
    vmath.Vector3 endPosition = vmath.Vector3(5, 2, 0);
    const tweenTime = 3; // seconds

    cannon.Box boxShape = cannon.Box(vmath.Vector3(1, 1, 1));
    cannon.Body body = cannon.Body(
      mass: 0,
      type: cannon.BodyTypes.kinematic,
      position: startPosition,
    );
    body.addShape(boxShape);
    world.addBody(body);
    demo.addVisual(body);

    // Compute direction vector and get total length of the path
    vmath.Vector3 direction = vmath.Vector3.zero();
    endPosition.sub2(startPosition, direction);
    double totalLength = direction.length;
    direction.normalize();

    double speed = totalLength / tweenTime;
    direction.scale2(speed, body.velocity);

    // Save the start time
    double startTime = world.time;

    vmath.Vector3 offset = vmath.Vector3.zero();

    void postStepListener() {
      // Progress is a number where 0 is at start position and 1 is at end position
      double progress = (world.time - startTime) / tweenTime;

      if (progress < 1) {
        direction.scale2(progress * totalLength, offset);
        startPosition.add2(offset, body.position);
      } else {
        body.velocity.setValues(0, 0, 0);
        body.position.setFrom(endPosition);
        world.removeEventListener('postStep', postStepListener);
      }
    }

    world.addEventListener('postStep',(event){postStepListener();});
  }

  void setupWorld(){
    setScene();
  }
  @override
  Widget build(BuildContext context) {
    return demo.threeDart();
  }
}