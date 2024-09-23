import 'package:flutter/material.dart';
import 'dart:math' as math;
import '../src/demo.dart';
import 'package:cannon_physics/cannon_physics.dart' as cannon;
import 'package:vector_math/vector_math.dart' as vmath;

class RagdollData{
  RagdollData({
    required this.bodies,
    required this.constraints
  });
  List<cannon.Body> bodies;
  List<cannon.Constraint> constraints;
}

class RagDoll extends StatefulWidget {
  const RagDoll({
    Key? key,
    this.offset = const Offset(0,0),
    this.settings
  }) : super(key: key);

  final Offset offset;
  final DemoSettings? settings;

  @override
  _RagDollState createState() => _RagDollState();
}

class _RagDollState extends State<RagDoll> {
  late Demo demo;

  @override
  void initState() {
    demo = Demo(
      onSetupComplete: (){setState(() {});},
      settings: DemoSettings(
        gx: 0,
        gy: -5,
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
  void setScene() {
    final world = demo.world;

    final groundShape = cannon.Plane();
    final groundBody = cannon.Body(mass: 0);
    groundBody.addShape(groundShape);
    groundBody.position.setValues(0, -1, 0);
    groundBody.quaternion.setFromEuler(-math.pi / 2, 0, 0);
    world.addBody(groundBody);
    demo.addVisual(groundBody);
  }
  void normalConeJoints(){
    setScene();
    final world = demo.world;

    // Add a sphere to land on
    final sphereBody = createStaticSphere();
    world.addBody(sphereBody);
    demo.addVisual(sphereBody);

    // Create the ragdoll
    // It returns an array of body parts and their constraints
    final RagdollData data = createRagdoll(
      scale: 3,
      angle: math.pi / 4,
      angleShoulders: math.pi / 3,
      twistAngle: math.pi / 8,
    );

    data.bodies.forEach((body){
      // Move the ragdoll up
      final position = vmath.Vector3(0, 10, 0);
      body.position.add2(position, body.position);

      world.addBody(body);
      demo.addVisual(body);
    });

    data.constraints.forEach((constraint){
      world.addConstraint(constraint);
    });
  }
  void noConeJoints(){
    setScene();
    final world = demo.world;

    // Add a sphere to land on
    final sphereBody = createStaticSphere();
    world.addBody(sphereBody);
    demo.addVisual(sphereBody);

    // Create the ragdoll
    // It returns an array of body parts and their constraints
    final RagdollData data = createRagdoll(
      scale: 3,
      angle: math.pi,
      angleShoulders: math.pi,
      twistAngle: math.pi,
    );

    data.bodies.forEach((body){
      // Move the ragdoll up
      final position = vmath.Vector3(0, 10, 0);
      body.position.add2(position, body.position);

      world.addBody(body);
      demo.addVisual(body);
    });

    data.constraints.forEach((constraint){
      world.addConstraint(constraint);
    });
  }
  void thinConeJoints(){
    setScene();
    final world = demo.world;

    // Add a sphere to land on
    final sphereBody = createStaticSphere();
    world.addBody(sphereBody);
    demo.addVisual(sphereBody);

    // Create the ragdoll
    // It returns an array of body parts and their constraints
    final RagdollData data = createRagdoll(
      scale: 3,
      angle: 0,
      angleShoulders: 0,
      twistAngle:0,
    );

    data.bodies.forEach((body){
      // Move the ragdoll up
      final position = vmath.Vector3(0, 10, 0);
      body.position.add2(position, body.position);

      world.addBody(body);
      demo.addVisual(body);
    });

    data.constraints.forEach((constraint){
      world.addConstraint(constraint);
    });
  }
  cannon.Body createStaticSphere() {
    final sphereShape = cannon.Sphere(4);
    final sphereBody = cannon.Body(mass: 0);
    sphereBody.addShape(sphereShape);
    sphereBody.position.setValues(0, -1, 0);
    return sphereBody;
  }
  RagdollData createRagdoll({
    double scale = 1, 
    required double angle,
    required double angleShoulders,
    required double twistAngle 
  }) {
    List<cannon.Body> bodies = [];
    List<cannon.Constraint> constraints = [];

    final shouldersDistance = 0.5 * scale;
    final upperArmLength = 0.4 * scale;
    final lowerArmLength = 0.4 * scale;
    final upperArmSize = 0.2 * scale;
    final lowerArmSize = 0.2 * scale;
    final neckLength = 0.1 * scale;
    final headRadius = 0.25 * scale;
    final upperBodyLength = 0.6 * scale;
    final pelvisLength = 0.4 * scale;
    final upperLegLength = 0.5 * scale;
    final upperLegSize = 0.2 * scale;
    final lowerLegSize = 0.2 * scale;
    final lowerLegLength = 0.5 * scale;

    final headShape = cannon.Sphere(headRadius);
    final upperArmShape = cannon.Box(
      vmath.Vector3(upperArmLength * 0.5, upperArmSize * 0.5, upperArmSize * 0.5)
    );
    final lowerArmShape = cannon.Box(
      vmath.Vector3(lowerArmLength * 0.5, lowerArmSize * 0.5, lowerArmSize * 0.5)
    );
    final upperBodyShape = cannon.Box(
      vmath.Vector3(shouldersDistance * 0.5, lowerArmSize * 0.5, upperBodyLength * 0.5)
    );
    final pelvisShape = cannon.Box(
      vmath.Vector3(shouldersDistance * 0.5, lowerArmSize * 0.5, pelvisLength * 0.5)
    );
    final upperLegShape = cannon.Box(
      vmath.Vector3(upperLegSize * 0.5, lowerArmSize * 0.5, upperLegLength * 0.5)
    );
    final lowerLegShape = cannon.Box(
      vmath.Vector3(lowerLegSize * 0.5, lowerArmSize * 0.5, lowerLegLength * 0.5)
    );

    // Lower legs
    final lowerLeftLeg = cannon.Body(
      mass: 1,
      position: vmath.Vector3(shouldersDistance / 2, 0, lowerLegLength / 2),
    );
    final lowerRightLeg = cannon.Body(
      mass: 1,
      position: vmath.Vector3(-shouldersDistance / 2, 0, lowerLegLength / 2),
    );
    lowerLeftLeg.addShape(lowerLegShape);
    lowerRightLeg.addShape(lowerLegShape);
    bodies.add(lowerLeftLeg);
    bodies.add(lowerRightLeg);

    // Upper legs
    final upperLeftLeg = cannon.Body(
      mass: 1,
      position: vmath.Vector3(
        shouldersDistance / 2,
        0,
        lowerLeftLeg.position.z + lowerLegLength / 2 + upperLegLength / 2
      ),
    );
    final upperRightLeg = cannon.Body(
      mass: 1,
      position: vmath.Vector3(
        -shouldersDistance / 2,
        0,
        lowerRightLeg.position.z + lowerLegLength / 2 + upperLegLength / 2
      ),
    );
    upperLeftLeg.addShape(upperLegShape);
    upperRightLeg.addShape(upperLegShape);
    bodies.add(upperLeftLeg);
    bodies.add(upperRightLeg);

    // Pelvis
    final pelvis = cannon.Body(
      mass: 1,
      position: vmath.Vector3(0, 0, upperLeftLeg.position.z + upperLegLength / 2 + pelvisLength / 2),
    );
    pelvis.addShape(pelvisShape);
    bodies.add(pelvis);

    // Upper body
    final upperBody = cannon.Body(
      mass: 1,
      position: vmath.Vector3(0, 0, pelvis.position.z + pelvisLength / 2 + upperBodyLength / 2),
    );
    upperBody.addShape(upperBodyShape);
    bodies.add(upperBody);

    // Head
    final head = cannon.Body(
      mass: 1,
      position: vmath.Vector3(0, 0, upperBody.position.z + upperBodyLength / 2 + headRadius + neckLength),
    );
    head.addShape(headShape);
    bodies.add(head);

    // Upper arms
    final upperLeftArm = cannon.Body(
      mass: 1,
      position: vmath.Vector3(
        shouldersDistance / 2 + upperArmLength / 2,
        0,
        upperBody.position.z + upperBodyLength / 2
      ),
    );
    final upperRightArm = cannon.Body(
      mass: 1,
      position: vmath.Vector3(
        -shouldersDistance / 2 - upperArmLength / 2,
        0,
        upperBody.position.z + upperBodyLength / 2
      ),
    );
    upperLeftArm.addShape(upperArmShape);
    upperRightArm.addShape(upperArmShape);
    bodies.add(upperLeftArm);
    bodies.add(upperRightArm);

    // Lower arms
    final lowerLeftArm = cannon.Body(
      mass: 1,
      position: vmath.Vector3(
        upperLeftArm.position.x + lowerArmLength / 2 + upperArmLength / 2,
        0,
        upperLeftArm.position.z
      ),
    );
    final lowerRightArm = cannon.Body(
      mass: 1,
      position: vmath.Vector3(
        upperRightArm.position.x - lowerArmLength / 2 - upperArmLength / 2,
        0,
        upperRightArm.position.z
      ),
    );
    lowerLeftArm.addShape(lowerArmShape);
    lowerRightArm.addShape(lowerArmShape);
    bodies.add(lowerLeftArm);
    bodies.add(lowerRightArm);

    // Neck joint
    final neckJoint = cannon.ConeTwistConstraint(head, upperBody,
      pivotA: vmath.Vector3(0, 0, -headRadius - neckLength / 2),
      pivotB: vmath.Vector3(0, 0, upperBodyLength / 2),
      axisA: vmath.Vector3(0, 0, 1),
      axisB: vmath.Vector3(0, 0, 1),
      angle: angle,
      twistAngle: twistAngle,
    );
    constraints.add(neckJoint);

    // Knee joints
    final leftKneeJoint = cannon.ConeTwistConstraint(lowerLeftLeg, upperLeftLeg,
      pivotA: vmath.Vector3(0, 0, lowerLegLength / 2),
      pivotB: vmath.Vector3(0, 0, -upperLegLength / 2),
      axisA: vmath.Vector3(0, 0, 1),
      axisB: vmath.Vector3(0, 0, 1),
      angle: angle,
      twistAngle: twistAngle,
    );
    final rightKneeJoint = cannon.ConeTwistConstraint(lowerRightLeg, upperRightLeg, 
      pivotA: vmath.Vector3(0, 0, lowerLegLength / 2),
      pivotB: vmath.Vector3(0, 0, -upperLegLength / 2),
      axisA: vmath.Vector3(0, 0, 1),
      axisB: vmath.Vector3(0, 0, 1),
      angle: angle,
      twistAngle: twistAngle,
    );
    constraints.add(leftKneeJoint);
    constraints.add(rightKneeJoint);

    // Hip joints
    final leftHipJoint = cannon.ConeTwistConstraint(upperLeftLeg, pelvis,
      pivotA: vmath.Vector3(0, 0, upperLegLength / 2),
      pivotB: vmath.Vector3(shouldersDistance / 2, 0, -pelvisLength / 2),
      axisA: vmath.Vector3(0, 0, 1),
      axisB: vmath.Vector3(0, 0, 1),
      angle: angle,
      twistAngle: twistAngle,
    );
    final rightHipJoint = cannon.ConeTwistConstraint(upperRightLeg, pelvis,
      pivotA: vmath.Vector3(0, 0, upperLegLength / 2),
      pivotB: vmath.Vector3(-shouldersDistance / 2, 0, -pelvisLength / 2),
      axisA: vmath.Vector3(0, 0, 1),
      axisB: vmath.Vector3(0, 0, 1),
      angle: angle,
      twistAngle: twistAngle,
    );
    constraints.add(leftHipJoint);
    constraints.add(rightHipJoint);

    // Spine
    final spineJoint = cannon.ConeTwistConstraint(pelvis, upperBody,
      pivotA: vmath.Vector3(0, 0, pelvisLength / 2),
      pivotB: vmath.Vector3(0, 0, -upperBodyLength / 2),
      axisA: vmath.Vector3(0, 0, 1),
      axisB: vmath.Vector3(0, 0, 1),
      angle: angle,
      twistAngle: twistAngle,
    );
    constraints.add(spineJoint);

    // Shoulders
    final leftShoulder = cannon.ConeTwistConstraint(upperBody, upperLeftArm,
      pivotA: vmath.Vector3(shouldersDistance / 2, 0, upperBodyLength / 2),
      pivotB: vmath.Vector3(-upperArmLength / 2, 0, 0),
      axisA: vmath.Vector3(1, 0, 0),
      axisB: vmath.Vector3(1, 0, 0),
      angle: angleShoulders,
    );
    final rightShoulder = cannon.ConeTwistConstraint(upperBody, upperRightArm,
      pivotA: vmath.Vector3(-shouldersDistance / 2, 0, upperBodyLength / 2),
      pivotB: vmath.Vector3(upperArmLength / 2, 0, 0),
      axisA: vmath.Vector3(1, 0, 0),
      axisB: vmath.Vector3(1, 0, 0),
      angle: angleShoulders,
      twistAngle: twistAngle,
    );
    constraints.add(leftShoulder);
    constraints.add(rightShoulder);

    // Elbow joint
    final leftElbowJoint = cannon.ConeTwistConstraint(lowerLeftArm, upperLeftArm,
      pivotA: vmath.Vector3(-lowerArmLength / 2, 0, 0),
      pivotB: vmath.Vector3(upperArmLength / 2, 0, 0),
      axisA: vmath.Vector3(1, 0, 0),
      axisB: vmath.Vector3(1, 0, 0),
      angle: angle,
      twistAngle: twistAngle,
    );
    final rightElbowJoint = cannon.ConeTwistConstraint(lowerRightArm, upperRightArm,
      pivotA: vmath.Vector3(lowerArmLength / 2, 0, 0),
      pivotB: vmath.Vector3(-upperArmLength / 2, 0, 0),
      axisA: vmath.Vector3(1, 0, 0),
      axisB: vmath.Vector3(1, 0, 0),
      angle: angle,
      twistAngle: twistAngle,
    );
    constraints.add(leftElbowJoint);
    constraints.add(rightElbowJoint);

    return RagdollData(bodies:bodies, constraints:constraints);
  }
  void setupWorld(){
    demo.addScene('Normal Cone Joints',normalConeJoints);
    demo.addScene('No Cone Joints',noConeJoints);
    demo.addScene('Thin Cone Joints',thinConeJoints);
  }
  @override
  Widget build(BuildContext context) {
    return demo.threeDart();
  }
}