import 'dart:async';
import 'dart:math' as math;

import 'package:cannon_physics_example/src/conversion_utils.dart';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';

import 'package:three_js/three_js.dart' as three;
import 'package:cannon_physics/cannon_physics.dart' as cannon;
import 'package:vector_math/vector_math.dart' as vmath;

class SphereData{
  SphereData({
    required this.mesh,
    required this.body,
  });

  three.Mesh mesh;
  cannon.Body body;
}

class TestGame extends StatefulWidget {
  const TestGame({Key? key,}):super(key: key);

  @override
  _TestGamePageState createState() => _TestGamePageState();
}

class _TestGamePageState extends State<TestGame> {
  late three.FirstPersonControls fpsControl;
  late three.ThreeJS threeJs;

  int stepsPerFrame = 5;
  double gravity = 30;

  List<cannon.Body> balls = [];
  int sphereIdx = 0;

  cannon.CapsuleLathe playerCollider = cannon.CapsuleLathe(
    radiusBottom: 0.35,
    radiusTop: 0.35,
    height: 0.65
  );//Vector3( 0, 0.35, 0 ), Vector3( 0, 1, 0 ), 0.35);
  late cannon.Body playerBody;

  bool playerOnFloor = false;
  int mouseTime = 0;
  Map<LogicalKeyboardKey,bool> keyStates = {
    LogicalKeyboardKey.keyW: false,
    LogicalKeyboardKey.keyA: false,
    LogicalKeyboardKey.keyS: false,
    LogicalKeyboardKey.keyD: false,
    LogicalKeyboardKey.space: false,

    LogicalKeyboardKey.arrowUp: false,
    LogicalKeyboardKey.arrowLeft: false,
    LogicalKeyboardKey.arrowDown: false,
    LogicalKeyboardKey.arrowRight: false,
  };

  three.Vector3 vector1 = three.Vector3();
  three.Vector3 vector2 = three.Vector3();
  three.Vector3 vector3 = three.Vector3();

  late final three.Vector3 playerVelocity;
  
  late cannon.Body sphereBody;

  late cannon.World world;
  List<SphereData> spheres = [];
  double? lastCallTime;
  bool split = false;
  bool paused = false;

  late vmath.Vector3 startingPos;

  @override
  void initState() {
    threeJs = three.ThreeJS(
      onSetupComplete: (){setState(() {});},
      setup: setup
    );
    super.initState();
  }
  @override
  void dispose() {
    fpsControl.dispose();
    threeJs.dispose();
    super.dispose();
  }
  //----------------------------------
  //  cannon PHYSICS
  //----------------------------------
  void initCannonPhysics(){
    world = cannon.World();
    world.quatNormalizeSkip = 0;
    world.quatNormalizeFast = false;
    world.allowSleep = true;

    cannon.GSSolver solver = cannon.GSSolver();
    lastCallTime = world.performance.now().toDouble();
    world.defaultContactMaterial.contactEquationStiffness = 1e7;
    world.defaultContactMaterial.contactEquationRelaxation = 4;

    solver.iterations = 1;
    solver.tolerance = 1;
    
    if(split){
      world.solver = cannon.SplitSolver(solver);
    }
    else{
      world.solver = solver;
    }

    world.gravity.setValues(0, -gravity, 0);
    world.broadphase = cannon.NaiveBroadphase();
  }

  void updatePhysics() {
    // Step world
    const timeStep = 1 / 60;
    final now = world.performance.now() / 1000;

    if (lastCallTime == null) {
      // last call time not saved, cant guess elapsed time. Take a simple step.
      world.step(timeStep);
      lastCallTime = now;
      return;
    }
    double timeSinceLastCall = now - lastCallTime!;

    world.step(timeStep, timeSinceLastCall, stepsPerFrame);
    lastCallTime = now;
  }

  Future<void> setup() async {
    initCannonPhysics();

    threeJs.scene = three.Scene();
    threeJs.scene.background = three.Color.fromHex32(0x88ccee);

    threeJs.camera = three.PerspectiveCamera(70, threeJs.width / threeJs.height, 0.1, 1000);
    threeJs.camera.rotation.order = three.RotationOrders.yxz;

    // lights
    three.HemisphereLight fillLight1 = three.HemisphereLight( 0x4488bb, 0x002244, 0.5 );
    fillLight1.position.setValues( 2, 1, 1 );
    threeJs.scene.add(fillLight1);

    three.DirectionalLight directionalLight = three.DirectionalLight( 0xffffff, 0.8 );
    directionalLight.position.setValues( - 5, 25, - 1 );
    // directionalLight.castShadow = true;

    // directionalLight.shadow!.camera!.near = 0.01;
    // directionalLight.shadow!.camera!.far = 500;
    // directionalLight.shadow!.camera!.right = 30;
    // directionalLight.shadow!.camera!.left = - 30;
    // directionalLight.shadow!.camera!.top	= 30;
    // directionalLight.shadow!.camera!.bottom = - 30;
    // directionalLight.shadow!.mapSize.width = 1024;
    // directionalLight.shadow!.mapSize.height = 1024;
    // directionalLight.shadow!.radius = 4;
    // directionalLight.shadow!.bias = - 0.00006;

    threeJs.scene.add(directionalLight);

   three.GLTFLoader().setPath('assets/models/').fromAsset('collision-world.glb').then((gltf){
      three.Object3D object = gltf!.scene;
      cannon.Trimesh triShape = ConversionUtils.fromGraphNode(object);
      final triBody = cannon.Body();
      triBody.addShape(triShape);
      world.addBody(triBody);
      
      threeJs.scene.add(object);

      object.traverse((child){
        if(child.type == 'Mesh'){
          three.Object3D part = child;
          part.castShadow = true;
          part.visible = true;
          part.receiveShadow = true;
        }
      });
    });

    // Create a slippery material (friction coefficient = 0.0)
    cannon.Material physicsMaterial = cannon.Material(name:'slipperyMaterial');
    cannon.ContactMaterial physicsPhysics = cannon.ContactMaterial(
      physicsMaterial, 
      physicsMaterial,
      friction: 0.0,
      restitution: 0.3,
    );
    world.addContactMaterial(physicsPhysics);

    // Create the user collision sphere
    const double mass = 5;
    const radius = 1.3;
    late cannon.Shape sphereShape = cannon.Sphere(radius);
    sphereBody = cannon.Body(mass: mass, material: physicsMaterial);
    sphereBody.addShape(sphereShape);
    sphereBody.position.setValues(0, 5, 0);
    sphereBody.linearDamping = 0.9;

    //Create Player
    playerBody = cannon.Body(mass: mass);
    playerBody.addShape(playerCollider);
    world.addBody(playerBody);

    startingPos = vmath.Vector3.copy(playerBody.position);

    //add fps controller
    fpsControl = three.FirstPersonControls(camera: threeJs.camera, listenableKey: threeJs.globalKey);
    fpsControl.lookSpeed = 1/100;
    fpsControl.movementSpeed = 15.0;
    fpsControl.lookType = three.LookType.position;
    playerVelocity = fpsControl.velocity;
    fpsControl.domElement.addEventListener( three.PeripheralType.keyup, (event){
      if(event.keyId == 32){
        playerVelocity.y = 15;
      }
    }, false );
    fpsControl.domElement.addEventListener( three.PeripheralType.pointerdown, (event){
      mouseTime = DateTime.now().millisecondsSinceEpoch;
    }, false );
    fpsControl.domElement.addEventListener( three.PeripheralType.pointerup, (event){
      throwBall();
    }, false );

    
    threeJs.addAnimationEvent((dt){
      world.fixedStep();
      updatePhysics();
      fpsControl.update(lastCallTime!);
      updateVisuals();
      teleportPlayerIfOob();
    });
  }

  void throwBall() {
    double shootVelocity = 15 + ( 1 - math.exp((mouseTime-DateTime.now().millisecondsSinceEpoch) * 0.1));
    cannon.Sphere ballShape = cannon.Sphere(0.2);
    three.SphereGeometry ballGeometry = three.SphereGeometry(ballShape.radius, 32, 32);

    three.Vector3 getShootDirection() {
      three.Vector3 vector = three.Vector3(0, 0, 1);
      vector.unproject(threeJs.camera);
      three.Ray ray = three.Ray.originDirection(sphereBody.position.toVector3(), vector.sub(sphereBody.position.toVector3()).normalize());
      return ray.direction;
    }

    cannon.Body ballBody = cannon.Body(mass: 1);
    ballBody.addShape(ballShape);
    three.Mesh ballMesh = three.Mesh(ballGeometry, three.MeshLambertMaterial.fromMap({ 'color': 0xdddddd }));

    ballMesh.castShadow = true;
    ballMesh.receiveShadow = true;

    spheres.add(SphereData(
      mesh: ballMesh,
      body: ballBody,
    ));

    three.Vector3 shootDirection = getShootDirection();
    ballBody.velocity.setValues(
      shootDirection.x * shootVelocity,
      shootDirection.y * shootVelocity,
      shootDirection.z * shootVelocity
    );
    const radius = 1.3;
    // Move the ball outside the player sphere
    double x = sphereBody.position.x + shootDirection.x * (radius * 1.02 + ballShape.radius);
    double y = sphereBody.position.y + shootDirection.y * (radius * 1.02 + ballShape.radius);
    double z = sphereBody.position.z + shootDirection.z * (radius * 1.02 + ballShape.radius);
    ballBody.position.setValues(x, y, z);
    ballMesh.position.setFrom(ballBody.position.toVector3());

    world.addBody(ballBody);
    threeJs.scene.add(ballMesh);
  }
  void updatePlayer(){
    final body = playerBody;
    // Interpolated or not?
    vmath.Vector3 position = body.interpolatedPosition;
    vmath.Quaternion quaternion = body.interpolatedQuaternion;

    if(paused) {
      position = body.position;
      quaternion = body.quaternion;
    }

    threeJs.camera.position.setFromCylindrical(position.toVector3());
    threeJs.camera.position.setFrom( three.Vector3(position.x,position.y+playerCollider.height,position.z));//position.toVector3()playerCollider.height);
  }
  void updateVisuals(){
    for (int i = 0; i < spheres.length; i++) {
      final body = spheres[i].body;
      final visual = spheres[i].mesh;
      three.Object3D dummy = three.Object3D();

      // Interpolated or not?
      vmath.Vector3 position = body.interpolatedPosition;
      vmath.Quaternion quaternion = body.interpolatedQuaternion;
      if(paused) {
        position = body.position;
        quaternion = body.quaternion;
      }

      if (visual is three.InstancedMesh) {
        dummy.position.setFrom(position.toVector3());
        dummy.quaternion.setFrom(quaternion.toQuaternion());

        dummy.updateMatrix();

        visual.setMatrixAt(body.index, dummy.matrix);
        visual.instanceMatrix!.needsUpdate = true;
      } 
      else {
        visual.position.setFrom(position.toVector3());
        visual.quaternion.setFrom(quaternion.toQuaternion());
      }
    }
    //updatePlayer();
  }

  void teleportPlayerIfOob(){
    if(threeJs.camera.position.y <= - 25){
      // playerCollider.start.set(0,0.35,0);
      // playerCollider.end.set(0,1,0);
      // playerCollider.radius = 0.35;
      final body = playerBody;
      body.position = startingPos.clone();
      threeJs.camera.position.setFrom(three.Vector3(startingPos.x,startingPos.y+playerCollider.height,startingPos.z));
      threeJs.camera.rotation.set(0,0,0);
    }
  }


  @override
  Widget build(BuildContext context) {
    return threeJs.build();
  }
}