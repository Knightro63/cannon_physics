import 'dart:async';
import 'dart:io';

import 'package:flutter/material.dart';
import 'package:flutter/foundation.dart';

import 'package:flutter_gl/flutter_gl.dart';
import 'package:cannon_physics/cannon_physics.dart' as cannon;
import 'package:three_dart/three_dart.dart' as three;
import 'package:three_dart/three_dart.dart' hide Texture, Color;
import 'package:three_dart_jsm/three_dart_jsm.dart';

import 'package:flutter/services.dart';

extension on cannon.Vec3{
  Vector3 toVector3(){
    return Vector3(x,y,z);
  }
}
extension on cannon.Quaternion{
  Quaternion toQuaternion(){
    return Quaternion(x,y,z,w);
  }
}
class FPSGame extends StatefulWidget {
  const FPSGame({
    Key? key,
    this.offset = const Offset(0,0)
  }) : super(key: key);

  final Offset offset;

  @override
  _FPSGamePageState createState() => _FPSGamePageState();
}

class _FPSGamePageState extends State<FPSGame> {
  FocusNode node = FocusNode();
  // gl values
  //late Object3D object;
  bool animationReady = false;
  late FlutterGlPlugin three3dRender;
  WebGLRenderTarget? renderTarget;
  WebGLRenderer? renderer;
  //late OrbitControls controls;
  int? fboId;
  late double width;
  late double height;
  Size? screenSize;
  late Scene scene;
  late Camera camera;
  double dpr = 1.0;
  bool verbose = false;
  bool disposed = false;
  final GlobalKey<DomLikeListenableState> _globalKey = GlobalKey<DomLikeListenableState>();
  dynamic sourceTexture;

  late three.BufferGeometry clothGeometry;
  late three.Mesh floorGeometry;

  //cannon var
  late cannon.World world;
  late cannon.Shape sphereShape;
  late cannon.Body sphereBody;
  late cannon.Material physicsMaterial;
  late three.MeshLambertMaterial material;
  late double lastCallTime;
  List<cannon.Body> balls = [];
  List<three.Mesh> ballMeshes = [];
  List<cannon.Body> boxes = [];
  List<three.Mesh> boxMeshes = [];

  int mouseTime = 0;

  @override
  void initState() {
    super.initState();
  }
  @override
  void dispose() {
    disposed = true;
    three3dRender.dispose();
    super.dispose();
  }
  void initScene() async{
    await initThree();
    initRenderer();
    initCannonPhysics();
    animate();
  }
  void initSize(BuildContext context) {
    if (screenSize != null) {
      return;
    }

    final mqd = MediaQuery.of(context);

    screenSize = mqd.size;
    dpr = mqd.devicePixelRatio;

    initPlatformState();
  }

  Future<void> initThree() async {
    scene = Scene();

    camera = PerspectiveCamera(75, width / height, 0.1, 1000);
    camera.rotation.order = 'YXZ';
    
    scene.add(AmbientLight( 0x3D4143 ) );
    DirectionalLight light = DirectionalLight( 0xffffff , 1.4);
    light.position.set( 300, 1000, 500 );
    light.target!.position.set( 0, 0, 0 );
    light.castShadow = true;

    int d = 300;
    light.shadow!.camera = OrthographicCamera( -d, d, d, -d,  500, 1600 );
    light.shadow!.bias = 0.0001;
    light.shadow!.mapSize.width = light.shadow!.mapSize.height = 1024;

    scene.add( light );

    material = three.MeshLambertMaterial({ 'color': 0xdddddd });

    // floor
    three.PlaneGeometry floorGeometry = three.PlaneGeometry(300, 300, 100, 100);
    three.Mesh floor = three.Mesh(floorGeometry, material);
    floor.receiveShadow = true;
    scene.add(floor);

    animationReady = true;
  }

  //----------------------------------
  //  cannon PHYSICS
  //----------------------------------
  void initCannonPhysics(){
    world = cannon.World();
    lastCallTime = world.performance.now().toDouble();
    // Tweak contact properties.
    // Contact stiffness - use to make softer/harder contacts
    world.defaultContactMaterial.contactEquationStiffness = 1e9;

    // Stabilization time in number of timesteps
    world.defaultContactMaterial.contactEquationRelaxation = 4;

    cannon.GSSolver solver = cannon.GSSolver();
    solver.iterations = 7;
    solver.tolerance = 0.1;
    world.solver = cannon.SplitSolver(solver);
    // use this to test non-split solver
    // world.solver = solver

    world.gravity.set(0, -20, 0);

    // Create a slippery material (friction coefficient = 0.0)
    physicsMaterial = cannon.Material(name:'physics');
    cannon.ContactMaterial physics_physics = cannon.ContactMaterial(
      physicsMaterial, 
      physicsMaterial,
      friction: 0.0,
      restitution: 0.3,
    );

    // We must add the contact materials to the world
    world.addContactMaterial(physics_physics);

    // Create the user collision sphere
    const radius = 1.3;
    sphereShape = cannon.Sphere(radius);
    sphereBody = cannon.Body( mass: 5, material: physicsMaterial );
    sphereBody.addShape(sphereShape);
    sphereBody.position.set(0, 5, 0);
    sphereBody.linearDamping = 0.9;
    world.addBody(sphereBody);

    // Create the ground plane
    cannon.Plane groundShape = cannon.Plane();
    cannon.Body groundBody = cannon.Body(mass: 0, material: physicsMaterial );
    groundBody.addShape(groundShape);
    groundBody.quaternion.setFromEuler(-Math.PI / 2, 0, 0);
    world.addBody(groundBody);

    // Add boxes both in cannon.js and three.js
    cannon.Vec3 halfExtents = cannon.Vec3(1, 1, 1);
    cannon.Box boxShape = cannon.Box(halfExtents);
    three.BoxGeometry boxGeometry = three.BoxGeometry(halfExtents.x * 2, halfExtents.y * 2, halfExtents.z * 2);

    for (int i = 0; i < 7; i++) {
      cannon.Body boxBody = cannon.Body(mass: 5);
      boxBody.addShape(boxShape);
      three.Mesh boxMesh = three.Mesh(boxGeometry, material);

      double x = (Math.random() - 0.5) * 20;
      double y = (Math.random() - 0.5) * 1 + 1;
      double z = (Math.random() - 0.5) * 20;

      boxBody.position.set(x, y, z);
      boxMesh.position.copy(boxBody.position);

      boxMesh.castShadow = true;
      boxMesh.receiveShadow = true;

      world.addBody(boxBody);
      scene.add(boxMesh);
      boxes.add(boxBody);
      boxMeshes.add(boxMesh);
    }

    // Add linked boxes
    const size = 0.5;
    const mass = 0.3;
    const space = 0.1 * size;
    const N = 5;
    cannon.Vec3 halfExtents2 = cannon.Vec3(size, size, size * 0.1);
    cannon.Box boxShape2 = cannon.Box(halfExtents2);
    three.BoxGeometry boxGeometry2 = three.BoxGeometry(halfExtents2.x * 2, halfExtents2.y * 2, halfExtents2.z * 2);

    late cannon.Body last;
    for (int i = 0; i < N; i++) {
      // Make the fist one static to support the others
      cannon.Body boxBody = cannon.Body( mass: i == 0 ? 0 : mass );
      boxBody.addShape(boxShape2);
      three.Mesh boxMesh = three.Mesh(boxGeometry2, material);
      boxBody.position.set(5, (N - i) * (size * 2 + 2 * space) + size * 2 + space, 0);
      boxBody.linearDamping = 0.01;
      boxBody.angularDamping = 0.01;

      boxMesh.castShadow = true;
      boxMesh.receiveShadow = true;

      world.addBody(boxBody);
      scene.add(boxMesh);
      boxes.add(boxBody);
      boxMeshes.add(boxMesh);

      if (i > 0) {
        // Connect the body to the last one
        cannon.PointToPointConstraint constraint1 = cannon.PointToPointConstraint(
          boxBody,
          last,
          cannon.Vec3(-size, size + space, 0),
          cannon.Vec3(-size, -size - space, 0)
        );
        cannon.PointToPointConstraint constranit2 = cannon.PointToPointConstraint(
          boxBody,
          last,
          cannon.Vec3(size, size + space, 0),
          cannon.Vec3(size, -size - space, 0)
        );
        world.addConstraint(constraint1);
        world.addConstraint(constranit2);
      }

      last = boxBody;
    }
  }

  void throwBall() {
    const shootVelocity = 15;
    cannon.Sphere ballShape = cannon.Sphere(0.2);
    three.SphereGeometry ballGeometry = three.SphereGeometry(ballShape.radius, 32, 32);

    three.Vector3 getShootDirection() {
      three.Vector3 vector = three.Vector3(0, 0, 1);
      vector.unproject(camera);
      three.Ray ray = three.Ray(sphereBody.position.toVector3(), vector.sub(sphereBody.position).normalize());
      return ray.direction;
    }

    cannon.Body ballBody = cannon.Body(mass: 1);
    ballBody.addShape(ballShape);
    three.Mesh ballMesh = three.Mesh(ballGeometry, material);

    ballMesh.castShadow = true;
    ballMesh.receiveShadow = true;

    world.addBody(ballBody);
    scene.add(ballMesh);
    balls.add(ballBody);
    ballMeshes.add(ballMesh);

    three.Vector3 shootDirection = getShootDirection();
    ballBody.velocity.set(
      shootDirection.x * shootVelocity,
      shootDirection.y * shootVelocity,
      shootDirection.z * shootVelocity
    );
    const radius = 1.3;
    // Move the ball outside the player sphere
    double x = sphereBody.position.x + shootDirection.x * (radius * 1.02 + ballShape.radius);
    double y = sphereBody.position.y + shootDirection.y * (radius * 1.02 + ballShape.radius);
    double z = sphereBody.position.z + shootDirection.z * (radius * 1.02 + ballShape.radius);
    ballBody.position.set(x, y, z);
    ballMesh.position.copy(ballBody.position);
  }

  void animate() {
    if (!mounted || disposed) {
      return;
    }

    render();

    Future.delayed(const Duration(milliseconds: 16), () {
      animate();
    });
  }
  void render() {
    final _gl = three3dRender.gl;
    renderer!.render(scene, camera);
    _gl.flush();
    //controls.update();

    double time = world.performance.now() / 1000;
    double dt = time - lastCallTime;
    lastCallTime = time;

    if (dt != 0) {
      world.fixedStep();

      // Update ball positions
      for (int i = 0; i < balls.length; i++) {
        ballMeshes[i].position.copy(balls[i].position);
        ballMeshes[i].quaternion.copy(balls[i].quaternion.toQuaternion());
      }

      // Update box positions
      for (int i = 0; i < boxes.length; i++) {
        boxMeshes[i].position.copy(boxes[i].position);
        boxMeshes[i].quaternion.copy(boxes[i].quaternion.toQuaternion());
      }
    }
    if(!kIsWeb) {
      three3dRender.updateTexture(sourceTexture);
    }
  }
  void initRenderer() {
    Map<String, dynamic> _options = {
      "width": width,
      "height": height,
      "gl": three3dRender.gl,
      "antialias": true,
      "canvas": three3dRender.element,
    };

    if(!kIsWeb && Platform.isAndroid){
      _options['logarithmicDepthBuffer'] = true;
    }

    renderer = WebGLRenderer(_options);
    renderer!.setPixelRatio(dpr);
    renderer!.setSize(width, height, false);
    renderer!.shadowMap.enabled = true;
    renderer!.shadowMap.type = three.PCFShadowMap;
    //renderer!.outputEncoding = three.sRGBEncoding;

    if(!kIsWeb){
      WebGLRenderTargetOptions pars = WebGLRenderTargetOptions({"format": RGBAFormat,"samples": 8});
      renderTarget = WebGLRenderTarget((width * dpr).toInt(), (height * dpr).toInt(), pars);
      renderer!.setRenderTarget(renderTarget);
      sourceTexture = renderer!.getRenderTargetGLTexture(renderTarget!);
    }
    else{
      renderTarget = null;
    }
  }

  Future<void> initPlatformState() async {
    width = screenSize!.width;
    height = screenSize!.height;

    three3dRender = FlutterGlPlugin();

    Map<String, dynamic> _options = {
      "antialias": true,
      "alpha": true,
      "width": width.toInt(),
      "height": height.toInt(),
      "dpr": dpr,
      'precision': 'highp'
    };
    await three3dRender.initialize(options: _options);

    setState(() {});

    // TODO web wait dom ok!!!
    Future.delayed(const Duration(milliseconds: 100), () async {
      await three3dRender.prepareContext();
      initScene();
    });
  }

  Widget threeDart() {
    return Builder(builder: (BuildContext context) {
      initSize(context);
      return Container(
        width: screenSize!.width,
        height: screenSize!.height,
        color: Theme.of(context).canvasColor,
        child: RawKeyboardListener(
          focusNode: node,
          child: Listener(
            onPointerDown: (details){
              mouseTime = DateTime.now().millisecondsSinceEpoch;
            },
            onPointerUp: (details){
              throwBall();
            },
            onPointerHover: (PointerHoverEvent details){
              if(animationReady){
                camera.rotation.y -= details.delta.dx/100;
                camera.rotation.x -= details.delta.dy/100;
              }
            },
            child: DomLikeListenable(
              key: _globalKey,
              builder: (BuildContext context) {
                FocusScope.of(context).requestFocus(node);
                return Container(
                  width: width,
                  height: height,
                  color: Theme.of(context).canvasColor,
                  child: Builder(builder: (BuildContext context) {
                    if (kIsWeb) {
                      return three3dRender.isInitialized
                          ? HtmlElementView(
                              viewType:
                                  three3dRender.textureId!.toString())
                          : Container();
                    } else {
                      return three3dRender.isInitialized
                          ? Texture(textureId: three3dRender.textureId!)
                          : Container();
                    }
                  })
                );
              }),
          )
        )
      );
    });
  }

  @override
  Widget build(BuildContext context) {
    return SizedBox(
      height: double.infinity,
      width: double.infinity,
      child: Stack(
        children: [
          threeDart(),
        ],
      )
    );
  }
}