import 'dart:async';
import 'dart:io';
import 'dart:math' as math;

import 'package:flutter/material.dart';
import 'package:flutter/foundation.dart';

import 'package:flutter_gl/flutter_gl.dart';
import 'package:cannon_physics/cannon_physics.dart' as cannon;
import 'package:three_dart/three_dart.dart' as three;
import 'package:three_dart/three_dart.dart' hide Texture, Color;
import 'package:three_dart_jsm/three_dart_jsm.dart';
import 'conversion_utils.dart';

enum RenderMode{solid,wireframe}

class DemoSettings{
  DemoSettings({
    this.stepFrequency= 60,
    this.quatNormalizeSkip= 2,
    this.quatNormalizeFast= true,
    this.gx= 0,
    this.gy= 0,
    this.gz= 0,
    this.iterations= 3,
    this.tolerance= 0.0001,
    this.k= 1e6,
    this.d= 3,
    this.scene= 0,
    this.paused= false,
    this.rendermode= RenderMode.solid,
    this.constraints= false,
    this.contacts= false, // Contact points
    this.cm2contact= false, // center of mass to contact points
    this.normals= false, // contact normals
    this.axes= false, // "local" frame axes
    this.shadows= false,
    this.aabbs= false,
    this.profiling= false,
    this.maxSubSteps = 20,
    //...options,
  });
  int stepFrequency;
  int quatNormalizeSkip;
  bool quatNormalizeFast;
  int gx;
  int gy;
  int gz;
  int iterations;
  double tolerance;
  double k;
  int d;
  int scene;
  bool paused;
  RenderMode rendermode;
  bool constraints;
  bool contacts;
  bool cm2contact;
  bool normals;
  bool axes;
  bool shadows;
  bool aabbs;
  bool profiling;
  int maxSubSteps;
  //...options,
}

class BasicPhysics extends StatefulWidget {
  const BasicPhysics({
    Key? key,
    this.offset = const Offset(0,0),
    this.settings
  }) : super(key: key);

  final Offset offset;
  final DemoSettings? settings;

  @override
  _BasicPhysicsPageState createState() => _BasicPhysicsPageState();
}

class _BasicPhysicsPageState extends State<BasicPhysics> {
  FocusNode node = FocusNode();
  // gl values
  //late Object3D object;
  bool animationReady = false;
  late FlutterGlPlugin three3dRender;
  WebGLRenderTarget? renderTarget;
  WebGLRenderer? renderer;
  late OrbitControls controls;
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
  
  double? lastCallTime;
  bool resetCallTime = false;
  double toRad = 0.0174532925199432957;

  late cannon.World world;
  List<cannon.Body> bodies = [];
  List<three.Object3D> visuals = [];
  late DemoSettings settings;

  @override
  void initState() {
    settings = widget.settings ?? DemoSettings();
    super.initState();
  }
  @override
  void dispose() {
    disposed = true;
    three3dRender.dispose();
    super.dispose();
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
  void animate() {
    if (!mounted || disposed) {
      return;
    }

    render();

    Future.delayed(const Duration(milliseconds: 1000~/60), () {
      updateCannonPhysics();
      animate();
    });
  }
  
  Future<void> initPage() async {
    scene = Scene();
    scene.fog = three.Fog(0x222222, 1000, 2000);

    camera = PerspectiveCamera(24, width / height, 5, 2000);
    camera.position.set(0,20,30);
    camera.lookAt(three.Vector3(0, 0, 0));

    controls = OrbitControls(camera, _globalKey);
    controls.rotateSpeed = 1.0;
    controls.zoomSpeed = 1.2;
    controls.enableDamping = true;
    controls.enablePan = false;
    controls.dampingFactor = 0.2;
    controls.minDistance = 10;
    controls.maxDistance = 500;
    
    scene.add(AmbientLight(0xffffff, 0.1));
    DirectionalLight directionalLight = DirectionalLight( 0xffffff , 0.15);
    directionalLight.position.set(-30, 40, 30);
    directionalLight.target!.position.set( 0, 0, 0 );
    scene.add(directionalLight);

    SpotLight spotLight = three.SpotLight(0xffffff, 0.9, 0, Math.PI / 8, 1);
    spotLight.position.set(-30, 40, 30);
    spotLight.target!.position.set(0, 0, 0);

    spotLight.castShadow = true;

    spotLight.shadow!.camera!.near = 10;
    spotLight.shadow!.camera!.far = 100;
    spotLight.shadow!.camera!.fov = 30;

    // spotLight.shadow.bias = -0.0001
    spotLight.shadow!.mapSize.width = 2048;
    spotLight.shadow!.mapSize.height = 2048;

    scene.add(spotLight);
  }

  void addVisual(cannon.Body body){
    // if it's a particle paint it red, if it's a trigger paint it as green, otherwise just gray
    final isParticle = body.shapes.every((s) => s is cannon.Particle);
    final material = isParticle ? this.particleMaterial : body.isTrigger ? this.triggerMaterial : this.currentMaterial;

    // get the correspondant three.js mesh
    final mesh = ConversionUtils.bodyToMesh(body, material);

    // enable shadows on every object
    mesh.traverse((child){
      child.castShadow = true;
      child.receiveShadow = true;
    });

    bodies.add(body);
    visuals.add(mesh);
    scene.add(mesh);
  }
  void addVisuals(List<cannon.Body> bodies) {
    bodies.forEach((body){
      addVisual(body);
    });
  }
  void removeVisual(cannon.Body body) {
    final index = bodies.indexOf(body);// .findIndex((b) => b.id === body.id);

    if (index == -1) {
      return;
    }

    final visual = visuals[index];

    bodies.splice(index, 1);
    visuals.splice(index, 1);

    scene.remove(visual);
  }

  void removeAllVisuals() {
    while (bodies.isNotEmpty) {
      removeVisual(bodies[0]);
    }
  }

  void start(){
    animationReady = true;
  }

  void updateCannonPhysics() {
    // Step world
    final timeStep = 1 / this.settings.stepFrequency;
    final now = world.performance.now() / 1000;

    if (lastCallTime == null) {
      // last call time not saved, cant guess elapsed time. Take a simple step.
      world.step(timeStep);
      lastCallTime = now;
      return;
    }
    double timeSinceLastCall = now - lastCallTime!;
    if (animationReady) {
      timeSinceLastCall = 0;
      resetCallTime = false;
    }
    world.step(timeStep, timeSinceLastCall, this.settings.maxSubSteps);
    lastCallTime = now;
  }
  getWorld() {
    return world;
  }
  void render() {
    final _gl = three3dRender.gl;
    renderer!.render(scene, camera);
    _gl.flush();
    controls.update();
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
  void initScene() async{
    await initPage();
    initRenderer();
    initCannonPhysics();
    animate();
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
          }
        ),
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