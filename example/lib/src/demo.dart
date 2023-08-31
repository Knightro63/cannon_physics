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
import 'package:flutter/services.dart';

enum RenderMode{solid,wireframe}

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

class DemoSettings{
  DemoSettings({
    this.stepFrequency = 60,
    this.quatNormalizeSkip = 2,
    this.quatNormalizeFast = true,
    this.gx = 0,
    this.gy = 0,
    this.gz = 0,
    this.iterations = 3,
    this.tolerance = 0.0001,
    this.k = 1e6,
    this.d = 3,
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
  });
  int stepFrequency;
  int quatNormalizeSkip;
  bool quatNormalizeFast;
  double gx;
  double gy;
  double gz;
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
}

/**
 * Demo utility class. If you want to learn how to connect cannon.js with three.js, please look at the examples/threejs_* instead.
 */
class Demo{
  void Function() onSetupComplete;
  Demo({required this.onSetupComplete,DemoSettings? settings, void Function()? updatePhysics}){
    this.settings = settings ?? DemoSettings();

    world = cannon.World();
    this.updatePhysics = updatePhysics ?? () => _updateCannonPhysics();
    

    world.gravity = cannon.Vec3(this.settings.gx,this.settings.gy,this.settings.gz);
    world.quatNormalizeSkip = this.settings.quatNormalizeSkip;
    world.quatNormalizeFast = this.settings.quatNormalizeFast;

    cannon.GSSolver solver = cannon.GSSolver();

    lastCallTime = world.performance.now().toDouble();
    world.defaultContactMaterial.contactEquationStiffness = this.settings.k;
    world.defaultContactMaterial.contactEquationRelaxation = 10;

    solver.iterations = this.settings.iterations;
    solver.tolerance = this.settings.tolerance;

    world.solver = solver;
    world.broadphase = cannon.NaiveBroadphase();

    if(this.settings.rendermode == RenderMode.solid){
      _currentMaterial = _solidMaterial;
    }
    else{
      _currentMaterial = _wireframeMaterial;
    }

    bboxMeshCache = GeometryCache(scene, (){
      return Mesh(three.BoxGeometry(1, 1, 1), bboxMaterial);
    });
  }

  final GlobalKey<DomLikeListenableState> globalKey = GlobalKey<DomLikeListenableState>();
  FocusNode _node = FocusNode();

  late GeometryCache bboxMeshCache;

  late cannon.World world;
  List<cannon.Body> bodies = [];
  List<three.Mesh> visuals = [];
  late DemoSettings settings;

  bool animationReady = false;
  late FlutterGlPlugin three3dRender;
  WebGLRenderTarget? renderTarget;
  WebGLRenderer? renderer;
  late OrbitControls controls;

  late double width;
  late double height;
  Size? screenSize;
  Scene scene = Scene();
  late Camera camera;
  double dpr = 1.0;
  bool verbose = false;
  bool disposed = false;
  
  dynamic sourceTexture;
  
  double? lastCallTime;
  bool resetCallTime = false;
  double toRad = 0.0174532925199432957;

  late SpotLight _spotLight;
  late AmbientLight _ambientLight;

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
  int mouseTime = 0;
  bool mounted = false;

  MeshBasicMaterial _wireframeMaterial = MeshBasicMaterial({'color': 0xffffff, 'wireframe': true});
  MeshLambertMaterial _solidMaterial = MeshLambertMaterial({'color': 0xdddddd});
  late three.Material _currentMaterial;

  late void Function() updatePhysics;

  bool get pause => settings.paused;
  cannon.World get getWorld => world;
  set pause(bool p){
    settings.paused = p;
    resetCallTime = true;
  }

  void dispose(){
    disposed = true;
    three3dRender.dispose();
  }

  void initSize(BuildContext context){
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
      if(!pause){
        updatePhysics();
      }
      animate();
    });
  }

  void initPage() async {
    scene.fog = three.Fog(0x222222, 1000, 2000);

    camera = PerspectiveCamera(24, width/height, 5, 2000);
    camera.position.set(0,20,30);
    camera.lookAt(three.Vector3(0, 0, 0));

    controls = OrbitControls(camera, globalKey);
    controls.rotateSpeed = 1.0;
    controls.zoomSpeed = 1.2;
    controls.enableDamping = true;
    controls.enablePan = false;
    controls.dampingFactor = 0.2;
    controls.minDistance = 10;
    controls.maxDistance = 500;

    _ambientLight = AmbientLight(0xffffff, 0.1);
    scene.add(_ambientLight);

    DirectionalLight directionalLight = DirectionalLight( 0xffffff , 0.15);
    directionalLight.position.set(-30, 40, 30);
    directionalLight.target!.position.set( 0, 0, 0 );
    scene.add(directionalLight);

    _spotLight = three.SpotLight(0xffffff, 0.9, 0.0, Math.PI / 8, 1.0);
    _spotLight.position.set(-30, 40, 30);
    _spotLight.target!.position.set(0, 0, 0);

    _spotLight.castShadow = true;

    _spotLight.shadow!.camera!.near = 10;
    _spotLight.shadow!.camera!.far = 100;
    _spotLight.shadow!.camera!.fov = 30;

    // spotLight.shadow.bias = -0.0001
    _spotLight.shadow!.mapSize.width = 2048;
    _spotLight.shadow!.mapSize.height = 2048;

    scene.add(_spotLight);
  }
  void setRenderMode(RenderMode mode){
    switch(mode) {
      case RenderMode.solid:
        _currentMaterial = _solidMaterial;
        _spotLight.intensity = 1;
        _ambientLight.color!.setHex(0x222222);
        break;
      case RenderMode.wireframe:
        _currentMaterial = _wireframeMaterial;
        _spotLight.intensity = 0;
        _ambientLight.color!.setHex(0xffffff);
        break;
    }

    // set the materials
    visuals.forEach((visual){
      if (visual.material != null) {
        visual.material = _currentMaterial;
      }
      visual.traverse((child){
        if (child.material) {
          child.material = _currentMaterial;
        }
      });
    });

    settings.rendermode = mode;
  }

  void addVisual(cannon.Body body, [three.Material? material]){
    MeshLambertMaterial particleMaterial = MeshLambertMaterial({ 'color': 0xff0000 });
    MeshBasicMaterial triggerMaterial = MeshBasicMaterial({ 'color': 0x00ff00, 'wireframe': false });
    // if it's a particle paint it red, if it's a trigger paint it as green, otherwise just gray
    final isParticle = body.shapes.every((s) => s is cannon.Particle);
    final mat = material ?? (isParticle ? particleMaterial : body.isTrigger ? triggerMaterial : _currentMaterial);

    // get the correspondant three.js mesh
    var mesh = ConversionUtils.bodyToMesh(body, mat);
    
    // enable shadows on every object
    // mesh.traverse((child){
    //   child.castShadow = true;
    //   child.receiveShadow = true;
    // });

    bodies.add(body);
    visuals.addAll(mesh);
    scene.addAll(mesh);
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
    resetCallTime = true;
  }

  void _updateCannonPhysics() {
    // Step world
    final timeStep = 1 / settings.stepFrequency;
    final now = world.performance.now() / 1000;

    if (lastCallTime == null) {
      // last call time not saved, cant guess elapsed time. Take a simple step.
      world.step(timeStep);
      lastCallTime = now;
      return;
    }
    double timeSinceLastCall = now - lastCallTime!;
    if (resetCallTime) {
      timeSinceLastCall = 0;
      resetCallTime = false;
    }
    world.step(timeStep, timeSinceLastCall, settings.maxSubSteps);
    lastCallTime = now;
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
    initPage();
    initRenderer();
    // setupWorld();
    mounted = true;
    animate();
    onSetupComplete();
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

    // TODO web wait dom ok!!!
    Future.delayed(const Duration(milliseconds: 100), () async {
      await three3dRender.prepareContext();
      initScene();
    });
  }

  void updateVisuals(){
    // Copy position data into visuals
    for (int i = 0; i < bodies.length; i++) {
      final body = bodies[i];
      final visual = visuals[i];
      Object3D dummy = Object3D();

      // Interpolated or not?
      cannon.Vec3 position = body.interpolatedPosition;
      cannon.Quaternion quaternion = body.interpolatedQuaternion;
      if (settings.paused) {
        position = body.position;
        quaternion = body.quaternion;
      }

      if (visual is InstancedMesh) {
        dummy.position.copy(position.toVector3());
        dummy.quaternion.copy(quaternion.toQuaternion());

        dummy.updateMatrix();

        visual.setMatrixAt(body.index, dummy.matrix);
        visual.instanceMatrix!.needsUpdate = true;
      } else {
        visual.position.copy(position.toVector3());
        visual.quaternion.copy(quaternion.toQuaternion());
      }
    }

    // Render contacts
    contactMeshCache.restart();
    if (settings.contacts) {
      // if ci is even - use body i, else j
      for (int i = 0; i < world.contacts.length; i++) {
        cannon.ContactEquation contact = world.contacts[i];

        for (int ij = 0; ij < 2; ij++) {
          Mesh mesh = contactMeshCache.request();
          cannon.Body b = ij == 0 ? contact.bi : contact.bj;
          cannon.Vec3 r = ij == 0 ? contact.ri : contact.rj;
          mesh.position.set(b.position.x + r.x, b.position.y + r.y, b.position.z + r.z);
        }
      }
    }
    contactMeshCache.hideCached();

    // Lines from center of mass to contact point
    cm2contactMeshCache.restart();
    if (settings.cm2contact) {
      for (int i = 0; i < world.contacts.length; i++) {
        cannon.ContactEquation contact = world.contacts[i];

        for (int ij = 0; ij < 2; ij++) {
          const line = cm2contactMeshCache.request();
          cannon.Body b = ij == 0 ? contact.bi : contact.bj;
          cannon.Vec3 r = ij == 0 ? contact.ri : contact.rj;
          line.scale.set(r.x, r.y, r.z);
          makeSureNotZero(line.scale);
          line.position.copy(b.position);
        }
      }
    }
    cm2contactMeshCache.hideCached();

    distanceConstraintMeshCache.restart();
    p2pConstraintMeshCache.restart();
    if (settings.constraints) {
      world.constraints.forEach((constraint){
        switch (true) {
          // Lines for distance constraints
          case constraint is cannon.DistanceConstraint: {
            constraint.equations.forEach((equation){
              cannon.Body bi = equation.bi;
              cannon.Body bj = equation.bj;

              const line = this.distanceConstraintMeshCache.request();

              // Remember, bj is either a Vec3 or a Body.
              cannon.Vec3 vector = bj.position ?? bj;

              line.scale.set(vector.x - bi.position.x, vector.y - bi.position.y, vector.z - bi.position.z)
              makeSureNotZero(line.scale);
              line.position.copy(bi.position);
            });

            break;
          }

          // Lines for point to point constraints
          case constraint is cannon.PointToPointConstraint: {
            constraint.equations.forEach((equation){
              cannon.Body bi = equation.bi;
              cannon.Body bj = equation.bj;

              const relLine1 = p2pConstraintMeshCache.request();
              const relLine2 = p2pConstraintMeshCache.request();
              const diffLine = p2pConstraintMeshCache.request();
              if (equation.ri) {
                relLine1.scale.set(equation.ri.x, equation.ri.y, equation.ri.z);
              }
              if (equation.rj) {
                relLine2.scale.set(equation.rj.x, equation.rj.y, equation.rj.z);
              }
              // BUG this is not exposed anymore in the ContactEquation, this sections needs to be updated
              if (equation.penetrationVec) {
                diffLine.scale.set(-equation.penetrationVec.x, -equation.penetrationVec.y, -equation.penetrationVec.z)
              }
              makeSureNotZero(relLine1.scale);
              makeSureNotZero(relLine2.scale);
              makeSureNotZero(diffLine.scale);
              relLine1.position.copy(bi.position);
              relLine2.position.copy(bj.position);

              if (equation.bj && equation.rj) {
                equation.bj.position.vadd(equation.rj, diffLine.position);
              }
            });
            break;
          }
        }
      });
    }
    p2pConstraintMeshCache.hideCached();
    distanceConstraintMeshCache.hideCached();

    // Normal lines
    this.normalMeshCache.restart();
    if (settings.normals) {
      for (int i = 0; i < world.contacts.length; i++) {
        cannon.ContactEquation constraint = world.contacts[i];

        cannon.Body bi = constraint.bi;
        cannon.Body bj = constraint.bj;
        const line = this.normalMeshCache.request();

        const constraintNormal = constraint.ni;
        const body = bi;
        line.scale.set(constraintNormal.x, constraintNormal.y, constraintNormal.z);
        makeSureNotZero(line.scale);
        line.position.copy(body.position);
        constraint.ri.vadd(line.position, line.position);
      }
    }
    this.normalMeshCache.hideCached();

    // Frame axes for each body
    this.axesMeshCache.restart();
    if (settings.axes) {
      for (int i = 0; i < bodies.length; i++) {
        cannon.Body body = bodies[i];

        const mesh = axesMeshCache.request();
        mesh.position.copy(body.position);
        mesh.quaternion.copy(body.quaternion);
      }
    }
    axesMeshCache.hideCached();

    // AABBs
    bboxMeshCache.restart();
    if (settings.aabbs) {
      for (int i = 0; i < bodies.length; i++) {
        cannon.Body body = bodies[i];
        if (body.updateAABB) {
          if (body.aabbNeedsUpdate) {
            body.updateAABB();
          }

          // Todo: cap the infinite AABB to scene AABB, for now just dont render
          if (
            isFinite(body.aabb.lowerBound.x) &&
            isFinite(body.aabb.lowerBound.y) &&
            isFinite(body.aabb.lowerBound.z) &&
            isFinite(body.aabb.upperBound.x) &&
            isFinite(body.aabb.upperBound.y) &&
            isFinite(body.aabb.upperBound.z) &&
            body.aabb.lowerBound.x - body.aabb.upperBound.x != 0 &&
            body.aabb.lowerBound.y - body.aabb.upperBound.y != 0 &&
            body.aabb.lowerBound.z - body.aabb.upperBound.z != 0
          ) {
            const mesh = bboxMeshCache.request();
            mesh.scale.set(
              body.aabb.lowerBound.x - body.aabb.upperBound.x,
              body.aabb.lowerBound.y - body.aabb.upperBound.y,
              body.aabb.lowerBound.z - body.aabb.upperBound.z
            );
            mesh.position.set(
              (body.aabb.lowerBound.x + body.aabb.upperBound.x) * 0.5,
              (body.aabb.lowerBound.y + body.aabb.upperBound.y) * 0.5,
              (body.aabb.lowerBound.z + body.aabb.upperBound.z) * 0.5
            );
          }
        }
      }
    }
    bboxMeshCache.hideCached();
  }
  void makeSureNotZero(vector) {
    if (vector.x == 0) {
      vector.x = 1e-6;
    }
    if (vector.y == 0) {
      vector.y = 1e-6;
    }
    if (vector.z == 0) {
      vector.z = 1e-6;
    }
  }
  Widget threeDart() {
    return Builder(builder: (BuildContext context) {
      initSize(context);
      return Container(
        width: screenSize!.width,
        height: screenSize!.height,
        color: Theme.of(context).canvasColor,
        child: RawKeyboardListener(
          focusNode: _node,
          onKey: (event){
            if(event is RawKeyDownEvent){
              if(
                event.data.logicalKey == LogicalKeyboardKey.keyW || 
                event.data.logicalKey == LogicalKeyboardKey.keyA || 
                event.data.logicalKey == LogicalKeyboardKey.keyS || 
                event.data.logicalKey == LogicalKeyboardKey.keyD || 
                event.data.logicalKey == LogicalKeyboardKey.arrowUp || 
                event.data.logicalKey == LogicalKeyboardKey.arrowLeft || 
                event.data.logicalKey == LogicalKeyboardKey.arrowDown || 
                event.data.logicalKey == LogicalKeyboardKey.arrowRight ||
                event.data.logicalKey == LogicalKeyboardKey.space
              ){
                keyStates[event.data.logicalKey] = true;
              }
            }
            else if(event is RawKeyUpEvent){
              if(
                event.data.logicalKey == LogicalKeyboardKey.keyW || 
                event.data.logicalKey == LogicalKeyboardKey.keyA || 
                event.data.logicalKey == LogicalKeyboardKey.keyS || 
                event.data.logicalKey == LogicalKeyboardKey.keyD ||
                event.data.logicalKey == LogicalKeyboardKey.arrowUp || 
                event.data.logicalKey == LogicalKeyboardKey.arrowLeft || 
                event.data.logicalKey == LogicalKeyboardKey.arrowDown || 
                event.data.logicalKey == LogicalKeyboardKey.arrowRight ||
                event.data.logicalKey == LogicalKeyboardKey.space
              ){
                keyStates[event.data.logicalKey] = false;
              }
            }
          },
          child: DomLikeListenable(
            key: globalKey,
            builder: (BuildContext context) {
              FocusScope.of(context).requestFocus(_node);
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
        )
      );
    });
  }
}