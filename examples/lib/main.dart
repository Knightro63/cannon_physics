import 'dart:io';

import 'package:cannon_physics_example/examples/basic_physics.dart';
import 'package:cannon_physics_example/examples/body_types.dart';
import 'package:cannon_physics_example/examples/bounce.dart';
import 'package:cannon_physics_example/examples/bunny.dart';
import 'package:cannon_physics_example/examples/callback.dart';
import 'package:cannon_physics_example/examples/collision_filter.dart';
import 'package:cannon_physics_example/examples/collisions.dart';
import 'package:cannon_physics_example/examples/compound.dart';
import 'package:cannon_physics_example/examples/constraints.dart';
import 'package:cannon_physics_example/examples/container.dart';
import 'package:cannon_physics_example/examples/convex.dart';
import 'package:cannon_physics_example/examples/events.dart';
import 'package:cannon_physics_example/examples/fixed_rotation.dart';
import 'package:cannon_physics_example/examples/fps.dart';
import 'package:cannon_physics_example/examples/friction.dart';
import 'package:cannon_physics_example/examples/friction_gravity.dart';
import 'package:cannon_physics_example/examples/heightfield.dart';
import 'package:cannon_physics_example/examples/hinge.dart';
import 'package:cannon_physics_example/examples/impulses.dart';
import 'package:cannon_physics_example/examples/jenga.dart';
import 'package:cannon_physics_example/examples/performance.dart';
import 'package:cannon_physics_example/examples/pile.dart';
import 'package:cannon_physics_example/examples/ragdoll.dart';
import 'package:cannon_physics_example/examples/raycast_vehicle.dart';
import 'package:cannon_physics_example/examples/rigid_vehicle.dart';
import 'package:cannon_physics_example/examples/shapes.dart';
import 'package:cannon_physics_example/examples/simple_friction.dart';
import 'package:cannon_physics_example/examples/single_body_on_plane.dart';
import 'package:cannon_physics_example/examples/sph.dart';
import 'package:cannon_physics_example/examples/split_solver.dart';
import 'package:cannon_physics_example/examples/spring.dart';
import 'package:cannon_physics_example/examples/tear.dart';
import 'package:cannon_physics_example/examples/trigger.dart';
import 'package:cannon_physics_example/examples/trimesh.dart';
import 'package:cannon_physics_example/examples/worker.dart'; //Fix
import 'package:css/css.dart';
import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart' hide Tween;
import 'package:cannon_physics_example/examples/cloth.dart';
import 'package:cannon_physics_example/examples/tween.dart';

void main() {
  runApp(MyApp());
}
class MyApp extends StatefulWidget{
  const MyApp({
    Key? key,
  }) : super(key: key);

  @override
  _MyAppState createState() => _MyAppState();
}

class _MyAppState extends State<MyApp> {
  final GlobalKey<NavigatorState> _navKey = GlobalKey<NavigatorState>();
  String onPage = '';

  void callback(String page){
    onPage = page;
    WidgetsBinding.instance.addPostFrameCallback((timeStamp) { 
      _navKey.currentState!.popAndPushNamed('/$page');
      setState(() {});
    });
  }

  // This widget is the root of your application.
  @override
  Widget build(BuildContext context) {
    widthInifity = MediaQuery.of(context).size.width;
    return SafeArea(
      child: MaterialApp(
        debugShowCheckedModeBanner: false,
        title: 'Flutter Demo',
        theme: CSS.darkTheme,
        home: Scaffold(
          appBar: (kIsWeb||!Platform.isAndroid) && onPage != ''? PreferredSize(
            preferredSize: Size(widthInifity,65),
            child:AppBar(callback: callback,page: onPage,)
          ):null,
          body: MaterialApp(
            debugShowCheckedModeBanner: false,
            title: 'Flutter Demo',
            theme: CSS.darkTheme,
            navigatorKey: _navKey,
            routes: {
              '/':(BuildContext context) {
                return Examples(callback: callback);
              },
              '/cloth':(BuildContext context) {
                return const Cloth();
              },
              '/fps':(BuildContext context) {
                return const FPSGame();
              },
              '/basic_physics':(BuildContext context) {
                return const BasicPhysics();
              },
              '/body_types':(BuildContext context) {
                return const BodyTypes();
              },
              '/bounce':(BuildContext context) {
                return const Bounce();
              },
              '/bunny':(BuildContext context) {
                return const Bunny();
              },
              '/callbacks':(BuildContext context) {
                return const Callback();
              },
              '/collision_filter':(BuildContext context) {
                return const CollisionFilter();
              },
              '/collisions':(BuildContext context) {
                return const Collisions();
              },
              '/compound':(BuildContext context) {
                return const Compound();
              },
              '/constraints':(BuildContext context) {
                return const Constraints();
              },
              '/container':(BuildContext context) {
                return const ContainerCP();
              },
              '/convex':(BuildContext context) {
                return const Convex();
              },
              '/events':(BuildContext context) {
                return const Events();
              },
              '/fixed_rotation':(BuildContext context) {
                return const FixedRotation();
              },
              '/friction_gravity':(BuildContext context) {
                return const FrictionGravity();
              },
              '/friction':(BuildContext context) {
                return const Friction();
              },
              '/heightfield':(BuildContext context) {
                return const Heightfield();
              },
              '/hinge':(BuildContext context) {
                return const Hinge();
              },
              '/impulses':(BuildContext context) {
                return const Impulses();
              },
              '/jenga':(BuildContext context) {
                return const Jenga();
              },
              '/performance':(BuildContext context) {
                return const Performance();
              },
              '/pile':(BuildContext context) {
                return const Pile();
              },
              '/ragdoll':(BuildContext context) {
                return const RagDoll();
              },
              '/raycast_vehicle':(BuildContext context) {
                return const RaycastVehicle();
              },
              '/rigid_vehicle':(BuildContext context) {
                return const RigidVehicle();
              },
              '/shapes':(BuildContext context) {
                return const Shapes();
              },
              '/simple_friction':(BuildContext context) {
                return const SimpleFriction();
              },
              '/single_body_on_plane':(BuildContext context) {
                return const SBOP();
              },
              '/sph':(BuildContext context) {
                return const SPH();
              },
              '/split_solver':(BuildContext context) {
                return const SplitSolver();
              },
              '/spring':(BuildContext context) {
                return const Spring();
              },
              '/tear':(BuildContext context) {
                return const Tear();
              },
              '/trigger':(BuildContext context) {
                return const Trigger();
              },
              '/trimesh':(BuildContext context) {
                return const TriMesh();
              },
              '/tween':(BuildContext context) {
                return const Tween();
              },
              '/worker':(BuildContext context) {
                return const Worker();
              },
            }
          ),
        )
      )
    );
  }
}

class AppBar extends StatelessWidget{
  AppBar({
    Key? key,
    required this.page,
    required this.callback
  }):super(key: key);
  String page;
  void Function(String page) callback;
  // This widget is the root of your application.
  @override
  Widget build(BuildContext context) {
    return Container(
      height: 65,
      padding: const EdgeInsets.only(left: 10),
      color: Theme.of(context).cardColor,
      child: Row(
        children: [
          InkWell(
            onTap: (){
              callback('');
            },
            child: const Icon(
              Icons.arrow_back_ios_new_rounded
            ),
          ),
          const SizedBox(width: 20,),
          Text(
            (page[0].toUpperCase()+page.substring(1)).replaceAll('_', ' '),
            style: Theme.of(context).primaryTextTheme.bodyLarge,
          )
        ],
      ),
    );
  }
}

class Examples extends StatefulWidget{
  const Examples({
    Key? key,
    required this.callback
  }) : super(key: key);

  final void Function(String page) callback;

  @override
  _ExamplesPageState createState() => _ExamplesPageState();
}

class _ExamplesPageState extends State<Examples> {
  List<String> ex = [
    'cloth',
    'fps',
    'basic_physics',
    'body_types',
    'bounce',
    'bunny',
    'callbacks',
    'collision_filter',
    'collisions',
    'compound',
    'constraints',
    'container',
    'convex',
    'events',
    'fixed_rotation',
    'friction_gravity',
    'friction',
    'heightfield',
    'hinge',
    'impulses',
    'jenga',
    'performance',
    'pile',
    'ragdoll',
    'raycast_vehicle',
    'rigid_vehicle',
    'shapes',
    'simple_friction',
    'single_body_on_plane',
    'sph',
    'split_solver',
    'spring',
    'tear',
    'trigger',
    'trimesh',
    'tween',
    'worker',
  ];
  double deviceHeight = double.infinity;
  double deviceWidth = double.infinity;

  List<Widget> displayExamples(){
    List<Widget> widgets = [];

    double response = CSS.responsive(width: 480);

    for(int i = 0;i < ex.length;i++){
      widgets.add(
        InkWell(
          onTap: (){
            widget.callback(ex[i]);
          },
          child: Container(
            margin: const EdgeInsets.all(10),
            width: response-65,
            height: response,
            decoration: BoxDecoration(
              color: Theme.of(context).cardColor,
              borderRadius: const BorderRadius.all(Radius.circular(10)),
              boxShadow: [
                BoxShadow(
                  color: Theme.of(context).shadowColor,
                  blurRadius: 5,
                  offset: const Offset(2, 2),
                ),
              ]
            ),
            child: Column(
              children:[
                Image.asset(
                  'assets/images/${ex[i]}.png',
                  fit: BoxFit.fitHeight,
                  width: response,
                  height: response-65,
                ),
                const SizedBox(height: 10),
                Text(
                  ex[i].replaceAll('_',' ').toUpperCase(),
                  style: Theme.of(context).primaryTextTheme.bodyLarge,
                )
              ]
            )
          )
        )
      );
    }

    return widgets;
  }

  @override
  Widget build(BuildContext context) {
    deviceHeight = MediaQuery.of(context).size.height;
    deviceWidth = MediaQuery.of(context).size.width;
    
    return SingleChildScrollView(
      child: Wrap(
        runAlignment: WrapAlignment.spaceBetween,
        alignment: WrapAlignment.center,
        crossAxisAlignment: WrapCrossAlignment.center,
        children: displayExamples(),
      )
    );
  }
}