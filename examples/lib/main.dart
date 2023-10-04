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
import 'package:flutter/material.dart' hide Tween;
import 'package:cannon_physics_example/examples/cloth.dart';
import 'package:cannon_physics_example/examples/tween.dart';

void main() {
  runApp(const MyApp());
}

class MyApp extends StatelessWidget {
  const MyApp({Key? key}) : super(key: key);

  // This widget is the root of your application.
  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      debugShowCheckedModeBanner: false,
      title: 'Flutter Demo',
      theme: ThemeData(
        primarySwatch: Colors.blue,
      ),
      home: const Examples(),
    );
  }
}

class Examples extends StatefulWidget{
  const Examples({
    Key? key,
  }) : super(key: key);

  @override
  _ExamplesPageState createState() => _ExamplesPageState();
}

class _ExamplesPageState extends State<Examples> {
  List<String> ex = [
    'cloth'
  ];
  double deviceHeight = double.infinity;
  double deviceWidth = double.infinity;

  List<Widget> displayExamples(){
    List<Widget> widgets = [];

    for(int i = 0;i < ex.length;i++){
      widgets.add(
        SizedBox(
          width: CSS.responsive(),
          height: CSS.responsive(),
          child: Column(
            children:[
              Container(
                width: CSS.responsive(),
                height: CSS.responsive(),
                child: InkWell(
                  onTap: (){

                  },
                  child: Image.asset('${ex[i]}.png'),
                ),
              )
            ]
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
    return SizedBox(
      height: deviceHeight,
      width: deviceWidth,
      child: Wrap(
        children: displayExamples(),
      )
    );
  }
}