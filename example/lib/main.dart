import 'package:cannon_physics_example/examples/basic_physics.dart';
import 'package:cannon_physics_example/examples/body_types.dart';
import 'package:cannon_physics_example/examples/bounce.dart';
import 'package:cannon_physics_example/examples/bunny.dart';
import 'package:cannon_physics_example/examples/callback.dart';
import 'package:cannon_physics_example/examples/collision_filter.dart';
import 'package:cannon_physics_example/examples/collisions.dart';
import 'package:cannon_physics_example/examples/compound.dart';
import 'package:cannon_physics_example/examples/constraints.dart'; //Fix
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
import 'package:cannon_physics_example/examples/ragdoll.dart'; //Fix
import 'package:cannon_physics_example/examples/raycast_vehicle.dart'; //Fix
import 'package:cannon_physics_example/examples/rigid_vehicle.dart'; //Fix
import 'package:cannon_physics_example/examples/split_solver.dart';
import 'package:cannon_physics_example/examples/spring.dart';
import 'package:cannon_physics_example/examples/tear.dart';
import 'package:cannon_physics_example/examples/trigger.dart';
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
      home: const RigidVehicle(),
    );
  }
}