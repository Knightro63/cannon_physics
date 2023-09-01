import 'package:cannon_physics_example/examples/basic_physics2.dart';
import 'package:cannon_physics_example/examples/bounce.dart';
import 'package:cannon_physics_example/examples/collisions.dart';
import 'package:cannon_physics_example/examples/fps.dart';
import 'package:cannon_physics_example/examples/hinge.dart';
import 'package:cannon_physics_example/examples/jenga.dart';
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
      home: const Hinge(),
    );
  }
}