import '../objects/rigid_body.dart';
import '../equations/equation_class.dart';

/// Constraint base class
class Constraint {

  /// Equations to be solved in this constraint.
  List<Equation> equations = [];

  Body bodyA;
  Body bodyB;
  late int id;
  
  /// Set to false if you don't want the bodies to collide when they are connected.
  bool collideConnected;

  static int idCounter = 0;

  Constraint(
    this.bodyA,
    this.bodyB,
    {
      this.collideConnected = true,
      bool wakeUpBodies = true
    }
  ) {
    id = Constraint.idCounter++;
    if (wakeUpBodies) {
      bodyA.wakeUp();
      bodyB.wakeUp();
    }
  }

  ///Update all the equations with data.
  void update() {
    throw('method update() not implmemented in this Constraint subclass!');
  }

  /// Enables all equations in the constraint.
  void enable() {
    final eqs = equations;
    for (int i = 0; i < eqs.length; i++) {
      eqs[i].enabled = true;
    }
  }
  
  /// Disables all equations in the constraint.
  void disable() {
    final eqs = equations;
    for (int i = 0; i < eqs.length; i++) {
      eqs[i].enabled = false;
    }
  }
}
