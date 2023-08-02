import '../equations/equation.dart';
import '../world/world_class.dart';

/// Constraint equation solver base class.
class Solver {
  /// All equations to be solved
  List<Equation> equations;
  
  /// The number of solver iterations determines quality of the constraints in the world.
  /// The number of solver iterations determines quality of the constraints in the world. The more iterations, the more correct simulation. More iterations need more computations though. If you have a large gravity force in your world, you will need more iterations.
  int iterations;

  /// @todo remove useless constructor
  Solver({
    this.equations = const [],
    this.iterations = 10,
  });

  /// Should be implemented in subclasses!
  /// @todo use abstract
  /// @return number of iterations performed
  int solve(double dt, World world){
    return 0;
  }

  void addEquation(Equation eq) {
    if (eq.enabled && !eq.bi.isTrigger && !eq.bj.isTrigger) {
      equations.add(eq);
    }
  }

  void removeEquation(Equation eq) {
    final eqs = equations;
    int i = eqs.indexOf(eq);
    if (i != -1) {
      eqs.removeAt(i);
    }
  }
  
  void removeAllEquations() {
    equations.length = 0;
  }
}
