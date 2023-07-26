import '../equations/equation.dart';
import '../world/world.dart';

/**
 * Constraint equation solver base class.
 */
class Solver {
  /**
   * All equations to be solved
   */
  List<Equation> equations;

  /**
   * @todo remove useless constructor
   */
  Solver({
    this.equations = const []
  });

  /**
   * Should be implemented in subclasses!
   * @todo use abstract
   * @return number of iterations performed
   */
  num solve(num dt, World world){
    return 0;
  }

  /**
   * Add an equation
   */
  void addEquation(Equation eq) {
    if (eq.enabled && !eq.bi.isTrigger && !eq.bj.isTrigger) {
      equations.add(eq);
    }
  }

  /**
   * Remove an equation
   */
  void removeEquation(Equation eq) {
    final eqs = equations;
    int i = eqs.indexOf(eq);
    if (i != -1) {
      eqs.splice(i, 1);
    }
  }

  /**
   * Add all equations
   */
  void removeAllEquations() {
    this.equations.length = 0;
  }
}
