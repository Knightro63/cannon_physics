import '../solver/solver.dart';
import '../world/world_class.dart';

/// Constraint equation Gauss-Seidel solver.
/// @todo The spook parameters should be specified for each constraint, not globally.
/// @see https://www8.cs.umu.se/kurser/5DV058/VT09/lectures/spooknotes.pdf
class GSSolver extends Solver {
  /// When tolerance is reached, the system is assumed to be converged.
  double tolerance;

  /// @todo remove useless constructor
  GSSolver({
    this.tolerance = 1e-7,
  }):super();

  // Just temporary number holders that we want to reuse each iteration.
  List<double?> gsSolverSolveLambda = [];
  List<double?> gsSolverSolveInvCs = [];
  List<double?> gsSolverSolveBs = [];

  /// Solve
  /// @return number of iterations performed
  @override
  int solve(double dt, World world) {
    int iter = 0;
    final int maxIter = iterations;
    final double tolSquared = tolerance * tolerance;
    final equations = this.equations;
    final int nEq = equations.length;
    final bodies = world.bodies;
    final int nBodies = bodies.length;
    final double h = dt;

    double? B;
    double? invC;
    double deltalambda;
    double deltalambdaTot;
    double gwlambda;
    double? lambdaj;

    // Update solve mass
    if (nEq != 0) {
      for (int i = 0; i != nBodies; i++) {
        bodies[i].updateSolveMassProperties();
      }
    }

    // Things that do not change during iteration can be computed once
    List<double?> invCs = gsSolverSolveInvCs;
    List<double?> bs = gsSolverSolveBs;
    List<double?> lambda = gsSolverSolveLambda;
    invCs.length = nEq;
    bs.length = nEq;
    lambda.length = nEq;
    for (int i = 0; i != nEq; i++) {
      final c = equations[i];
      lambda[i] = 0.0;
      bs[i] = c.computeB(h);
      invCs[i] = 1.0 / c.computeC();
    }

    if (nEq != 0) {
      // Reset vlambda
      for (int i = 0; i != nBodies; i++) {
        final b = bodies[i];
        final vlambda = b.vlambda;
        final wlambda = b.wlambda;
        vlambda.set(0, 0, 0);
        wlambda.set(0, 0, 0);
      }

      // Iterate over equations
      for (iter = 0; iter != maxIter; iter++) {
        // Accumulate the total error for each iteration.
        deltalambdaTot = 0.0;

        for (int j = 0; j != nEq; j++) {
          final c = equations[j];

          // Compute iteration
          B = bs[j];
          invC = invCs[j];
          lambdaj = lambda[j];
          if(B == null || invC == null || lambdaj == null || lambda[j] == null) break;
          gwlambda = c.computeGWlambda();
          deltalambda = invC * (B - gwlambda - c.eps * lambdaj);

          // Clamp if we are not within the min/max interval
          if (lambdaj + deltalambda < c.minForce) {
            deltalambda = c.minForce - lambdaj;
          } else if (lambdaj + deltalambda > c.maxForce) {
            deltalambda = c.maxForce - lambdaj;
          }
          lambda[j] = lambda[j]!+deltalambda;

          deltalambdaTot += deltalambda > 0.0 ? deltalambda : -deltalambda; // abs(deltalambda)

          c.addToWlambda(deltalambda);
        }

        // If the total error is small enough - stop iterate
        if (deltalambdaTot * deltalambdaTot < tolSquared) {
          break;
        }
      }

      // Add result to velocity
      for (int i = 0; i != nBodies; i++) {
        final b = bodies[i];
        final v = b.velocity;
        final w = b.angularVelocity;

        b.vlambda.vmul(b.linearFactor, b.vlambda);
        v.vadd(b.vlambda, v);

        b.wlambda.vmul(b.angularFactor, b.wlambda);
        w.vadd(b.wlambda, w);
      }

      // Set the `.multiplier` property of each equation
      int l = equations.length-1;
      double invDt = 1 / h;
      while(l >= 0) {
        equations[l].multiplier = lambda[l]! * invDt;
        l--;
      }
    }

    return iter;
  }
}
