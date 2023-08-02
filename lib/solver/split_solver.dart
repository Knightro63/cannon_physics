import '../solver/solver.dart';
import '../objects/body.dart';
import '../equations/equation.dart';
import '../world/world_class.dart';
import './gs_solver.dart';

//type SplitSolverNode = { body: Body | null; children: SplitSolverNode[]; eqs: Equation[]; visited: boolean }

class SplitSolverNode{
  SplitSolverNode({
    this.body,
    this.children = const [],
    this.eqs = const [],
    this.visited = false
  });
  Body? body;
  List<SplitSolverNode> children;
  List<Equation> eqs;
  bool visited;
}


/// Splits the equations into islands and solves them independently. Can improve performance.
class SplitSolver extends Solver {
  /// When tolerance is reached, the system is assumed to be converged.
  double tolerance = 1e-7;
  /// subsolver
  GSSolver subsolver;
  List<SplitSolverNode> nodes = [];
  List<SplitSolverNode> nodePool = [];

  SplitSolver(this.subsolver):super(){
    // Create needed nodes, reuse if possible
    while (nodePool.length < 128) {
      nodePool.add(createNode());
    }
  }

  // Returns the number of subsystems
  final List<SplitSolverNode> _splitSolverSolveNodes = []; // All allocated node objects
  //final List<SplitSolverNode> _splitSolverSolveNodePool = []; // All allocated node objects
  final List<Equation> _splitSolverSolveEqs = []; // Temp array
  //final List<Body> _splitSolverSolveBds = []; // Temp array
  final List<Body> _splitSolverSolveDummyWorld = [];// { bodies: Body[] } = { bodies: [] }; // Temp object
  final List<SplitSolverNode> _queue = [];

  /// Solve the subsystems
  /// @return number of iterations performed
  @override
  int solve(double dt, World world) {
    final nodes = _splitSolverSolveNodes;
    final nodePool = this.nodePool;
    final bodies = world.bodies;
    final equations = this.equations;
    final neq = equations.length;
    final nbodies = bodies.length;
    final subsolver = this.subsolver;

    // Create needed nodes, reuse if possible
    while (nodePool.length < nbodies) {
      nodePool.add(createNode());
    }
    nodes.length = nbodies;
    for (int i = 0; i < nbodies; i++) {
      nodes[i] = nodePool[i];
    }

    // Reset node values
    for (int i = 0; i != nbodies; i++) {
      final node = nodes[i];
      node.body = bodies[i];
      node.children.length = 0;
      node.eqs.length = 0;
      node.visited = false;
    }
    for (int k = 0; k != neq; k++) {
      final eq = equations[k];
      final i = bodies.indexOf(eq.bi);
      final j = bodies.indexOf(eq.bj);
      final ni = nodes[i];
      final nj = nodes[j];
      ni.children.add(nj);
      ni.eqs.add(eq);
      nj.children.add(ni);
      nj.eqs.add(eq);
    }

    //:  | false;
    int n = 0;
    List<Equation> eqs = _splitSolverSolveEqs;

    subsolver.tolerance = tolerance;
    subsolver.iterations = iterations;

    final dummyWorld = _splitSolverSolveDummyWorld;
    while(true) {
      SplitSolverNode? child = getUnvisitedNode(nodes);
      if(child == null) break;
      eqs.length = 0;
      dummyWorld.length = 0;
      bfs(child, visitFunc, dummyWorld, eqs);

      final neqs = eqs.length;

      //eqs = eqs.sort(sortById);
      eqs.sort(sortById);

      for (int i = 0; i != neqs; i++) {
        subsolver.addEquation(eqs[i]);
      }

      //final iter = subsolver.solve(dt, dummyWorld as World);
      subsolver.removeAllEquations();
      n++;
    }

    return n;
  }

  SplitSolverNode? getUnvisitedNode(List<SplitSolverNode> nodes){
    int nNodes = nodes.length;
    for (int i = 0; i != nNodes; i++) {
      final node = nodes[i];
      if (!node.visited && node.body?.type != BodyTypes.static) {
        return node;
      }
    }
    return null;
  }
  SplitSolverNode createNode(){
      return SplitSolverNode();//{ body:null, children:[], eqs:[], visited:false };
  }
  void bfs(
    SplitSolverNode root,
    void Function(SplitSolverNode node, List<Body?> bds, List<Equation> eqs) visitFunc,
    List<Body?> bds,
    List<Equation> eqs
  ) {
    _queue.add(root);
    root.visited = true;
    visitFunc(root, bds, eqs);
    while (_queue.isNotEmpty) {
      final node = _queue.removeLast();
      while(true) {
        SplitSolverNode? child = getUnvisitedNode(node.children);
        if(child == null) break;
        child.visited = true;
        visitFunc(child, bds, eqs);
        _queue.add(child);
      }
    }
  }

  void visitFunc(SplitSolverNode node, List<Body?> bds, List<Equation> eqs) {
    bds.add(node.body);
    final neqs = node.eqs.length;
    for (int i = 0; i != neqs; i++) {
      final eq = node.eqs[i];
      if (!eqs.contains(eq)) {
        eqs.add(eq);
      }
    }
  }

  int sortById(Equation a, Equation b) {
    return b.id - a.id;
  }
}