import '../solver/solver.dart';
import '../objects/body.dart';
import '../equations/equation.dart';
import '../world/world.dart' hide Body;
import './gs_solver.dart';

//type SplitSolverNode = { body: Body | null; children: SplitSolverNode[]; eqs: Equation[]; visited: boolean }

class SplitSolverNode{
  //SplitSolverNode();
  Body? body;
  List children;
  List<Equation> eqs;
  bool visited = false;
}

/**
 * Splits the equations into islands and solves them independently. Can improve performance.
 */
class SplitSolver extends Solver {
  /**
   * The number of solver iterations determines quality of the constraints in the world. The more iterations, the more correct simulation. More iterations need more computations though. If you have a large gravity force in your world, you will need more iterations.
   */
  int iterations = 10;

  /**
   * When tolerance is reached, the system is assumed to be converged.
   */
  double tolerance = 1e-7;
  /** subsolver */
  GSSolver subsolver;
  List<SplitSolverNode> nodes = [];
  List<SplitSolverNode> nodePool = [];

  SplitSolver(this.subsolver):super(){
    // Create needed nodes, reuse if possible
    while (nodePool.length < 128) {
      nodePool.add(this.createNode());
    }
  }

  /**
   * Solve the subsystems
   * @return number of iterations performed
   */
  @override
  int solve(double dt, World world) {
    const nodes = SplitSolver_solve_nodes
    const nodePool = this.nodePool
    const bodies = world.bodies
    const equations = this.equations
    const Neq = equations.length
    const Nbodies = bodies.length
    const subsolver = this.subsolver

    // Create needed nodes, reuse if possible
    while (nodePool.length < Nbodies) {
      nodePool.push(this.createNode())
    }
    nodes.length = Nbodies
    for (let i = 0; i < Nbodies; i++) {
      nodes[i] = nodePool[i]
    }

    // Reset node values
    for (let i = 0; i !== Nbodies; i++) {
      const node = nodes[i]
      node.body = bodies[i]
      node.children.length = 0
      node.eqs.length = 0
      node.visited = false
    }
    for (let k = 0; k !== Neq; k++) {
      const eq = equations[k]
      const i = bodies.indexOf(eq.bi)
      const j = bodies.indexOf(eq.bj)
      const ni = nodes[i]
      const nj = nodes[j]
      ni.children.push(nj)
      ni.eqs.push(eq)
      nj.children.push(ni)
      nj.eqs.push(eq)
    }

    let child: SplitSolverNode | false
    let n = 0
    let eqs = SplitSolver_solve_eqs

    subsolver.tolerance = this.tolerance
    subsolver.iterations = this.iterations

    const dummyWorld = SplitSolver_solve_dummyWorld
    while ((child = getUnvisitedNode(nodes))) {
      eqs.length = 0
      dummyWorld.bodies.length = 0
      bfs(child, visitFunc, dummyWorld.bodies, eqs)

      const Neqs = eqs.length

      eqs = eqs.sort(sortById)

      for (let i = 0; i !== Neqs; i++) {
        subsolver.addEquation(eqs[i])
      }

      const iter = subsolver.solve(dt, dummyWorld as World)
      subsolver.removeAllEquations()
      n++
    }

    return n
  }

  // Returns the number of subsystems
  List<SplitSolverNode> SplitSolver_solve_nodes = []; // All allocated node objects
  const SplitSolver_solve_nodePool: SplitSolverNode[] = []; // All allocated node objects
  const SplitSolver_solve_eqs: Equation[] = []; // Temp array
  const SplitSolver_solve_bds: Body[] = []; // Temp array
  const SplitSolver_solve_dummyWorld: { bodies: Body[] } = { bodies: [] }; // Temp object

  const STATIC = Body.STATIC;

  SplitSolverNode? getUnvisitedNode(List<SplitSolverNode> nodes){
    int nNodes = nodes.length;
    for (int i = 0; i != nNodes; i++) {
      final node = nodes[i];
      if (!node.visited && !(node.body!.type & STATIC)) {
        return node;
      }
    }
    return null;
  }

  const queue: SplitSolverNode[] = []

  void bfs(
    root: SplitSolverNode,
    visitFunc: (node: SplitSolverNode, bds: (Body | null)[], eqs: Equation[]) => void,
    bds: (Body | null)[],
    eqs: Equation[]
  ) {
    queue.push(root)
    root.visited = true
    visitFunc(root, bds, eqs)
    while (queue.length) {
      const node = queue.pop()!
      // Loop over unvisited child nodes
      let child: SplitSolverNode | false
      while ((child = getUnvisitedNode(node.children))) {
        child.visited = true
        visitFunc(child, bds, eqs)
        queue.push(child)
      }
    }
  }

  void visitFunc(SplitSolverNode node, List<Body?> bds, List<Equation> eqs) {
    bds.push(node.body)
    const Neqs = node.eqs.length
    for (let i = 0; i !== Neqs; i++) {
      const eq = node.eqs[i]
      if (!eqs.includes(eq)) {
        eqs.push(eq)
      }
    }
  }

  int sortById(a: { id: number }, b: { id: number }) {
    return b.id - a.id;
  }
}
