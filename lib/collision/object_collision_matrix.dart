import '../objects/body.dart';

/// Records what objects are colliding with each other
class ObjectCollisionMatrix {
  /// The matrix storage.
  Map<String, bool> matrix = {};

  ObjectCollisionMatrix(Map<String, bool>? matrix){
    this.matrix = matrix ?? {};
  }

  /// get
  bool get(Body bi, Body bj) {
    int i = bi.id;
    int j = bj.id;
    if (j > i) {
      final temp = j;
      j = i;
      i = temp;
    }
    return matrix.containsKey('$i-$j');
  }

  /// set
  void set(Body bi, Body bj, bool value) {
    int i = bi.id;
    int j = bj.id;
    if (j > i) {
      final temp = j;
      j = i;
      i = temp;
    }
    if (value) {
      matrix['$i-$j'] = true;
    } else {
      matrix.remove('$i-$j');
    }
  }

  /// Empty the matrix
  void reset() {
    matrix.clear();
  }

  /// Set max number of objects
  void setNumObjects(int n) {}
}
