import '../objects/body.dart';

/**
 * Collision "matrix".
 * It's actually a triangular-shaped array of whether two bodies are touching this step, for reference next step
 */
class ArrayCollisionMatrix {
  /**
   * The matrix storage.
   */
  List<num> matrix;

  ArrayCollisionMatrix([this.matrix = const []]);

  /**
   * Get an element
   */
  num get(Body bi, Body  bj) {
    int i = bi.index;//{ index: i } 
    int j = bj.index;//{ index: j }
    if (j > i) {
      final temp = j;
      j = i;
      i = temp;
    }
    return this.matrix[((i * (i + 1)) >> 1) + j - 1];
  }

  /**
   * Set an element
   */
  void set(Body bi, Body bj, bool value) {
    int i = bi.index;//{ index: i }
    int j = bj.index;//{ index: j }
    if (j > i) {
      final temp = j;
      j = i;
      i = temp;
    }
    matrix[((i * (i + 1)) >> 1) + j - 1] = value ? 1 : 0;
  }

  /**
   * Sets all elements to zero
   */
  void reset() {
    for (int i = 0, l = matrix.length; i != l; i++) {
      matrix[i] = 0;
    }
  }

  
  /// Sets the max number of objects
  void setNumObjects(int n) {
    matrix.length = (n * (n - 1)) >> 1;
  }
}
