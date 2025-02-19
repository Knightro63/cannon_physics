import 'package:vector_math/vector_math.dart';

/// For pooling objects that can be reused.
class Pool {
  /// he objects array.
  List objects = [];
  /// The type of the objects.
  Object type = Object();//: any = Object

  /// Release an object after use
  Pool release([List? args]){
    args ??= [];
    final nargs = args.length;
    for (int i = 0; i != nargs; i++) {
      objects.add(args[i]);
    }
    return this;
  }

  /// Get an object
  Vector3 get() {
    if (objects.isEmpty) {
      return constructObject();
    } else {
      return objects.removeLast();
    }
  }

  /// Construct an object. Should be implemented in each subclass.
  Vector3 constructObject(){
    throw ('constructObject() not implemented in this Pool subclass yet!');
  }

  /// @return Self, for chaining
  Pool resize(num size) {
    final objects = this.objects;

    while (objects.length > size) {
      objects.removeLast();
    }

    while (objects.length < size) {
      objects.add(constructObject());
    }

    return this;
  }
}
