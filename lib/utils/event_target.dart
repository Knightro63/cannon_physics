import '../objects/body.dart';
import '../shapes/shape.dart';
import '../equations/contact_equation.dart';

/// Base class for objects that dispatches events.
class EventTarget {
  Map<String,List<Function>>? _listeners;//private _listeners: Record<string, Function[]> | null;

  /// Add an event listener
  /// @return The self object, for chainability.
  EventTarget addEventListener(String type, Function(dynamic) listener) {
    _listeners ??= {};
    final listeners = _listeners;
    if (listeners?[type] == null) {
      listeners![type] = [];
    }
    if (!listeners![type]!.contains(listener)) {
      listeners[type]!.add(listener);
    }
    return this;
  }

  /// Check if an event listener is added
  bool hasEventListener(String type, void Function() listener) {
    if (_listeners == null) {
      return false;
    }
    final listeners = _listeners;
    if (listeners?[type] != null && listeners![type]!.contains(listener)) {
      return true;
    }
    return false;
  }

  /// Check if any event listener of the given type is added
  bool hasAnyEventListener(String type){
    if (_listeners == null) {
      return false;
    }
    final listeners = _listeners;
    return listeners?[type] != null;
  }

  /// Remove an event listener
  /// @return The self object, for chainability.
  EventTarget removeEventListener(String type, Function listener){
    if (_listeners == null) {
      return this;
    }
    final listeners = _listeners;
    if (listeners?[type] == null) {
      return this;
    }
    final index = listeners?[type]?.indexOf(listener);
    if (index != null && index != -1) {
      listeners?[type]?.removeAt(index);
    }
    return this;
  }

  /// Emit an event.
  /// @return The self object, for chainability.
  EventTarget dispatchEvent(Event event){
    if (_listeners == null) {
      return this;
    }
    
    final listeners = _listeners;
    final listenerArray = listeners?[event.type];
    //print(event.type);
    if (listenerArray != null) {
      //print(event.type);
      event.target = this;
      for (int i = 0, l = listenerArray.length; i < l; i++) {
        //print('LISTEN: ${listenerArray[i]}');
        listenerArray[i].call(event);
      }
    }
    return this;
  }
}

class Event{
  Event(this.type);
  final String type;
  EventTarget? target;

}

class CollideEvent extends Event{
  CollideEvent({
    type = 'collide',
    this.body,
    this.contact
  }):super(type);
  Body? body; 
  ContactEquation? contact;
}

class ContactEvent extends Event{
  ContactEvent({
    required type,
    this.bodyA,
    this.bodyB
  }):super(type);
  Body? bodyA;
  Body? bodyB;
}

class ShapeContactEvent extends Event{
  ShapeContactEvent({
    required type,
    this.bodyA,
    this.bodyB,
    this.shapeA,
    this.shapeB
  }):super(type);
  Body? bodyA;
  Body? bodyB;
  Shape? shapeA;
  Shape? shapeB;
}

class BodyEvent extends Event{
  BodyEvent({
    type = '',
    EventTarget? target
  }):super(type){
    this.target = target;
  }

  //Body? target;
}