#include "boost/serialization/binary_object.hpp"

template<class T>
const T& Blackboard::read(const T *component) {
   return *component;
}

template<class T>
void Blackboard::write(T *component, const T& value) {
   *component = value;
}
