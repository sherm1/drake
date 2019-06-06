#pragma once

#include <functional>
#include <string>
#include <utility>

#include "drake/common/autodiff.h"
#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_bool.h"
#include "drake/common/drake_optional.h"
#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"
#include "drake/common/type_safe_index.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/value_to_abstract_value.h"

namespace drake {
namespace systems {

template <typename T>
class System;


/** Represents the general concept of a System variable, including state
variables and parameters.

@tparam T The vector element type, which must be a valid Eigen scalar.

Instantiated templates for the following kinds of T's are provided:

- double
- AutoDiffXd
- symbolic::Expression

They are already available to link against in the containing library. */
template <typename T>
class SystemVariable final {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SystemVariable)

  /** (Advanced) Constructs a variable of the given unique name, that is owned
  by the given system. The system must outlive this object. */
  SystemVariable(std::string name, const System<T>* system)
      : name_(std::move(name)), system_(*system) {}

  /** Obtains the vector value for a numeric variable.
  @throws std::logic_error if this variable is not numeric. */
  const VectorX<T>& value(const Context<T>& context) const {
    return context.get_variable_value(index());
  }

  /** Obtains the value for this variable as the specified type.
  @throws std::logic_error if the value can't be returned as this type. */
  template <typename ValueType>
  const ValueType& value(const Context<T>& context) const {
    return context.template get_variable_value<ValueType>(index());
  }

  /** Obtains the vector value of a numeric variable overlaid with a struct
  permitting named access through the struct's members. The size of the struct
  must match the length of the vector. Example: @code
    // Assume v is a SystemVariable.
    struct States { double x1, x2; };
    const States& states = v.value_overlay<States>(context);
    double sum = states.x1 + states.x2;
  @endcode
  */
  template <template<typename> class Overlay>
  const Overlay<T>& value_overlay(const Context<T>& context) const {
    const VectorX<T>& vector_value = value(context);
    if (sizeof(Overlay<T>) != vector_value.size() * sizeof(T))
      throw std::logic_error("Overlay size didn't match variable size.");
    return *reinterpret_cast<const Overlay<T>*>(vector_value.data());
  }

  // This is just like FixValue() for an input port.
  /** Sets the value for this variable, which must have an appropriate type. */
  template <typename ValueType>
  void SetValue(Context<T>* context, const ValueType& var_value) const {
    DRAKE_DEMAND(context != nullptr);
    DRAKE_ASSERT_VOID(system().ThrowIfContextNotCompatible(*context));
    const bool is_numeric_variable = (data_type() == kVectorValued);
    std::unique_ptr<AbstractValue> abstract_value =
        is_numeric_variable
        ? internal::ValueToVectorValue<T>::ToAbstract(__func__, var_value)
        : internal::ValueToAbstractValue::ToAbstract(__func__, var_value);
    return context->SetVariableValue(index(), std::move(abstract_value));
  }

  // TODO is this the diagram or the defining leafsystem?
  /** Returns a reference to the System that owns this variable. */
  const System<T>& system() const { return system_; }
  SystemVariableIndex index() const { return index_; }
  PortDataType data_type() const { return data_type_; }

 private:
  const std::string name_;
  PortDataType data_type_;
  std::unique_ptr<AbstractValue> model_value_;

  const System<T>& system_;  // The system that owns this variable.
  SystemVariableIndex index_;  // Index within the owning system.
};

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::SystemVariable)
