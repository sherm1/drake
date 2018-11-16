#include "drake/systems/primitives/signal_logger.h"

#include "drake/common/default_scalars.h"

namespace drake {
namespace systems {

template <typename T>
SignalLogger<T>::SignalLogger(int input_size, int batch_allocation_size)
    : log_(input_size, batch_allocation_size) {
  this->DeclareInputPort("data", kVectorValued, input_size);

  // Unconditionally log the initial conditions.
  this->DeclareInitializationEvent(PublishEvent<T>(
      [this](const Context<T>& context, const PublishEvent<T>&) {
        this->AddLogEntry(context);
      }));

  // If no period is set, we'll log every step.
  this->DeclarePerStepEvent(PublishEvent<T>(
      [this](const Context<T>& context, const PublishEvent<T>&) {
        if (!periodic_publish_) this->AddLogEntry(context);
      }));
}

template <typename T>
void SignalLogger<T>::set_publish_period(double period) {
  DRAKE_DEMAND(period > 0);
  periodic_publish_ = true;  // Disable per-step publishing.
  // We always log at initialization so the first periodic publish should
  // occur one period in.
  this->DeclarePeriodicEvent(period, period,
                             PublishEvent<T>([this](const Context<T>& context,
                                                    const PublishEvent<T>&) {
                               this->AddLogEntry(context);
                             }));
}

template <typename T>
const InputPort<T>& SignalLogger<T>::get_input_port()
const {
  return System<T>::get_input_port(0);
}

template <typename T>
void SignalLogger<T>::AddLogEntry(const Context<T>& context) const {
  log_.AddData(context.get_time(),
               this->EvalVectorInput(context, 0)->get_value());
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::SignalLogger)
