

#include <ocs2_3r_nom/dynamics/BallbotSystemDynamics.h>

// robcogen
#include <iit/rbd/rbd.h>
#include <iit/rbd/traits/TraitSelector.h>

#include "ocs2_3r_nom/generated/forward_dynamics.h"
#include "ocs2_3r_nom/generated/inertia_properties.h"
#include "ocs2_3r_nom/generated/inverse_dynamics.h"
#include "ocs2_3r_nom/generated/jsim.h"
#include "ocs2_3r_nom/generated/transforms.h"

namespace ocs2 {
namespace ballbot {

ad_vector_t BallbotSystemDynamics::systemFlowMap(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& input,
                                                 const ad_vector_t& parameters) const {
  // compute actuationMatrix S_transposed which appears in the equations: M(q)\dot v + h = S^(transpose)\tau
  iit::Ballbot::tpl::JointState<ad_scalar_t> qdd;
  iit::Ballbot::tpl::JointState<ad_scalar_t> g_part;

  using trait_t = typename iit::rbd::tpl::TraitSelector<ad_scalar_t>::Trait;
  iit::Ballbot::dyn::tpl::InertiaProperties<trait_t> inertias;
  iit::Ballbot::tpl::MotionTransforms<trait_t> transforms;
  iit::Ballbot::dyn::tpl::ForwardDynamics<trait_t> forward_dyn(inertias, transforms);
  iit::Ballbot::dyn::tpl::InverseDynamics<trait_t> inverse_dyn(inertias, transforms);
  forward_dyn.fd(qdd, state.head<3>(), state.tail<3>(), input);
  inverse_dyn.G_terms(g_part, state.head<3>());


  ad_vector_t stateDerivative(6);
  stateDerivative << state.tail<3>(), qdd;
  // dxdt
  //ad_vector_t stateDerivative(9);
  //stateDerivative << state.tail<3>(), qdd, g_part;
  return stateDerivative;
}

}  // namespace ballbot
}  // namespace ocs2
