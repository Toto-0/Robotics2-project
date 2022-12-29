#include "miscellaneous.h"
#include <iit/rbd/utils.h>

using namespace iit::Ballbot;
using namespace iit::Ballbot::dyn;

iit::rbd::Vector3d iit::Ballbot::getWholeBodyCOM(const InertiaProperties& inertiaProps, const HomogeneousTransforms& ht) {
  iit::rbd::Vector3d tmpSum(iit::rbd::Vector3d::Zero());

  HomogeneousTransforms::MatrixType tmpX(HomogeneousTransforms::MatrixType::Identity());
  tmpX = tmpX * ht.fr_world_X_fr_link_1;
  tmpSum += inertiaProps.getMass_link_1() *
          ( iit::rbd::Utils::transform(tmpX, inertiaProps.getCOM_link_1()));
  
  tmpX = tmpX * ht.fr_link_1_X_fr_link_2;
  tmpSum += inertiaProps.getMass_link_2() *
          ( iit::rbd::Utils::transform(tmpX, inertiaProps.getCOM_link_2()));
  
  tmpX = tmpX * ht.fr_link_2_X_fr_gripper;
  tmpSum += inertiaProps.getMass_gripper() *
          ( iit::rbd::Utils::transform(tmpX, inertiaProps.getCOM_gripper()));
  
  return tmpSum / inertiaProps.getTotalMass();
}

iit::rbd::Vector3d iit::Ballbot::getWholeBodyCOM(const InertiaProperties& inertiaProps, const JointState& q, HomogeneousTransforms& ht) {
  // First updates the coordinate transforms that will be used by the routine
  ht.fr_world_X_fr_link_1(q);
  ht.fr_link_1_X_fr_link_2(q);
  ht.fr_link_2_X_fr_gripper(q);

  // The actual calculus
  return getWholeBodyCOM(inertiaProps, ht);
}
