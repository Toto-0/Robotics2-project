// Initialization of static-const data
template <typename TRAIT>
const typename iit::Ballbot::dyn::tpl::InverseDynamics<TRAIT>::ExtForces
iit::Ballbot::dyn::tpl::InverseDynamics<TRAIT>::zeroExtForces(Force::Zero());

template <typename TRAIT>
iit::Ballbot::dyn::tpl::InverseDynamics<TRAIT>::InverseDynamics(IProperties& inertia, MTransforms& transforms) :
    inertiaProps( & inertia ),
    xm( & transforms ),
    link_1_I(inertiaProps->getTensor_link_1() ),
    link_2_I(inertiaProps->getTensor_link_2() ),
    gripper_I(inertiaProps->getTensor_gripper() )
    {
#ifndef EIGEN_NO_DEBUG
    std::cout << "Robot Ballbot, InverseDynamics<TRAIT>::InverseDynamics()" << std::endl;
    std::cout << "Compiled with Eigen debug active" << std::endl;
#endif
    link_1_v.setZero();
    link_2_v.setZero();
    gripper_v.setZero();

    vcross.setZero();
}

template <typename TRAIT>
void iit::Ballbot::dyn::tpl::InverseDynamics<TRAIT>::id(
    JointState& jForces,
    const JointState& qd, const JointState& qdd,
    const ExtForces& fext)
{
    firstPass(qd, qdd, fext);
    secondPass(jForces);
}

template <typename TRAIT>
void iit::Ballbot::dyn::tpl::InverseDynamics<TRAIT>::G_terms(JointState& jForces)
{
    // Link 'link_1'
    link_1_a = (xm->fr_link_1_X_fr_world).col(iit::rbd::LY) * Scalar(iit::rbd::g);
    link_1_f = link_1_I * link_1_a;
    // Link 'link_2'
    link_2_a = (xm->fr_link_2_X_fr_link_1) * link_1_a;
    link_2_f = link_2_I * link_2_a;
    // Link 'gripper'
    gripper_a = (xm->fr_gripper_X_fr_link_2) * link_2_a;
    gripper_f = gripper_I * gripper_a;

    secondPass(jForces);

}

template <typename TRAIT>
void iit::Ballbot::dyn::tpl::InverseDynamics<TRAIT>::C_terms(JointState& jForces, const JointState& qd)
{
    // Link 'link_1'
    link_1_v(iit::rbd::AZ) = qd(JOINT_1);   // link_1_v = vJ, for the first link of a fixed base robot
    
    link_1_f = iit::rbd::vxIv(qd(JOINT_1), link_1_I);
    
    // Link 'link_2'
    link_2_v = ((xm->fr_link_2_X_fr_link_1) * link_1_v);
    link_2_v(iit::rbd::AZ) += qd(JOINT_2);
    
    iit::rbd::motionCrossProductMx<Scalar>(link_2_v, vcross);
    
    link_2_a = (vcross.col(iit::rbd::AZ) * qd(JOINT_2));
    
    link_2_f = link_2_I * link_2_a + iit::rbd::vxIv(link_2_v, link_2_I);
    
    // Link 'gripper'
    gripper_v = ((xm->fr_gripper_X_fr_link_2) * link_2_v);
    gripper_v(iit::rbd::AZ) += qd(JOINT_3);
    
    iit::rbd::motionCrossProductMx<Scalar>(gripper_v, vcross);
    
    gripper_a = (xm->fr_gripper_X_fr_link_2) * link_2_a + vcross.col(iit::rbd::AZ) * qd(JOINT_3);
    
    gripper_f = gripper_I * gripper_a + iit::rbd::vxIv(gripper_v, gripper_I);
    

    secondPass(jForces);
}

template <typename TRAIT>
void iit::Ballbot::dyn::tpl::InverseDynamics<TRAIT>::firstPass(const JointState& qd, const JointState& qdd, const ExtForces& fext)
{

     // First pass, link 'link_1'
    link_1_a = (xm->fr_link_1_X_fr_world).col(iit::rbd::LY) * Scalar(iit::rbd::g);
    link_1_a(iit::rbd::AZ) += qdd(JOINT_1);
    link_1_v(iit::rbd::AZ) = qd(JOINT_1);   // link_1_v = vJ, for the first link of a fixed base robot
    
    link_1_f = link_1_I * link_1_a + iit::rbd::vxIv(qd(JOINT_1), link_1_I)  - fext[LINK_1];
    
    // First pass, link 'link_2'
    link_2_v = ((xm->fr_link_2_X_fr_link_1) * link_1_v);
    link_2_v(iit::rbd::AZ) += qd(JOINT_2);
    
    iit::rbd::motionCrossProductMx<Scalar>(link_2_v, vcross);
    
    link_2_a = (xm->fr_link_2_X_fr_link_1) * link_1_a + vcross.col(iit::rbd::AZ) * qd(JOINT_2);
    link_2_a(iit::rbd::AZ) += qdd(JOINT_2);
    
    link_2_f = link_2_I * link_2_a + iit::rbd::vxIv(link_2_v, link_2_I) - fext[LINK_2];
    
    // First pass, link 'gripper'
    gripper_v = ((xm->fr_gripper_X_fr_link_2) * link_2_v);
    gripper_v(iit::rbd::AZ) += qd(JOINT_3);
    
    iit::rbd::motionCrossProductMx<Scalar>(gripper_v, vcross);
    
    gripper_a = (xm->fr_gripper_X_fr_link_2) * link_2_a + vcross.col(iit::rbd::AZ) * qd(JOINT_3);
    gripper_a(iit::rbd::AZ) += qdd(JOINT_3);
    
    gripper_f = gripper_I * gripper_a + iit::rbd::vxIv(gripper_v, gripper_I) - fext[GRIPPER];
    
}

template <typename TRAIT>
void iit::Ballbot::dyn::tpl::InverseDynamics<TRAIT>::secondPass(JointState& jForces)
{

    // Link 'gripper'
    jForces(JOINT_3) = gripper_f(iit::rbd::AZ);
    link_2_f += xm->fr_gripper_X_fr_link_2.transpose() * gripper_f;
    // Link 'link_2'
    jForces(JOINT_2) = link_2_f(iit::rbd::AZ);
    link_1_f += xm->fr_link_2_X_fr_link_1.transpose() * link_2_f;
    // Link 'link_1'
    jForces(JOINT_1) = link_1_f(iit::rbd::AZ);
}
