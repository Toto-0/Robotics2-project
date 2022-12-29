
// Initialization of static-const data
template <typename TRAIT>
const typename iit::Ballbot::dyn::tpl::ForwardDynamics<TRAIT>::ExtForces
    iit::Ballbot::dyn::tpl::ForwardDynamics<TRAIT>::zeroExtForces(Force::Zero());

template <typename TRAIT>
iit::Ballbot::dyn::tpl::ForwardDynamics<TRAIT>::ForwardDynamics(iit::Ballbot::dyn::tpl::InertiaProperties<TRAIT>& inertia, MTransforms& transforms) :
    inertiaProps( & inertia ),
    motionTransforms( & transforms )
{
    link_1_v.setZero();
    link_1_c.setZero();
    link_2_v.setZero();
    link_2_c.setZero();
    gripper_v.setZero();
    gripper_c.setZero();

    vcross.setZero();
    Ia_r.setZero();

}

template <typename TRAIT>
void iit::Ballbot::dyn::tpl::ForwardDynamics<TRAIT>::fd(
    JointState& qdd,
    const JointState& qd,
    const JointState& tau,
    const ExtForces& fext/* = zeroExtForces */)
{
    
    link_1_AI = inertiaProps->getTensor_link_1();
    link_1_p = - fext[LINK_1];
    link_2_AI = inertiaProps->getTensor_link_2();
    link_2_p = - fext[LINK_2];
    gripper_AI = inertiaProps->getTensor_gripper();
    gripper_p = - fext[GRIPPER];
    //std::cout << link_1_AI << std::endl;
    // ---------------------- FIRST PASS ---------------------- //
    // Note that, during the first pass, the articulated inertias are really
    //  just the spatial inertia of the links (see assignments above).
    //  Afterwards things change, and articulated inertias shall not be used
    //  in functions which work specifically with spatial inertias.
    
    // + Link link_1
    //  - The spatial velocity:
    link_1_v(iit::rbd::AZ) = qd(JOINT_1);
    
    //  - The bias force term:
    link_1_p += iit::rbd::vxIv(qd(JOINT_1), link_1_AI);
    
    // + Link link_2
    //  - The spatial velocity:
    link_2_v = (motionTransforms-> fr_link_2_X_fr_link_1) * link_1_v;
    link_2_v(iit::rbd::AZ) += qd(JOINT_2);
    
    //  - The velocity-product acceleration term:
    iit::rbd::motionCrossProductMx<Scalar>(link_2_v, vcross);
    link_2_c = vcross.col(iit::rbd::AZ) * qd(JOINT_2);
    
    //  - The bias force term:
    link_2_p += iit::rbd::vxIv(link_2_v, link_2_AI);
    
    // + Link gripper
    //  - The spatial velocity:
    gripper_v = (motionTransforms-> fr_gripper_X_fr_link_2) * link_2_v;
    gripper_v(iit::rbd::AZ) += qd(JOINT_3);
    
    //  - The velocity-product acceleration term:
    iit::rbd::motionCrossProductMx<Scalar>(gripper_v, vcross);
    gripper_c = vcross.col(iit::rbd::AZ) * qd(JOINT_3);
    
    //  - The bias force term:
    gripper_p += iit::rbd::vxIv(gripper_v, gripper_AI);

    
    // ---------------------- SECOND PASS ---------------------- //
    Matrix66S IaB;
    Force pa;
    
    // + Link base
    gripper_u = tau(JOINT_3) - gripper_p(iit::rbd::AZ);
    gripper_U = gripper_AI.col(iit::rbd::AZ);
    gripper_D = gripper_U(iit::rbd::AZ);
    
    iit::rbd::compute_Ia_revolute(gripper_AI, gripper_U, gripper_D, Ia_r);  // same as: Ia_r = gripper_AI - gripper_U/gripper_D * gripper_U.transpose();
    pa = gripper_p + Ia_r * gripper_c + gripper_U * gripper_u/gripper_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_gripper_X_fr_link_2, IaB);
    link_2_AI += IaB;
    link_2_p += (motionTransforms-> fr_gripper_X_fr_link_2).transpose() * pa;
    
    // + Link link_2
    link_2_u = tau(JOINT_2) - link_2_p(iit::rbd::AZ);
    link_2_U = link_2_AI.col(iit::rbd::AZ);
    link_2_D = link_2_U(iit::rbd::AZ);
    
    
    iit::rbd::compute_Ia_revolute(link_2_AI, link_2_U, link_2_D, Ia_r);  // same as: Ia_r = link_2_AI - link_2_U/link_2_D * link_2_U.transpose();
    pa = link_2_p + Ia_r * link_2_c + link_2_U * link_2_u/link_2_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_link_2_X_fr_link_1, IaB);
    link_1_AI += IaB;
    link_1_p += (motionTransforms-> fr_link_2_X_fr_link_1).transpose() * pa;
    
    // + Link link_1
    link_1_u = tau(JOINT_1) - link_1_p(iit::rbd::AZ);
    link_1_U = link_1_AI.col(iit::rbd::AZ);
    link_1_D = link_1_U(iit::rbd::AZ);
    
    
    // ---------------------- THIRD PASS ---------------------- //
    link_1_a = (motionTransforms-> fr_link_1_X_fr_world).col(iit::rbd::LZ) * Scalar(iit::rbd::g);
    qdd(JOINT_1) = (link_1_u - link_1_U.dot(link_1_a)) / link_1_D;
    link_1_a(iit::rbd::AZ) += qdd(JOINT_1);
    
    link_2_a = (motionTransforms-> fr_link_2_X_fr_link_1) * link_1_a + link_2_c;
    qdd(JOINT_2) = (link_2_u - link_2_U.dot(link_2_a)) / link_2_D;
    link_2_a(iit::rbd::AZ) += qdd(JOINT_2);
    
    gripper_a = (motionTransforms-> fr_gripper_X_fr_link_2) * link_2_a + gripper_c;
    qdd(JOINT_3) = (gripper_u - gripper_U.dot(gripper_a)) / gripper_D;
    gripper_a(iit::rbd::AZ) += qdd(JOINT_3);
    
}
