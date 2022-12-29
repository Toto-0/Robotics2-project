template <typename TRAIT>
iit::Ballbot::dyn::tpl::InertiaProperties<TRAIT>::InertiaProperties()
{
    com_link_1 = iit::rbd::Vector3d(comx_link_1,0.0,0.0).cast<Scalar>();
    tensor_link_1.fill(
        Scalar(m_link_1),
        com_link_1,
        rbd::Utils::buildInertiaTensor(
                Scalar(ix_link_1),
                Scalar(iy_link_1),
                Scalar(iz_link_1),
                Scalar(0.0),
                Scalar(0.0),
                Scalar(0.0)) );

    com_link_2 = iit::rbd::Vector3d(comx_link_2,0.0,0.0).cast<Scalar>();
    tensor_link_2.fill(
        Scalar(m_link_2),
        com_link_2,
        rbd::Utils::buildInertiaTensor(
                Scalar(ix_link_2),
                Scalar(iy_link_2),
                Scalar(iz_link_2),
                Scalar(0.0),
                Scalar(0.0),
                Scalar(0.0)) );

    com_gripper = iit::rbd::Vector3d(comx_gripper,0.0,0.0).cast<Scalar>();
    tensor_gripper.fill(
        Scalar(m_gripper),
        com_gripper,
        rbd::Utils::buildInertiaTensor(
                Scalar(ix_gripper),
                Scalar(iy_gripper),
                Scalar(iz_gripper),
                Scalar(0.0),
                Scalar(0.0),
                Scalar(0.0)) );

}

