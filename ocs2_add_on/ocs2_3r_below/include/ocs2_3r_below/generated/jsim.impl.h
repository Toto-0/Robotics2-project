

//Implementation of default constructor
template <typename TRAIT>
iit::Ballbot::dyn::tpl::JSIM<TRAIT>::JSIM(IProperties& inertiaProperties, FTransforms& forceTransforms) :
    linkInertias(inertiaProperties),
    frcTransf( &forceTransforms ),
    gripper_Ic(linkInertias.getTensor_gripper())
{
    //Initialize the matrix itself
    this->setZero();
}

#define DATA tpl::JSIM<TRAIT>::operator()

template <typename TRAIT>
const typename iit::Ballbot::dyn::tpl::JSIM<TRAIT>& iit::Ballbot::dyn::tpl::JSIM<TRAIT>::update(const JointState& state) {
    ForceVector F;

    // Precomputes only once the coordinate transforms:
    frcTransf -> fr_link_2_X_fr_gripper(state);
    frcTransf -> fr_link_1_X_fr_link_2(state);

    // Initializes the composite inertia tensors
    link_1_Ic = linkInertias.getTensor_link_1();
    link_2_Ic = linkInertias.getTensor_link_2();

    // "Bottom-up" loop to update the inertia-composite property of each link, for the current configuration

    // Link gripper:
    iit::rbd::transformInertia<Scalar>(gripper_Ic, frcTransf -> fr_link_2_X_fr_gripper, Ic_spare);
    link_2_Ic += Ic_spare;

    F = gripper_Ic.col(iit::rbd::AZ);
    DATA(JOINT_3, JOINT_3) = F(iit::rbd::AZ);

    F = frcTransf -> fr_link_2_X_fr_gripper * F;
    DATA(JOINT_3, JOINT_2) = F(iit::rbd::AZ);
    DATA(JOINT_2, JOINT_3) = DATA(JOINT_3, JOINT_2);
    F = frcTransf -> fr_link_1_X_fr_link_2 * F;
    DATA(JOINT_3, JOINT_1) = F(iit::rbd::AZ);
    DATA(JOINT_1, JOINT_3) = DATA(JOINT_3, JOINT_1);
    

    // Link link_2:
    iit::rbd::transformInertia<Scalar>(link_2_Ic, frcTransf -> fr_link_1_X_fr_link_2, Ic_spare);
    link_1_Ic += Ic_spare;

    F = link_2_Ic.col(iit::rbd::AZ);
    DATA(JOINT_2, JOINT_2) = F(iit::rbd::AZ);

    F = frcTransf -> fr_link_1_X_fr_link_2 * F;
    DATA(JOINT_2, JOINT_1) = F(iit::rbd::AZ);
    DATA(JOINT_1, JOINT_2) = DATA(JOINT_2, JOINT_1);

    // Link link_1:

    F = link_1_Ic.col(iit::rbd::AZ);
    DATA(JOINT_1, JOINT_1) = F(iit::rbd::AZ);


    return *this;
}

#undef DATA
#undef F

template <typename TRAIT>
void iit::Ballbot::dyn::tpl::JSIM<TRAIT>::computeL() {
    L = this -> template triangularView<Eigen::Lower>();
    // Joint joint_3, index 2 :
    L(2, 2) = std::sqrt(L(2, 2));
    L(2, 1) = L(2, 1) / L(2, 2);
    L(2, 0) = L(2, 0) / L(2, 2);
    L(1, 1) = L(1, 1) - L(2, 1) * L(2, 1);
    L(1, 0) = L(1, 0) - L(2, 1) * L(2, 0);
    L(0, 0) = L(0, 0) - L(2, 0) * L(2, 0);
    
    // Joint joint_2, index 1 :
    L(1, 1) = std::sqrt(L(1, 1));
    L(1, 0) = L(1, 0) / L(1, 1);
    L(0, 0) = L(0, 0) - L(1, 0) * L(1, 0);
    
    // Joint joint_1, index 0 :
    L(0, 0) = std::sqrt(L(0, 0));
    
}

template <typename TRAIT>
void iit::Ballbot::dyn::tpl::JSIM<TRAIT>::computeInverse() {
    computeLInverse();

    inverse(0, 0) =  + (Linv(0, 0) * Linv(0, 0));
    inverse(1, 1) =  + (Linv(1, 0) * Linv(1, 0)) + (Linv(1, 1) * Linv(1, 1));
    inverse(1, 0) =  + (Linv(1, 0) * Linv(0, 0));
    inverse(0, 1) = inverse(1, 0);
    inverse(2, 2) =  + (Linv(2, 0) * Linv(2, 0)) + (Linv(2, 1) * Linv(2, 1)) + (Linv(2, 2) * Linv(2, 2));
    inverse(2, 1) =  + (Linv(2, 0) * Linv(1, 0)) + (Linv(2, 1) * Linv(1, 1));
    inverse(1, 2) = inverse(2, 1);
    inverse(2, 0) =  + (Linv(2, 0) * Linv(0, 0));
    inverse(0, 2) = inverse(2, 0);
}

template <typename TRAIT>
void iit::Ballbot::dyn::tpl::JSIM<TRAIT>::computeLInverse() {
    //assumes L has been computed already
    Linv(0, 0) = 1 / L(0, 0);
    Linv(1, 1) = 1 / L(1, 1);
    Linv(2, 2) = 1 / L(2, 2);
    Linv(1, 0) = - Linv(0, 0) * ((Linv(1, 1) * L(1, 0)) + 0);
    Linv(2, 1) = - Linv(1, 1) * ((Linv(2, 2) * L(2, 1)) + 0);
    Linv(2, 0) = - Linv(0, 0) * ((Linv(2, 1) * L(1, 0)) + (Linv(2, 2) * L(2, 0)) + 0);
}

