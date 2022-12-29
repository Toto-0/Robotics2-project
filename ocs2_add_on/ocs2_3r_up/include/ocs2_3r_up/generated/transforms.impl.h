
// Constructors
template <typename TRAIT>
iit::Ballbot::tpl::MotionTransforms<TRAIT>::MotionTransforms
    ()
     :
    fr_world_X_fr_gripper(),
    fr_gripper_X_fr_world(),
    fr_world_X_fr_link_1(),
    fr_link_1_X_fr_world(),
    fr_world_X_fr_link_2(),
    fr_link_2_X_fr_world(),
    fr_world_X_fr_base_link(),
    fr_base_link_X_fr_world(),
    fr_world_X_fr_end(),
    fr_end_X_fr_world(),
    fr_world_X_fr_gripper_COM(),
    fr_gripper_COM_X_fr_world(),
    fr_world_X_fr_link_1_COM(),
    fr_link_1_COM_X_fr_world(),
    fr_world_X_fr_link_2_COM(),
    fr_link_2_COM_X_fr_world(),
    fr_world_X_fr_world_COM(),
    fr_world_COM_X_fr_world(),
    fr_world_X_fr_joint_1(),
    fr_world_X_fr_joint_2(),
    fr_world_X_fr_joint_3(),
    fr_link_2_X_fr_link_1(),
    fr_link_1_X_fr_link_2(),
    fr_gripper_X_fr_link_2(),
    fr_link_2_X_fr_gripper()
{
    updateParameters();
}
template <typename TRAIT>
void iit::Ballbot::tpl::MotionTransforms<TRAIT>::updateParameters() {
}

template <typename TRAIT>
iit::Ballbot::tpl::ForceTransforms<TRAIT>::ForceTransforms
    ()
     :
    fr_world_X_fr_gripper(),
    fr_gripper_X_fr_world(),
    fr_world_X_fr_link_1(),
    fr_link_1_X_fr_world(),
    fr_world_X_fr_link_2(),
    fr_link_2_X_fr_world(),
    fr_world_X_fr_base_link(),
    fr_base_link_X_fr_world(),
    fr_world_X_fr_end(),
    fr_end_X_fr_world(),
    fr_world_X_fr_gripper_COM(),
    fr_gripper_COM_X_fr_world(),
    fr_world_X_fr_link_1_COM(),
    fr_link_1_COM_X_fr_world(),
    fr_world_X_fr_link_2_COM(),
    fr_link_2_COM_X_fr_world(),
    fr_world_X_fr_world_COM(),
    fr_world_COM_X_fr_world(),
    fr_world_X_fr_joint_1(),
    fr_world_X_fr_joint_2(),
    fr_world_X_fr_joint_3(),
    fr_link_2_X_fr_link_1(),
    fr_link_1_X_fr_link_2(),
    fr_gripper_X_fr_link_2(),
    fr_link_2_X_fr_gripper()
{
    updateParameters();
}
template <typename TRAIT>
void iit::Ballbot::tpl::ForceTransforms<TRAIT>::updateParameters() {
}

template <typename TRAIT>
iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::HomogeneousTransforms
    ()
     :
    fr_world_X_fr_gripper(),
    fr_gripper_X_fr_world(),
    fr_world_X_fr_link_1(),
    fr_link_1_X_fr_world(),
    fr_world_X_fr_link_2(),
    fr_link_2_X_fr_world(),
    fr_world_X_fr_base_link(),
    fr_base_link_X_fr_world(),
    fr_world_X_fr_end(),
    fr_end_X_fr_world(),
    fr_world_X_fr_gripper_COM(),
    fr_gripper_COM_X_fr_world(),
    fr_world_X_fr_link_1_COM(),
    fr_link_1_COM_X_fr_world(),
    fr_world_X_fr_link_2_COM(),
    fr_link_2_COM_X_fr_world(),
    fr_world_X_fr_world_COM(),
    fr_world_COM_X_fr_world(),
    fr_world_X_fr_joint_1(),
    fr_world_X_fr_joint_2(),
    fr_world_X_fr_joint_3(),
    fr_link_2_X_fr_link_1(),
    fr_link_1_X_fr_link_2(),
    fr_gripper_X_fr_link_2(),
    fr_link_2_X_fr_gripper()
{
    updateParameters();
}
template <typename TRAIT>
void iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::updateParameters() {
}

template <typename TRAIT>

iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_world_X_fr_gripper::Type_fr_world_X_fr_gripper()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_world_X_fr_gripper& iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_world_X_fr_gripper::update(const JState& q)
{
    Scalar sin_q_joint_1  = TRAIT::sin( q(JOINT_1) );
    Scalar cos_q_joint_1  = TRAIT::cos( q(JOINT_1) );
    Scalar sin_q_joint_2  = TRAIT::sin( q(JOINT_2) );
    Scalar cos_q_joint_2  = TRAIT::cos( q(JOINT_2) );
    Scalar sin_q_joint_3  = TRAIT::sin( q(JOINT_3) );
    Scalar cos_q_joint_3  = TRAIT::cos( q(JOINT_3) );
    (*this)(0,0) = (((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * cos_q_joint_3);
    (*this)(0,1) = (((sin_q_joint_1 * sin_q_joint_2)-(cos_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(1,0) = (((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * sin_q_joint_2)+(sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(1,1) = (((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * cos_q_joint_3);
    (*this)(3,0) = ((( tz_joint_1 * sin_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * cos_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((- tz_joint_1 * cos_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(3,1) = ((( tz_joint_1 * cos_q_joint_1 * sin_q_joint_2)+( tz_joint_1 * sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+((( tz_joint_1 * sin_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * cos_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(3,2) = ( tx_joint_3 * cos_q_joint_1 * sin_q_joint_2)+( tx_joint_3 * sin_q_joint_1 * cos_q_joint_2)+( tx_joint_2 * sin_q_joint_1);
    (*this)(3,3) = (((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * cos_q_joint_3);
    (*this)(3,4) = (((sin_q_joint_1 * sin_q_joint_2)-(cos_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(4,0) = (((- tz_joint_1 * cos_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+((( tz_joint_1 * cos_q_joint_1 * cos_q_joint_2)-( tz_joint_1 * sin_q_joint_1 * sin_q_joint_2)) * cos_q_joint_3);
    (*this)(4,1) = ((( tz_joint_1 * sin_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * cos_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((- tz_joint_1 * cos_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(4,2) = ( tx_joint_3 * sin_q_joint_1 * sin_q_joint_2)-( tx_joint_3 * cos_q_joint_1 * cos_q_joint_2)-( tx_joint_2 * cos_q_joint_1);
    (*this)(4,3) = (((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * sin_q_joint_2)+(sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(4,4) = (((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * cos_q_joint_3);
    (*this)(5,0) = ((( tx_joint_2 * cos_q_joint_2)+ tx_joint_3) * sin_q_joint_3)+( tx_joint_2 * sin_q_joint_2 * cos_q_joint_3);
    (*this)(5,1) = ((( tx_joint_2 * cos_q_joint_2)+ tx_joint_3) * cos_q_joint_3)-( tx_joint_2 * sin_q_joint_2 * sin_q_joint_3);
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_gripper_X_fr_world::Type_fr_gripper_X_fr_world()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_gripper_X_fr_world& iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_gripper_X_fr_world::update(const JState& q)
{
    Scalar sin_q_joint_3  = TRAIT::sin( q(JOINT_3) );
    Scalar cos_q_joint_3  = TRAIT::cos( q(JOINT_3) );
    Scalar sin_q_joint_2  = TRAIT::sin( q(JOINT_2) );
    Scalar cos_q_joint_2  = TRAIT::cos( q(JOINT_2) );
    Scalar sin_q_joint_1  = TRAIT::sin( q(JOINT_1) );
    Scalar cos_q_joint_1  = TRAIT::cos( q(JOINT_1) );
    (*this)(0,0) = (((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * cos_q_joint_3);
    (*this)(0,1) = (((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * sin_q_joint_2)+(sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(1,0) = (((sin_q_joint_1 * sin_q_joint_2)-(cos_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(1,1) = (((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * cos_q_joint_3);
    (*this)(3,0) = ((( tz_joint_1 * sin_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * cos_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((- tz_joint_1 * cos_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(3,1) = (((- tz_joint_1 * cos_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+((( tz_joint_1 * cos_q_joint_1 * cos_q_joint_2)-( tz_joint_1 * sin_q_joint_1 * sin_q_joint_2)) * cos_q_joint_3);
    (*this)(3,2) = ((( tx_joint_2 * cos_q_joint_2)+ tx_joint_3) * sin_q_joint_3)+( tx_joint_2 * sin_q_joint_2 * cos_q_joint_3);
    (*this)(3,3) = (((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * cos_q_joint_3);
    (*this)(3,4) = (((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * sin_q_joint_2)+(sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(4,0) = ((( tz_joint_1 * cos_q_joint_1 * sin_q_joint_2)+( tz_joint_1 * sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+((( tz_joint_1 * sin_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * cos_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(4,1) = ((( tz_joint_1 * sin_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * cos_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((- tz_joint_1 * cos_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(4,2) = ((( tx_joint_2 * cos_q_joint_2)+ tx_joint_3) * cos_q_joint_3)-( tx_joint_2 * sin_q_joint_2 * sin_q_joint_3);
    (*this)(4,3) = (((sin_q_joint_1 * sin_q_joint_2)-(cos_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(4,4) = (((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * cos_q_joint_3);
    (*this)(5,0) = ( tx_joint_3 * cos_q_joint_1 * sin_q_joint_2)+( tx_joint_3 * sin_q_joint_1 * cos_q_joint_2)+( tx_joint_2 * sin_q_joint_1);
    (*this)(5,1) = ( tx_joint_3 * sin_q_joint_1 * sin_q_joint_2)-( tx_joint_3 * cos_q_joint_1 * cos_q_joint_2)-( tx_joint_2 * cos_q_joint_1);
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_world_X_fr_link_1::Type_fr_world_X_fr_link_1()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_world_X_fr_link_1& iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_world_X_fr_link_1::update(const JState& q)
{
    Scalar sin_q_joint_1  = TRAIT::sin( q(JOINT_1) );
    Scalar cos_q_joint_1  = TRAIT::cos( q(JOINT_1) );
    (*this)(0,0) = cos_q_joint_1;
    (*this)(0,1) = -sin_q_joint_1;
    (*this)(1,0) = sin_q_joint_1;
    (*this)(1,1) = cos_q_joint_1;
    (*this)(3,0) = - tz_joint_1 * sin_q_joint_1;
    (*this)(3,1) = - tz_joint_1 * cos_q_joint_1;
    (*this)(3,3) = cos_q_joint_1;
    (*this)(3,4) = -sin_q_joint_1;
    (*this)(4,0) =  tz_joint_1 * cos_q_joint_1;
    (*this)(4,1) = - tz_joint_1 * sin_q_joint_1;
    (*this)(4,3) = sin_q_joint_1;
    (*this)(4,4) = cos_q_joint_1;
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_link_1_X_fr_world::Type_fr_link_1_X_fr_world()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_link_1_X_fr_world& iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_link_1_X_fr_world::update(const JState& q)
{
    Scalar sin_q_joint_1  = TRAIT::sin( q(JOINT_1) );
    Scalar cos_q_joint_1  = TRAIT::cos( q(JOINT_1) );
    (*this)(0,0) = cos_q_joint_1;
    (*this)(0,1) = sin_q_joint_1;
    (*this)(1,0) = -sin_q_joint_1;
    (*this)(1,1) = cos_q_joint_1;
    (*this)(3,0) = - tz_joint_1 * sin_q_joint_1;
    (*this)(3,1) =  tz_joint_1 * cos_q_joint_1;
    (*this)(3,3) = cos_q_joint_1;
    (*this)(3,4) = sin_q_joint_1;
    (*this)(4,0) = - tz_joint_1 * cos_q_joint_1;
    (*this)(4,1) = - tz_joint_1 * sin_q_joint_1;
    (*this)(4,3) = -sin_q_joint_1;
    (*this)(4,4) = cos_q_joint_1;
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_world_X_fr_link_2::Type_fr_world_X_fr_link_2()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_world_X_fr_link_2& iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_world_X_fr_link_2::update(const JState& q)
{
    Scalar sin_q_joint_1  = TRAIT::sin( q(JOINT_1) );
    Scalar cos_q_joint_1  = TRAIT::cos( q(JOINT_1) );
    Scalar sin_q_joint_2  = TRAIT::sin( q(JOINT_2) );
    Scalar cos_q_joint_2  = TRAIT::cos( q(JOINT_2) );
    (*this)(0,0) = (cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2);
    (*this)(0,1) = (-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2);
    (*this)(1,0) = (cos_q_joint_1 * sin_q_joint_2)+(sin_q_joint_1 * cos_q_joint_2);
    (*this)(1,1) = (cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2);
    (*this)(3,0) = (- tz_joint_1 * cos_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * sin_q_joint_1 * cos_q_joint_2);
    (*this)(3,1) = ( tz_joint_1 * sin_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * cos_q_joint_1 * cos_q_joint_2);
    (*this)(3,2) =  tx_joint_2 * sin_q_joint_1;
    (*this)(3,3) = (cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2);
    (*this)(3,4) = (-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2);
    (*this)(4,0) = ( tz_joint_1 * cos_q_joint_1 * cos_q_joint_2)-( tz_joint_1 * sin_q_joint_1 * sin_q_joint_2);
    (*this)(4,1) = (- tz_joint_1 * cos_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * sin_q_joint_1 * cos_q_joint_2);
    (*this)(4,2) = - tx_joint_2 * cos_q_joint_1;
    (*this)(4,3) = (cos_q_joint_1 * sin_q_joint_2)+(sin_q_joint_1 * cos_q_joint_2);
    (*this)(4,4) = (cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2);
    (*this)(5,0) =  tx_joint_2 * sin_q_joint_2;
    (*this)(5,1) =  tx_joint_2 * cos_q_joint_2;
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_link_2_X_fr_world::Type_fr_link_2_X_fr_world()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_link_2_X_fr_world& iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_link_2_X_fr_world::update(const JState& q)
{
    Scalar sin_q_joint_2  = TRAIT::sin( q(JOINT_2) );
    Scalar cos_q_joint_2  = TRAIT::cos( q(JOINT_2) );
    Scalar sin_q_joint_1  = TRAIT::sin( q(JOINT_1) );
    Scalar cos_q_joint_1  = TRAIT::cos( q(JOINT_1) );
    (*this)(0,0) = (cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2);
    (*this)(0,1) = (cos_q_joint_1 * sin_q_joint_2)+(sin_q_joint_1 * cos_q_joint_2);
    (*this)(1,0) = (-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2);
    (*this)(1,1) = (cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2);
    (*this)(3,0) = (- tz_joint_1 * cos_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * sin_q_joint_1 * cos_q_joint_2);
    (*this)(3,1) = ( tz_joint_1 * cos_q_joint_1 * cos_q_joint_2)-( tz_joint_1 * sin_q_joint_1 * sin_q_joint_2);
    (*this)(3,2) =  tx_joint_2 * sin_q_joint_2;
    (*this)(3,3) = (cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2);
    (*this)(3,4) = (cos_q_joint_1 * sin_q_joint_2)+(sin_q_joint_1 * cos_q_joint_2);
    (*this)(4,0) = ( tz_joint_1 * sin_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * cos_q_joint_1 * cos_q_joint_2);
    (*this)(4,1) = (- tz_joint_1 * cos_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * sin_q_joint_1 * cos_q_joint_2);
    (*this)(4,2) =  tx_joint_2 * cos_q_joint_2;
    (*this)(4,3) = (-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2);
    (*this)(4,4) = (cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2);
    (*this)(5,0) =  tx_joint_2 * sin_q_joint_1;
    (*this)(5,1) = - tx_joint_2 * cos_q_joint_1;
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_world_X_fr_base_link::Type_fr_world_X_fr_base_link()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_world_X_fr_base_link& iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_world_X_fr_base_link::update(const JState& q)
{
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_base_link_X_fr_world::Type_fr_base_link_X_fr_world()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_base_link_X_fr_world& iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_base_link_X_fr_world::update(const JState& q)
{
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_world_X_fr_end::Type_fr_world_X_fr_end()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_world_X_fr_end& iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_world_X_fr_end::update(const JState& q)
{
    Scalar sin_q_joint_1  = TRAIT::sin( q(JOINT_1) );
    Scalar cos_q_joint_1  = TRAIT::cos( q(JOINT_1) );
    Scalar sin_q_joint_2  = TRAIT::sin( q(JOINT_2) );
    Scalar cos_q_joint_2  = TRAIT::cos( q(JOINT_2) );
    Scalar sin_q_joint_3  = TRAIT::sin( q(JOINT_3) );
    Scalar cos_q_joint_3  = TRAIT::cos( q(JOINT_3) );
    (*this)(0,0) = (((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * cos_q_joint_3);
    (*this)(0,1) = (((sin_q_joint_1 * sin_q_joint_2)-(cos_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(1,0) = (((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * sin_q_joint_2)+(sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(1,1) = (((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * cos_q_joint_3);
    (*this)(3,0) = ((( tz_joint_1 * sin_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * cos_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((- tz_joint_1 * cos_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(3,1) = ((( tz_joint_1 * cos_q_joint_1 * sin_q_joint_2)+( tz_joint_1 * sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+((( tz_joint_1 * sin_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * cos_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(3,2) = ((( tx_fr_end * cos_q_joint_1 * cos_q_joint_2)-( tx_fr_end * sin_q_joint_1 * sin_q_joint_2)) * sin_q_joint_3)+((( tx_fr_end * cos_q_joint_1 * sin_q_joint_2)+( tx_fr_end * sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3)+( tx_joint_3 * cos_q_joint_1 * sin_q_joint_2)+( tx_joint_3 * sin_q_joint_1 * cos_q_joint_2)+( tx_joint_2 * sin_q_joint_1);
    (*this)(3,3) = (((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * cos_q_joint_3);
    (*this)(3,4) = (((sin_q_joint_1 * sin_q_joint_2)-(cos_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(4,0) = (((- tz_joint_1 * cos_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+((( tz_joint_1 * cos_q_joint_1 * cos_q_joint_2)-( tz_joint_1 * sin_q_joint_1 * sin_q_joint_2)) * cos_q_joint_3);
    (*this)(4,1) = ((( tz_joint_1 * sin_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * cos_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((- tz_joint_1 * cos_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(4,2) = ((( tx_fr_end * cos_q_joint_1 * sin_q_joint_2)+( tx_fr_end * sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+((( tx_fr_end * sin_q_joint_1 * sin_q_joint_2)-( tx_fr_end * cos_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3)+( tx_joint_3 * sin_q_joint_1 * sin_q_joint_2)-( tx_joint_3 * cos_q_joint_1 * cos_q_joint_2)-( tx_joint_2 * cos_q_joint_1);
    (*this)(4,3) = (((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * sin_q_joint_2)+(sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(4,4) = (((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * cos_q_joint_3);
    (*this)(5,0) = ((( tx_joint_2 * cos_q_joint_2)+ tx_joint_3) * sin_q_joint_3)+( tx_joint_2 * sin_q_joint_2 * cos_q_joint_3);
    (*this)(5,1) = (- tx_joint_2 * sin_q_joint_2 * sin_q_joint_3)+((( tx_joint_2 * cos_q_joint_2)+ tx_joint_3) * cos_q_joint_3)+ tx_fr_end;
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_end_X_fr_world::Type_fr_end_X_fr_world()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_end_X_fr_world& iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_end_X_fr_world::update(const JState& q)
{
    Scalar sin_q_joint_3  = TRAIT::sin( q(JOINT_3) );
    Scalar cos_q_joint_3  = TRAIT::cos( q(JOINT_3) );
    Scalar sin_q_joint_2  = TRAIT::sin( q(JOINT_2) );
    Scalar cos_q_joint_2  = TRAIT::cos( q(JOINT_2) );
    Scalar sin_q_joint_1  = TRAIT::sin( q(JOINT_1) );
    Scalar cos_q_joint_1  = TRAIT::cos( q(JOINT_1) );
    (*this)(0,0) = (((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * cos_q_joint_3);
    (*this)(0,1) = (((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * sin_q_joint_2)+(sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(1,0) = (((sin_q_joint_1 * sin_q_joint_2)-(cos_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(1,1) = (((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * cos_q_joint_3);
    (*this)(3,0) = ((( tz_joint_1 * sin_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * cos_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((- tz_joint_1 * cos_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(3,1) = (((- tz_joint_1 * cos_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+((( tz_joint_1 * cos_q_joint_1 * cos_q_joint_2)-( tz_joint_1 * sin_q_joint_1 * sin_q_joint_2)) * cos_q_joint_3);
    (*this)(3,2) = ((( tx_joint_2 * cos_q_joint_2)+ tx_joint_3) * sin_q_joint_3)+( tx_joint_2 * sin_q_joint_2 * cos_q_joint_3);
    (*this)(3,3) = (((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * cos_q_joint_3);
    (*this)(3,4) = (((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * sin_q_joint_2)+(sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(4,0) = ((( tz_joint_1 * cos_q_joint_1 * sin_q_joint_2)+( tz_joint_1 * sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+((( tz_joint_1 * sin_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * cos_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(4,1) = ((( tz_joint_1 * sin_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * cos_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((- tz_joint_1 * cos_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(4,2) = (- tx_joint_2 * sin_q_joint_2 * sin_q_joint_3)+((( tx_joint_2 * cos_q_joint_2)+ tx_joint_3) * cos_q_joint_3)+ tx_fr_end;
    (*this)(4,3) = (((sin_q_joint_1 * sin_q_joint_2)-(cos_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(4,4) = (((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * cos_q_joint_3);
    (*this)(5,0) = ((( tx_fr_end * cos_q_joint_1 * cos_q_joint_2)-( tx_fr_end * sin_q_joint_1 * sin_q_joint_2)) * sin_q_joint_3)+((( tx_fr_end * cos_q_joint_1 * sin_q_joint_2)+( tx_fr_end * sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3)+( tx_joint_3 * cos_q_joint_1 * sin_q_joint_2)+( tx_joint_3 * sin_q_joint_1 * cos_q_joint_2)+( tx_joint_2 * sin_q_joint_1);
    (*this)(5,1) = ((( tx_fr_end * cos_q_joint_1 * sin_q_joint_2)+( tx_fr_end * sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+((( tx_fr_end * sin_q_joint_1 * sin_q_joint_2)-( tx_fr_end * cos_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3)+( tx_joint_3 * sin_q_joint_1 * sin_q_joint_2)-( tx_joint_3 * cos_q_joint_1 * cos_q_joint_2)-( tx_joint_2 * cos_q_joint_1);
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_world_X_fr_gripper_COM::Type_fr_world_X_fr_gripper_COM()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_world_X_fr_gripper_COM& iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_world_X_fr_gripper_COM::update(const JState& q)
{
    Scalar sin_q_joint_1  = TRAIT::sin( q(JOINT_1) );
    Scalar cos_q_joint_1  = TRAIT::cos( q(JOINT_1) );
    Scalar sin_q_joint_2  = TRAIT::sin( q(JOINT_2) );
    Scalar cos_q_joint_2  = TRAIT::cos( q(JOINT_2) );
    Scalar sin_q_joint_3  = TRAIT::sin( q(JOINT_3) );
    Scalar cos_q_joint_3  = TRAIT::cos( q(JOINT_3) );
    (*this)(0,0) = (((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * cos_q_joint_3);
    (*this)(0,1) = (((sin_q_joint_1 * sin_q_joint_2)-(cos_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(1,0) = (((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * sin_q_joint_2)+(sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(1,1) = (((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * cos_q_joint_3);
    (*this)(3,0) = ((( tz_joint_1 * sin_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * cos_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((- tz_joint_1 * cos_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(3,1) = ((( tz_joint_1 * cos_q_joint_1 * sin_q_joint_2)+( tz_joint_1 * sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+((( tz_joint_1 * sin_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * cos_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(3,2) = ((( tx_fr_gripper_COM * cos_q_joint_1 * cos_q_joint_2)-( tx_fr_gripper_COM * sin_q_joint_1 * sin_q_joint_2)) * sin_q_joint_3)+((( tx_fr_gripper_COM * cos_q_joint_1 * sin_q_joint_2)+( tx_fr_gripper_COM * sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3)+( tx_joint_3 * cos_q_joint_1 * sin_q_joint_2)+( tx_joint_3 * sin_q_joint_1 * cos_q_joint_2)+( tx_joint_2 * sin_q_joint_1);
    (*this)(3,3) = (((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * cos_q_joint_3);
    (*this)(3,4) = (((sin_q_joint_1 * sin_q_joint_2)-(cos_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(4,0) = (((- tz_joint_1 * cos_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+((( tz_joint_1 * cos_q_joint_1 * cos_q_joint_2)-( tz_joint_1 * sin_q_joint_1 * sin_q_joint_2)) * cos_q_joint_3);
    (*this)(4,1) = ((( tz_joint_1 * sin_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * cos_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((- tz_joint_1 * cos_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(4,2) = ((( tx_fr_gripper_COM * cos_q_joint_1 * sin_q_joint_2)+( tx_fr_gripper_COM * sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+((( tx_fr_gripper_COM * sin_q_joint_1 * sin_q_joint_2)-( tx_fr_gripper_COM * cos_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3)+( tx_joint_3 * sin_q_joint_1 * sin_q_joint_2)-( tx_joint_3 * cos_q_joint_1 * cos_q_joint_2)-( tx_joint_2 * cos_q_joint_1);
    (*this)(4,3) = (((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * sin_q_joint_2)+(sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(4,4) = (((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * cos_q_joint_3);
    (*this)(5,0) = ((( tx_joint_2 * cos_q_joint_2)+ tx_joint_3) * sin_q_joint_3)+( tx_joint_2 * sin_q_joint_2 * cos_q_joint_3);
    (*this)(5,1) = (- tx_joint_2 * sin_q_joint_2 * sin_q_joint_3)+((( tx_joint_2 * cos_q_joint_2)+ tx_joint_3) * cos_q_joint_3)+ tx_fr_gripper_COM;
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_gripper_COM_X_fr_world::Type_fr_gripper_COM_X_fr_world()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_gripper_COM_X_fr_world& iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_gripper_COM_X_fr_world::update(const JState& q)
{
    Scalar sin_q_joint_3  = TRAIT::sin( q(JOINT_3) );
    Scalar cos_q_joint_3  = TRAIT::cos( q(JOINT_3) );
    Scalar sin_q_joint_2  = TRAIT::sin( q(JOINT_2) );
    Scalar cos_q_joint_2  = TRAIT::cos( q(JOINT_2) );
    Scalar sin_q_joint_1  = TRAIT::sin( q(JOINT_1) );
    Scalar cos_q_joint_1  = TRAIT::cos( q(JOINT_1) );
    (*this)(0,0) = (((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * cos_q_joint_3);
    (*this)(0,1) = (((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * sin_q_joint_2)+(sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(1,0) = (((sin_q_joint_1 * sin_q_joint_2)-(cos_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(1,1) = (((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * cos_q_joint_3);
    (*this)(3,0) = ((( tz_joint_1 * sin_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * cos_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((- tz_joint_1 * cos_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(3,1) = (((- tz_joint_1 * cos_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+((( tz_joint_1 * cos_q_joint_1 * cos_q_joint_2)-( tz_joint_1 * sin_q_joint_1 * sin_q_joint_2)) * cos_q_joint_3);
    (*this)(3,2) = ((( tx_joint_2 * cos_q_joint_2)+ tx_joint_3) * sin_q_joint_3)+( tx_joint_2 * sin_q_joint_2 * cos_q_joint_3);
    (*this)(3,3) = (((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * cos_q_joint_3);
    (*this)(3,4) = (((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * sin_q_joint_2)+(sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(4,0) = ((( tz_joint_1 * cos_q_joint_1 * sin_q_joint_2)+( tz_joint_1 * sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+((( tz_joint_1 * sin_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * cos_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(4,1) = ((( tz_joint_1 * sin_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * cos_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((- tz_joint_1 * cos_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(4,2) = (- tx_joint_2 * sin_q_joint_2 * sin_q_joint_3)+((( tx_joint_2 * cos_q_joint_2)+ tx_joint_3) * cos_q_joint_3)+ tx_fr_gripper_COM;
    (*this)(4,3) = (((sin_q_joint_1 * sin_q_joint_2)-(cos_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(4,4) = (((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * cos_q_joint_3);
    (*this)(5,0) = ((( tx_fr_gripper_COM * cos_q_joint_1 * cos_q_joint_2)-( tx_fr_gripper_COM * sin_q_joint_1 * sin_q_joint_2)) * sin_q_joint_3)+((( tx_fr_gripper_COM * cos_q_joint_1 * sin_q_joint_2)+( tx_fr_gripper_COM * sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3)+( tx_joint_3 * cos_q_joint_1 * sin_q_joint_2)+( tx_joint_3 * sin_q_joint_1 * cos_q_joint_2)+( tx_joint_2 * sin_q_joint_1);
    (*this)(5,1) = ((( tx_fr_gripper_COM * cos_q_joint_1 * sin_q_joint_2)+( tx_fr_gripper_COM * sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+((( tx_fr_gripper_COM * sin_q_joint_1 * sin_q_joint_2)-( tx_fr_gripper_COM * cos_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3)+( tx_joint_3 * sin_q_joint_1 * sin_q_joint_2)-( tx_joint_3 * cos_q_joint_1 * cos_q_joint_2)-( tx_joint_2 * cos_q_joint_1);
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_world_X_fr_link_1_COM::Type_fr_world_X_fr_link_1_COM()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) =  tx_fr_link_1_COM;    // Maxima DSL: _k__tx_fr_link_1_COM
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_world_X_fr_link_1_COM& iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_world_X_fr_link_1_COM::update(const JState& q)
{
    Scalar sin_q_joint_1  = TRAIT::sin( q(JOINT_1) );
    Scalar cos_q_joint_1  = TRAIT::cos( q(JOINT_1) );
    (*this)(0,0) = cos_q_joint_1;
    (*this)(0,1) = -sin_q_joint_1;
    (*this)(1,0) = sin_q_joint_1;
    (*this)(1,1) = cos_q_joint_1;
    (*this)(3,0) = - tz_joint_1 * sin_q_joint_1;
    (*this)(3,1) = - tz_joint_1 * cos_q_joint_1;
    (*this)(3,2) =  tx_fr_link_1_COM * sin_q_joint_1;
    (*this)(3,3) = cos_q_joint_1;
    (*this)(3,4) = -sin_q_joint_1;
    (*this)(4,0) =  tz_joint_1 * cos_q_joint_1;
    (*this)(4,1) = - tz_joint_1 * sin_q_joint_1;
    (*this)(4,2) = - tx_fr_link_1_COM * cos_q_joint_1;
    (*this)(4,3) = sin_q_joint_1;
    (*this)(4,4) = cos_q_joint_1;
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_link_1_COM_X_fr_world::Type_fr_link_1_COM_X_fr_world()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,2) =  tx_fr_link_1_COM;    // Maxima DSL: _k__tx_fr_link_1_COM
    (*this)(4,5) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_link_1_COM_X_fr_world& iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_link_1_COM_X_fr_world::update(const JState& q)
{
    Scalar sin_q_joint_1  = TRAIT::sin( q(JOINT_1) );
    Scalar cos_q_joint_1  = TRAIT::cos( q(JOINT_1) );
    (*this)(0,0) = cos_q_joint_1;
    (*this)(0,1) = sin_q_joint_1;
    (*this)(1,0) = -sin_q_joint_1;
    (*this)(1,1) = cos_q_joint_1;
    (*this)(3,0) = - tz_joint_1 * sin_q_joint_1;
    (*this)(3,1) =  tz_joint_1 * cos_q_joint_1;
    (*this)(3,3) = cos_q_joint_1;
    (*this)(3,4) = sin_q_joint_1;
    (*this)(4,0) = - tz_joint_1 * cos_q_joint_1;
    (*this)(4,1) = - tz_joint_1 * sin_q_joint_1;
    (*this)(4,3) = -sin_q_joint_1;
    (*this)(4,4) = cos_q_joint_1;
    (*this)(5,0) =  tx_fr_link_1_COM * sin_q_joint_1;
    (*this)(5,1) = - tx_fr_link_1_COM * cos_q_joint_1;
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_world_X_fr_link_2_COM::Type_fr_world_X_fr_link_2_COM()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_world_X_fr_link_2_COM& iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_world_X_fr_link_2_COM::update(const JState& q)
{
    Scalar sin_q_joint_1  = TRAIT::sin( q(JOINT_1) );
    Scalar cos_q_joint_1  = TRAIT::cos( q(JOINT_1) );
    Scalar sin_q_joint_2  = TRAIT::sin( q(JOINT_2) );
    Scalar cos_q_joint_2  = TRAIT::cos( q(JOINT_2) );
    (*this)(0,0) = (cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2);
    (*this)(0,1) = (-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2);
    (*this)(1,0) = (cos_q_joint_1 * sin_q_joint_2)+(sin_q_joint_1 * cos_q_joint_2);
    (*this)(1,1) = (cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2);
    (*this)(3,0) = (- tz_joint_1 * cos_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * sin_q_joint_1 * cos_q_joint_2);
    (*this)(3,1) = ( tz_joint_1 * sin_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * cos_q_joint_1 * cos_q_joint_2);
    (*this)(3,2) = ( tx_fr_link_2_COM * cos_q_joint_1 * sin_q_joint_2)+( tx_fr_link_2_COM * sin_q_joint_1 * cos_q_joint_2)+( tx_joint_2 * sin_q_joint_1);
    (*this)(3,3) = (cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2);
    (*this)(3,4) = (-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2);
    (*this)(4,0) = ( tz_joint_1 * cos_q_joint_1 * cos_q_joint_2)-( tz_joint_1 * sin_q_joint_1 * sin_q_joint_2);
    (*this)(4,1) = (- tz_joint_1 * cos_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * sin_q_joint_1 * cos_q_joint_2);
    (*this)(4,2) = ( tx_fr_link_2_COM * sin_q_joint_1 * sin_q_joint_2)-( tx_fr_link_2_COM * cos_q_joint_1 * cos_q_joint_2)-( tx_joint_2 * cos_q_joint_1);
    (*this)(4,3) = (cos_q_joint_1 * sin_q_joint_2)+(sin_q_joint_1 * cos_q_joint_2);
    (*this)(4,4) = (cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2);
    (*this)(5,0) =  tx_joint_2 * sin_q_joint_2;
    (*this)(5,1) = ( tx_joint_2 * cos_q_joint_2)+ tx_fr_link_2_COM;
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_link_2_COM_X_fr_world::Type_fr_link_2_COM_X_fr_world()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_link_2_COM_X_fr_world& iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_link_2_COM_X_fr_world::update(const JState& q)
{
    Scalar sin_q_joint_2  = TRAIT::sin( q(JOINT_2) );
    Scalar cos_q_joint_2  = TRAIT::cos( q(JOINT_2) );
    Scalar sin_q_joint_1  = TRAIT::sin( q(JOINT_1) );
    Scalar cos_q_joint_1  = TRAIT::cos( q(JOINT_1) );
    (*this)(0,0) = (cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2);
    (*this)(0,1) = (cos_q_joint_1 * sin_q_joint_2)+(sin_q_joint_1 * cos_q_joint_2);
    (*this)(1,0) = (-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2);
    (*this)(1,1) = (cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2);
    (*this)(3,0) = (- tz_joint_1 * cos_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * sin_q_joint_1 * cos_q_joint_2);
    (*this)(3,1) = ( tz_joint_1 * cos_q_joint_1 * cos_q_joint_2)-( tz_joint_1 * sin_q_joint_1 * sin_q_joint_2);
    (*this)(3,2) =  tx_joint_2 * sin_q_joint_2;
    (*this)(3,3) = (cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2);
    (*this)(3,4) = (cos_q_joint_1 * sin_q_joint_2)+(sin_q_joint_1 * cos_q_joint_2);
    (*this)(4,0) = ( tz_joint_1 * sin_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * cos_q_joint_1 * cos_q_joint_2);
    (*this)(4,1) = (- tz_joint_1 * cos_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * sin_q_joint_1 * cos_q_joint_2);
    (*this)(4,2) = ( tx_joint_2 * cos_q_joint_2)+ tx_fr_link_2_COM;
    (*this)(4,3) = (-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2);
    (*this)(4,4) = (cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2);
    (*this)(5,0) = ( tx_fr_link_2_COM * cos_q_joint_1 * sin_q_joint_2)+( tx_fr_link_2_COM * sin_q_joint_1 * cos_q_joint_2)+( tx_joint_2 * sin_q_joint_1);
    (*this)(5,1) = ( tx_fr_link_2_COM * sin_q_joint_1 * sin_q_joint_2)-( tx_fr_link_2_COM * cos_q_joint_1 * cos_q_joint_2)-( tx_joint_2 * cos_q_joint_1);
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_world_X_fr_world_COM::Type_fr_world_X_fr_world_COM()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_world_X_fr_world_COM& iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_world_X_fr_world_COM::update(const JState& q)
{
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_world_COM_X_fr_world::Type_fr_world_COM_X_fr_world()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_world_COM_X_fr_world& iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_world_COM_X_fr_world::update(const JState& q)
{
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_world_X_fr_joint_1::Type_fr_world_X_fr_joint_1()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = - tz_joint_1;    // Maxima DSL: -_k__tz_joint_1
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) =  tz_joint_1;    // Maxima DSL: _k__tz_joint_1
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_world_X_fr_joint_1& iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_world_X_fr_joint_1::update(const JState& q)
{
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_world_X_fr_joint_2::Type_fr_world_X_fr_joint_2()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) =  tx_joint_2;    // Maxima DSL: _k__tx_joint_2
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_world_X_fr_joint_2& iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_world_X_fr_joint_2::update(const JState& q)
{
    Scalar sin_q_joint_1  = TRAIT::sin( q(JOINT_1) );
    Scalar cos_q_joint_1  = TRAIT::cos( q(JOINT_1) );
    (*this)(0,0) = cos_q_joint_1;
    (*this)(0,1) = -sin_q_joint_1;
    (*this)(1,0) = sin_q_joint_1;
    (*this)(1,1) = cos_q_joint_1;
    (*this)(3,0) = - tz_joint_1 * sin_q_joint_1;
    (*this)(3,1) = - tz_joint_1 * cos_q_joint_1;
    (*this)(3,2) =  tx_joint_2 * sin_q_joint_1;
    (*this)(3,3) = cos_q_joint_1;
    (*this)(3,4) = -sin_q_joint_1;
    (*this)(4,0) =  tz_joint_1 * cos_q_joint_1;
    (*this)(4,1) = - tz_joint_1 * sin_q_joint_1;
    (*this)(4,2) = - tx_joint_2 * cos_q_joint_1;
    (*this)(4,3) = sin_q_joint_1;
    (*this)(4,4) = cos_q_joint_1;
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_world_X_fr_joint_3::Type_fr_world_X_fr_joint_3()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_world_X_fr_joint_3& iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_world_X_fr_joint_3::update(const JState& q)
{
    Scalar sin_q_joint_1  = TRAIT::sin( q(JOINT_1) );
    Scalar cos_q_joint_1  = TRAIT::cos( q(JOINT_1) );
    Scalar sin_q_joint_2  = TRAIT::sin( q(JOINT_2) );
    Scalar cos_q_joint_2  = TRAIT::cos( q(JOINT_2) );
    (*this)(0,0) = (cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2);
    (*this)(0,1) = (-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2);
    (*this)(1,0) = (cos_q_joint_1 * sin_q_joint_2)+(sin_q_joint_1 * cos_q_joint_2);
    (*this)(1,1) = (cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2);
    (*this)(3,0) = (- tz_joint_1 * cos_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * sin_q_joint_1 * cos_q_joint_2);
    (*this)(3,1) = ( tz_joint_1 * sin_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * cos_q_joint_1 * cos_q_joint_2);
    (*this)(3,2) = ( tx_joint_3 * cos_q_joint_1 * sin_q_joint_2)+( tx_joint_3 * sin_q_joint_1 * cos_q_joint_2)+( tx_joint_2 * sin_q_joint_1);
    (*this)(3,3) = (cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2);
    (*this)(3,4) = (-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2);
    (*this)(4,0) = ( tz_joint_1 * cos_q_joint_1 * cos_q_joint_2)-( tz_joint_1 * sin_q_joint_1 * sin_q_joint_2);
    (*this)(4,1) = (- tz_joint_1 * cos_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * sin_q_joint_1 * cos_q_joint_2);
    (*this)(4,2) = ( tx_joint_3 * sin_q_joint_1 * sin_q_joint_2)-( tx_joint_3 * cos_q_joint_1 * cos_q_joint_2)-( tx_joint_2 * cos_q_joint_1);
    (*this)(4,3) = (cos_q_joint_1 * sin_q_joint_2)+(sin_q_joint_1 * cos_q_joint_2);
    (*this)(4,4) = (cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2);
    (*this)(5,0) =  tx_joint_2 * sin_q_joint_2;
    (*this)(5,1) = ( tx_joint_2 * cos_q_joint_2)+ tx_joint_3;
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_link_2_X_fr_link_1::Type_fr_link_2_X_fr_link_1()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = - tx_joint_2;    // Maxima DSL: -_k__tx_joint_2
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_link_2_X_fr_link_1& iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_link_2_X_fr_link_1::update(const JState& q)
{
    Scalar sin_q_joint_2  = TRAIT::sin( q(JOINT_2) );
    Scalar cos_q_joint_2  = TRAIT::cos( q(JOINT_2) );
    (*this)(0,0) = cos_q_joint_2;
    (*this)(0,1) = sin_q_joint_2;
    (*this)(1,0) = -sin_q_joint_2;
    (*this)(1,1) = cos_q_joint_2;
    (*this)(3,2) =  tx_joint_2 * sin_q_joint_2;
    (*this)(3,3) = cos_q_joint_2;
    (*this)(3,4) = sin_q_joint_2;
    (*this)(4,2) =  tx_joint_2 * cos_q_joint_2;
    (*this)(4,3) = -sin_q_joint_2;
    (*this)(4,4) = cos_q_joint_2;
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_link_1_X_fr_link_2::Type_fr_link_1_X_fr_link_2()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = - tx_joint_2;    // Maxima DSL: -_k__tx_joint_2
    (*this)(4,5) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_link_1_X_fr_link_2& iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_link_1_X_fr_link_2::update(const JState& q)
{
    Scalar sin_q_joint_2  = TRAIT::sin( q(JOINT_2) );
    Scalar cos_q_joint_2  = TRAIT::cos( q(JOINT_2) );
    (*this)(0,0) = cos_q_joint_2;
    (*this)(0,1) = -sin_q_joint_2;
    (*this)(1,0) = sin_q_joint_2;
    (*this)(1,1) = cos_q_joint_2;
    (*this)(3,3) = cos_q_joint_2;
    (*this)(3,4) = -sin_q_joint_2;
    (*this)(4,3) = sin_q_joint_2;
    (*this)(4,4) = cos_q_joint_2;
    (*this)(5,0) =  tx_joint_2 * sin_q_joint_2;
    (*this)(5,1) =  tx_joint_2 * cos_q_joint_2;
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_gripper_X_fr_link_2::Type_fr_gripper_X_fr_link_2()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = - tx_joint_3;    // Maxima DSL: -_k__tx_joint_3
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_gripper_X_fr_link_2& iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_gripper_X_fr_link_2::update(const JState& q)
{
    Scalar sin_q_joint_3  = TRAIT::sin( q(JOINT_3) );
    Scalar cos_q_joint_3  = TRAIT::cos( q(JOINT_3) );
    (*this)(0,0) = cos_q_joint_3;
    (*this)(0,1) = sin_q_joint_3;
    (*this)(1,0) = -sin_q_joint_3;
    (*this)(1,1) = cos_q_joint_3;
    (*this)(3,2) =  tx_joint_3 * sin_q_joint_3;
    (*this)(3,3) = cos_q_joint_3;
    (*this)(3,4) = sin_q_joint_3;
    (*this)(4,2) =  tx_joint_3 * cos_q_joint_3;
    (*this)(4,3) = -sin_q_joint_3;
    (*this)(4,4) = cos_q_joint_3;
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_link_2_X_fr_gripper::Type_fr_link_2_X_fr_gripper()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = - tx_joint_3;    // Maxima DSL: -_k__tx_joint_3
    (*this)(4,5) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_link_2_X_fr_gripper& iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_link_2_X_fr_gripper::update(const JState& q)
{
    Scalar sin_q_joint_3  = TRAIT::sin( q(JOINT_3) );
    Scalar cos_q_joint_3  = TRAIT::cos( q(JOINT_3) );
    (*this)(0,0) = cos_q_joint_3;
    (*this)(0,1) = -sin_q_joint_3;
    (*this)(1,0) = sin_q_joint_3;
    (*this)(1,1) = cos_q_joint_3;
    (*this)(3,3) = cos_q_joint_3;
    (*this)(3,4) = -sin_q_joint_3;
    (*this)(4,3) = sin_q_joint_3;
    (*this)(4,4) = cos_q_joint_3;
    (*this)(5,0) =  tx_joint_3 * sin_q_joint_3;
    (*this)(5,1) =  tx_joint_3 * cos_q_joint_3;
    return *this;
}

template <typename TRAIT>
iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_world_X_fr_gripper::Type_fr_world_X_fr_gripper()
{
    (*this)(0,2) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_world_X_fr_gripper& iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_world_X_fr_gripper::update(const JState& q)
{
    Scalar sin_q_joint_1  = TRAIT::sin( q(JOINT_1) );
    Scalar cos_q_joint_1  = TRAIT::cos( q(JOINT_1) );
    Scalar sin_q_joint_2  = TRAIT::sin( q(JOINT_2) );
    Scalar cos_q_joint_2  = TRAIT::cos( q(JOINT_2) );
    Scalar sin_q_joint_3  = TRAIT::sin( q(JOINT_3) );
    Scalar cos_q_joint_3  = TRAIT::cos( q(JOINT_3) );
    (*this)(0,0) = (((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * cos_q_joint_3);
    (*this)(0,1) = (((sin_q_joint_1 * sin_q_joint_2)-(cos_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(0,3) = ((( tz_joint_1 * sin_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * cos_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((- tz_joint_1 * cos_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(0,4) = ((( tz_joint_1 * cos_q_joint_1 * sin_q_joint_2)+( tz_joint_1 * sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+((( tz_joint_1 * sin_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * cos_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(0,5) = ( tx_joint_3 * cos_q_joint_1 * sin_q_joint_2)+( tx_joint_3 * sin_q_joint_1 * cos_q_joint_2)+( tx_joint_2 * sin_q_joint_1);
    (*this)(1,0) = (((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * sin_q_joint_2)+(sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(1,1) = (((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * cos_q_joint_3);
    (*this)(1,3) = (((- tz_joint_1 * cos_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+((( tz_joint_1 * cos_q_joint_1 * cos_q_joint_2)-( tz_joint_1 * sin_q_joint_1 * sin_q_joint_2)) * cos_q_joint_3);
    (*this)(1,4) = ((( tz_joint_1 * sin_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * cos_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((- tz_joint_1 * cos_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(1,5) = ( tx_joint_3 * sin_q_joint_1 * sin_q_joint_2)-( tx_joint_3 * cos_q_joint_1 * cos_q_joint_2)-( tx_joint_2 * cos_q_joint_1);
    (*this)(2,3) = ((( tx_joint_2 * cos_q_joint_2)+ tx_joint_3) * sin_q_joint_3)+( tx_joint_2 * sin_q_joint_2 * cos_q_joint_3);
    (*this)(2,4) = ((( tx_joint_2 * cos_q_joint_2)+ tx_joint_3) * cos_q_joint_3)-( tx_joint_2 * sin_q_joint_2 * sin_q_joint_3);
    (*this)(3,3) = (((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * cos_q_joint_3);
    (*this)(3,4) = (((sin_q_joint_1 * sin_q_joint_2)-(cos_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(4,3) = (((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * sin_q_joint_2)+(sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(4,4) = (((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * cos_q_joint_3);
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_gripper_X_fr_world::Type_fr_gripper_X_fr_world()
{
    (*this)(0,2) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_gripper_X_fr_world& iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_gripper_X_fr_world::update(const JState& q)
{
    Scalar sin_q_joint_3  = TRAIT::sin( q(JOINT_3) );
    Scalar cos_q_joint_3  = TRAIT::cos( q(JOINT_3) );
    Scalar sin_q_joint_2  = TRAIT::sin( q(JOINT_2) );
    Scalar cos_q_joint_2  = TRAIT::cos( q(JOINT_2) );
    Scalar sin_q_joint_1  = TRAIT::sin( q(JOINT_1) );
    Scalar cos_q_joint_1  = TRAIT::cos( q(JOINT_1) );
    (*this)(0,0) = (((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * cos_q_joint_3);
    (*this)(0,1) = (((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * sin_q_joint_2)+(sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(0,3) = ((( tz_joint_1 * sin_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * cos_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((- tz_joint_1 * cos_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(0,4) = (((- tz_joint_1 * cos_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+((( tz_joint_1 * cos_q_joint_1 * cos_q_joint_2)-( tz_joint_1 * sin_q_joint_1 * sin_q_joint_2)) * cos_q_joint_3);
    (*this)(0,5) = ((( tx_joint_2 * cos_q_joint_2)+ tx_joint_3) * sin_q_joint_3)+( tx_joint_2 * sin_q_joint_2 * cos_q_joint_3);
    (*this)(1,0) = (((sin_q_joint_1 * sin_q_joint_2)-(cos_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(1,1) = (((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * cos_q_joint_3);
    (*this)(1,3) = ((( tz_joint_1 * cos_q_joint_1 * sin_q_joint_2)+( tz_joint_1 * sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+((( tz_joint_1 * sin_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * cos_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(1,4) = ((( tz_joint_1 * sin_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * cos_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((- tz_joint_1 * cos_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(1,5) = ((( tx_joint_2 * cos_q_joint_2)+ tx_joint_3) * cos_q_joint_3)-( tx_joint_2 * sin_q_joint_2 * sin_q_joint_3);
    (*this)(2,3) = ( tx_joint_3 * cos_q_joint_1 * sin_q_joint_2)+( tx_joint_3 * sin_q_joint_1 * cos_q_joint_2)+( tx_joint_2 * sin_q_joint_1);
    (*this)(2,4) = ( tx_joint_3 * sin_q_joint_1 * sin_q_joint_2)-( tx_joint_3 * cos_q_joint_1 * cos_q_joint_2)-( tx_joint_2 * cos_q_joint_1);
    (*this)(3,3) = (((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * cos_q_joint_3);
    (*this)(3,4) = (((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * sin_q_joint_2)+(sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(4,3) = (((sin_q_joint_1 * sin_q_joint_2)-(cos_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(4,4) = (((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * cos_q_joint_3);
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_world_X_fr_link_1::Type_fr_world_X_fr_link_1()
{
    (*this)(0,2) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_world_X_fr_link_1& iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_world_X_fr_link_1::update(const JState& q)
{
    Scalar sin_q_joint_1  = TRAIT::sin( q(JOINT_1) );
    Scalar cos_q_joint_1  = TRAIT::cos( q(JOINT_1) );
    (*this)(0,0) = cos_q_joint_1;
    (*this)(0,1) = -sin_q_joint_1;
    (*this)(0,3) = - tz_joint_1 * sin_q_joint_1;
    (*this)(0,4) = - tz_joint_1 * cos_q_joint_1;
    (*this)(1,0) = sin_q_joint_1;
    (*this)(1,1) = cos_q_joint_1;
    (*this)(1,3) =  tz_joint_1 * cos_q_joint_1;
    (*this)(1,4) = - tz_joint_1 * sin_q_joint_1;
    (*this)(3,3) = cos_q_joint_1;
    (*this)(3,4) = -sin_q_joint_1;
    (*this)(4,3) = sin_q_joint_1;
    (*this)(4,4) = cos_q_joint_1;
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_link_1_X_fr_world::Type_fr_link_1_X_fr_world()
{
    (*this)(0,2) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_link_1_X_fr_world& iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_link_1_X_fr_world::update(const JState& q)
{
    Scalar sin_q_joint_1  = TRAIT::sin( q(JOINT_1) );
    Scalar cos_q_joint_1  = TRAIT::cos( q(JOINT_1) );
    (*this)(0,0) = cos_q_joint_1;
    (*this)(0,1) = sin_q_joint_1;
    (*this)(0,3) = - tz_joint_1 * sin_q_joint_1;
    (*this)(0,4) =  tz_joint_1 * cos_q_joint_1;
    (*this)(1,0) = -sin_q_joint_1;
    (*this)(1,1) = cos_q_joint_1;
    (*this)(1,3) = - tz_joint_1 * cos_q_joint_1;
    (*this)(1,4) = - tz_joint_1 * sin_q_joint_1;
    (*this)(3,3) = cos_q_joint_1;
    (*this)(3,4) = sin_q_joint_1;
    (*this)(4,3) = -sin_q_joint_1;
    (*this)(4,4) = cos_q_joint_1;
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_world_X_fr_link_2::Type_fr_world_X_fr_link_2()
{
    (*this)(0,2) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_world_X_fr_link_2& iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_world_X_fr_link_2::update(const JState& q)
{
    Scalar sin_q_joint_1  = TRAIT::sin( q(JOINT_1) );
    Scalar cos_q_joint_1  = TRAIT::cos( q(JOINT_1) );
    Scalar sin_q_joint_2  = TRAIT::sin( q(JOINT_2) );
    Scalar cos_q_joint_2  = TRAIT::cos( q(JOINT_2) );
    (*this)(0,0) = (cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2);
    (*this)(0,1) = (-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2);
    (*this)(0,3) = (- tz_joint_1 * cos_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * sin_q_joint_1 * cos_q_joint_2);
    (*this)(0,4) = ( tz_joint_1 * sin_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * cos_q_joint_1 * cos_q_joint_2);
    (*this)(0,5) =  tx_joint_2 * sin_q_joint_1;
    (*this)(1,0) = (cos_q_joint_1 * sin_q_joint_2)+(sin_q_joint_1 * cos_q_joint_2);
    (*this)(1,1) = (cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2);
    (*this)(1,3) = ( tz_joint_1 * cos_q_joint_1 * cos_q_joint_2)-( tz_joint_1 * sin_q_joint_1 * sin_q_joint_2);
    (*this)(1,4) = (- tz_joint_1 * cos_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * sin_q_joint_1 * cos_q_joint_2);
    (*this)(1,5) = - tx_joint_2 * cos_q_joint_1;
    (*this)(2,3) =  tx_joint_2 * sin_q_joint_2;
    (*this)(2,4) =  tx_joint_2 * cos_q_joint_2;
    (*this)(3,3) = (cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2);
    (*this)(3,4) = (-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2);
    (*this)(4,3) = (cos_q_joint_1 * sin_q_joint_2)+(sin_q_joint_1 * cos_q_joint_2);
    (*this)(4,4) = (cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2);
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_link_2_X_fr_world::Type_fr_link_2_X_fr_world()
{
    (*this)(0,2) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_link_2_X_fr_world& iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_link_2_X_fr_world::update(const JState& q)
{
    Scalar sin_q_joint_2  = TRAIT::sin( q(JOINT_2) );
    Scalar cos_q_joint_2  = TRAIT::cos( q(JOINT_2) );
    Scalar sin_q_joint_1  = TRAIT::sin( q(JOINT_1) );
    Scalar cos_q_joint_1  = TRAIT::cos( q(JOINT_1) );
    (*this)(0,0) = (cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2);
    (*this)(0,1) = (cos_q_joint_1 * sin_q_joint_2)+(sin_q_joint_1 * cos_q_joint_2);
    (*this)(0,3) = (- tz_joint_1 * cos_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * sin_q_joint_1 * cos_q_joint_2);
    (*this)(0,4) = ( tz_joint_1 * cos_q_joint_1 * cos_q_joint_2)-( tz_joint_1 * sin_q_joint_1 * sin_q_joint_2);
    (*this)(0,5) =  tx_joint_2 * sin_q_joint_2;
    (*this)(1,0) = (-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2);
    (*this)(1,1) = (cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2);
    (*this)(1,3) = ( tz_joint_1 * sin_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * cos_q_joint_1 * cos_q_joint_2);
    (*this)(1,4) = (- tz_joint_1 * cos_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * sin_q_joint_1 * cos_q_joint_2);
    (*this)(1,5) =  tx_joint_2 * cos_q_joint_2;
    (*this)(2,3) =  tx_joint_2 * sin_q_joint_1;
    (*this)(2,4) = - tx_joint_2 * cos_q_joint_1;
    (*this)(3,3) = (cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2);
    (*this)(3,4) = (cos_q_joint_1 * sin_q_joint_2)+(sin_q_joint_1 * cos_q_joint_2);
    (*this)(4,3) = (-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2);
    (*this)(4,4) = (cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2);
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_world_X_fr_base_link::Type_fr_world_X_fr_base_link()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_world_X_fr_base_link& iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_world_X_fr_base_link::update(const JState& q)
{
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_base_link_X_fr_world::Type_fr_base_link_X_fr_world()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_base_link_X_fr_world& iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_base_link_X_fr_world::update(const JState& q)
{
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_world_X_fr_end::Type_fr_world_X_fr_end()
{
    (*this)(0,2) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_world_X_fr_end& iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_world_X_fr_end::update(const JState& q)
{
    Scalar sin_q_joint_1  = TRAIT::sin( q(JOINT_1) );
    Scalar cos_q_joint_1  = TRAIT::cos( q(JOINT_1) );
    Scalar sin_q_joint_2  = TRAIT::sin( q(JOINT_2) );
    Scalar cos_q_joint_2  = TRAIT::cos( q(JOINT_2) );
    Scalar sin_q_joint_3  = TRAIT::sin( q(JOINT_3) );
    Scalar cos_q_joint_3  = TRAIT::cos( q(JOINT_3) );
    (*this)(0,0) = (((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * cos_q_joint_3);
    (*this)(0,1) = (((sin_q_joint_1 * sin_q_joint_2)-(cos_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(0,3) = ((( tz_joint_1 * sin_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * cos_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((- tz_joint_1 * cos_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(0,4) = ((( tz_joint_1 * cos_q_joint_1 * sin_q_joint_2)+( tz_joint_1 * sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+((( tz_joint_1 * sin_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * cos_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(0,5) = ((( tx_fr_end * cos_q_joint_1 * cos_q_joint_2)-( tx_fr_end * sin_q_joint_1 * sin_q_joint_2)) * sin_q_joint_3)+((( tx_fr_end * cos_q_joint_1 * sin_q_joint_2)+( tx_fr_end * sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3)+( tx_joint_3 * cos_q_joint_1 * sin_q_joint_2)+( tx_joint_3 * sin_q_joint_1 * cos_q_joint_2)+( tx_joint_2 * sin_q_joint_1);
    (*this)(1,0) = (((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * sin_q_joint_2)+(sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(1,1) = (((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * cos_q_joint_3);
    (*this)(1,3) = (((- tz_joint_1 * cos_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+((( tz_joint_1 * cos_q_joint_1 * cos_q_joint_2)-( tz_joint_1 * sin_q_joint_1 * sin_q_joint_2)) * cos_q_joint_3);
    (*this)(1,4) = ((( tz_joint_1 * sin_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * cos_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((- tz_joint_1 * cos_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(1,5) = ((( tx_fr_end * cos_q_joint_1 * sin_q_joint_2)+( tx_fr_end * sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+((( tx_fr_end * sin_q_joint_1 * sin_q_joint_2)-( tx_fr_end * cos_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3)+( tx_joint_3 * sin_q_joint_1 * sin_q_joint_2)-( tx_joint_3 * cos_q_joint_1 * cos_q_joint_2)-( tx_joint_2 * cos_q_joint_1);
    (*this)(2,3) = ((( tx_joint_2 * cos_q_joint_2)+ tx_joint_3) * sin_q_joint_3)+( tx_joint_2 * sin_q_joint_2 * cos_q_joint_3);
    (*this)(2,4) = (- tx_joint_2 * sin_q_joint_2 * sin_q_joint_3)+((( tx_joint_2 * cos_q_joint_2)+ tx_joint_3) * cos_q_joint_3)+ tx_fr_end;
    (*this)(3,3) = (((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * cos_q_joint_3);
    (*this)(3,4) = (((sin_q_joint_1 * sin_q_joint_2)-(cos_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(4,3) = (((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * sin_q_joint_2)+(sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(4,4) = (((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * cos_q_joint_3);
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_end_X_fr_world::Type_fr_end_X_fr_world()
{
    (*this)(0,2) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_end_X_fr_world& iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_end_X_fr_world::update(const JState& q)
{
    Scalar sin_q_joint_3  = TRAIT::sin( q(JOINT_3) );
    Scalar cos_q_joint_3  = TRAIT::cos( q(JOINT_3) );
    Scalar sin_q_joint_2  = TRAIT::sin( q(JOINT_2) );
    Scalar cos_q_joint_2  = TRAIT::cos( q(JOINT_2) );
    Scalar sin_q_joint_1  = TRAIT::sin( q(JOINT_1) );
    Scalar cos_q_joint_1  = TRAIT::cos( q(JOINT_1) );
    (*this)(0,0) = (((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * cos_q_joint_3);
    (*this)(0,1) = (((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * sin_q_joint_2)+(sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(0,3) = ((( tz_joint_1 * sin_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * cos_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((- tz_joint_1 * cos_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(0,4) = (((- tz_joint_1 * cos_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+((( tz_joint_1 * cos_q_joint_1 * cos_q_joint_2)-( tz_joint_1 * sin_q_joint_1 * sin_q_joint_2)) * cos_q_joint_3);
    (*this)(0,5) = ((( tx_joint_2 * cos_q_joint_2)+ tx_joint_3) * sin_q_joint_3)+( tx_joint_2 * sin_q_joint_2 * cos_q_joint_3);
    (*this)(1,0) = (((sin_q_joint_1 * sin_q_joint_2)-(cos_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(1,1) = (((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * cos_q_joint_3);
    (*this)(1,3) = ((( tz_joint_1 * cos_q_joint_1 * sin_q_joint_2)+( tz_joint_1 * sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+((( tz_joint_1 * sin_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * cos_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(1,4) = ((( tz_joint_1 * sin_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * cos_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((- tz_joint_1 * cos_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(1,5) = (- tx_joint_2 * sin_q_joint_2 * sin_q_joint_3)+((( tx_joint_2 * cos_q_joint_2)+ tx_joint_3) * cos_q_joint_3)+ tx_fr_end;
    (*this)(2,3) = ((( tx_fr_end * cos_q_joint_1 * cos_q_joint_2)-( tx_fr_end * sin_q_joint_1 * sin_q_joint_2)) * sin_q_joint_3)+((( tx_fr_end * cos_q_joint_1 * sin_q_joint_2)+( tx_fr_end * sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3)+( tx_joint_3 * cos_q_joint_1 * sin_q_joint_2)+( tx_joint_3 * sin_q_joint_1 * cos_q_joint_2)+( tx_joint_2 * sin_q_joint_1);
    (*this)(2,4) = ((( tx_fr_end * cos_q_joint_1 * sin_q_joint_2)+( tx_fr_end * sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+((( tx_fr_end * sin_q_joint_1 * sin_q_joint_2)-( tx_fr_end * cos_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3)+( tx_joint_3 * sin_q_joint_1 * sin_q_joint_2)-( tx_joint_3 * cos_q_joint_1 * cos_q_joint_2)-( tx_joint_2 * cos_q_joint_1);
    (*this)(3,3) = (((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * cos_q_joint_3);
    (*this)(3,4) = (((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * sin_q_joint_2)+(sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(4,3) = (((sin_q_joint_1 * sin_q_joint_2)-(cos_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(4,4) = (((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * cos_q_joint_3);
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_world_X_fr_gripper_COM::Type_fr_world_X_fr_gripper_COM()
{
    (*this)(0,2) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_world_X_fr_gripper_COM& iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_world_X_fr_gripper_COM::update(const JState& q)
{
    Scalar sin_q_joint_1  = TRAIT::sin( q(JOINT_1) );
    Scalar cos_q_joint_1  = TRAIT::cos( q(JOINT_1) );
    Scalar sin_q_joint_2  = TRAIT::sin( q(JOINT_2) );
    Scalar cos_q_joint_2  = TRAIT::cos( q(JOINT_2) );
    Scalar sin_q_joint_3  = TRAIT::sin( q(JOINT_3) );
    Scalar cos_q_joint_3  = TRAIT::cos( q(JOINT_3) );
    (*this)(0,0) = (((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * cos_q_joint_3);
    (*this)(0,1) = (((sin_q_joint_1 * sin_q_joint_2)-(cos_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(0,3) = ((( tz_joint_1 * sin_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * cos_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((- tz_joint_1 * cos_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(0,4) = ((( tz_joint_1 * cos_q_joint_1 * sin_q_joint_2)+( tz_joint_1 * sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+((( tz_joint_1 * sin_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * cos_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(0,5) = ((( tx_fr_gripper_COM * cos_q_joint_1 * cos_q_joint_2)-( tx_fr_gripper_COM * sin_q_joint_1 * sin_q_joint_2)) * sin_q_joint_3)+((( tx_fr_gripper_COM * cos_q_joint_1 * sin_q_joint_2)+( tx_fr_gripper_COM * sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3)+( tx_joint_3 * cos_q_joint_1 * sin_q_joint_2)+( tx_joint_3 * sin_q_joint_1 * cos_q_joint_2)+( tx_joint_2 * sin_q_joint_1);
    (*this)(1,0) = (((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * sin_q_joint_2)+(sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(1,1) = (((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * cos_q_joint_3);
    (*this)(1,3) = (((- tz_joint_1 * cos_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+((( tz_joint_1 * cos_q_joint_1 * cos_q_joint_2)-( tz_joint_1 * sin_q_joint_1 * sin_q_joint_2)) * cos_q_joint_3);
    (*this)(1,4) = ((( tz_joint_1 * sin_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * cos_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((- tz_joint_1 * cos_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(1,5) = ((( tx_fr_gripper_COM * cos_q_joint_1 * sin_q_joint_2)+( tx_fr_gripper_COM * sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+((( tx_fr_gripper_COM * sin_q_joint_1 * sin_q_joint_2)-( tx_fr_gripper_COM * cos_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3)+( tx_joint_3 * sin_q_joint_1 * sin_q_joint_2)-( tx_joint_3 * cos_q_joint_1 * cos_q_joint_2)-( tx_joint_2 * cos_q_joint_1);
    (*this)(2,3) = ((( tx_joint_2 * cos_q_joint_2)+ tx_joint_3) * sin_q_joint_3)+( tx_joint_2 * sin_q_joint_2 * cos_q_joint_3);
    (*this)(2,4) = (- tx_joint_2 * sin_q_joint_2 * sin_q_joint_3)+((( tx_joint_2 * cos_q_joint_2)+ tx_joint_3) * cos_q_joint_3)+ tx_fr_gripper_COM;
    (*this)(3,3) = (((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * cos_q_joint_3);
    (*this)(3,4) = (((sin_q_joint_1 * sin_q_joint_2)-(cos_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(4,3) = (((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * sin_q_joint_2)+(sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(4,4) = (((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * cos_q_joint_3);
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_gripper_COM_X_fr_world::Type_fr_gripper_COM_X_fr_world()
{
    (*this)(0,2) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_gripper_COM_X_fr_world& iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_gripper_COM_X_fr_world::update(const JState& q)
{
    Scalar sin_q_joint_3  = TRAIT::sin( q(JOINT_3) );
    Scalar cos_q_joint_3  = TRAIT::cos( q(JOINT_3) );
    Scalar sin_q_joint_2  = TRAIT::sin( q(JOINT_2) );
    Scalar cos_q_joint_2  = TRAIT::cos( q(JOINT_2) );
    Scalar sin_q_joint_1  = TRAIT::sin( q(JOINT_1) );
    Scalar cos_q_joint_1  = TRAIT::cos( q(JOINT_1) );
    (*this)(0,0) = (((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * cos_q_joint_3);
    (*this)(0,1) = (((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * sin_q_joint_2)+(sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(0,3) = ((( tz_joint_1 * sin_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * cos_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((- tz_joint_1 * cos_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(0,4) = (((- tz_joint_1 * cos_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+((( tz_joint_1 * cos_q_joint_1 * cos_q_joint_2)-( tz_joint_1 * sin_q_joint_1 * sin_q_joint_2)) * cos_q_joint_3);
    (*this)(0,5) = ((( tx_joint_2 * cos_q_joint_2)+ tx_joint_3) * sin_q_joint_3)+( tx_joint_2 * sin_q_joint_2 * cos_q_joint_3);
    (*this)(1,0) = (((sin_q_joint_1 * sin_q_joint_2)-(cos_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(1,1) = (((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * cos_q_joint_3);
    (*this)(1,3) = ((( tz_joint_1 * cos_q_joint_1 * sin_q_joint_2)+( tz_joint_1 * sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+((( tz_joint_1 * sin_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * cos_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(1,4) = ((( tz_joint_1 * sin_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * cos_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((- tz_joint_1 * cos_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(1,5) = (- tx_joint_2 * sin_q_joint_2 * sin_q_joint_3)+((( tx_joint_2 * cos_q_joint_2)+ tx_joint_3) * cos_q_joint_3)+ tx_fr_gripper_COM;
    (*this)(2,3) = ((( tx_fr_gripper_COM * cos_q_joint_1 * cos_q_joint_2)-( tx_fr_gripper_COM * sin_q_joint_1 * sin_q_joint_2)) * sin_q_joint_3)+((( tx_fr_gripper_COM * cos_q_joint_1 * sin_q_joint_2)+( tx_fr_gripper_COM * sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3)+( tx_joint_3 * cos_q_joint_1 * sin_q_joint_2)+( tx_joint_3 * sin_q_joint_1 * cos_q_joint_2)+( tx_joint_2 * sin_q_joint_1);
    (*this)(2,4) = ((( tx_fr_gripper_COM * cos_q_joint_1 * sin_q_joint_2)+( tx_fr_gripper_COM * sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+((( tx_fr_gripper_COM * sin_q_joint_1 * sin_q_joint_2)-( tx_fr_gripper_COM * cos_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3)+( tx_joint_3 * sin_q_joint_1 * sin_q_joint_2)-( tx_joint_3 * cos_q_joint_1 * cos_q_joint_2)-( tx_joint_2 * cos_q_joint_1);
    (*this)(3,3) = (((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * cos_q_joint_3);
    (*this)(3,4) = (((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * sin_q_joint_2)+(sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(4,3) = (((sin_q_joint_1 * sin_q_joint_2)-(cos_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(4,4) = (((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * cos_q_joint_3);
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_world_X_fr_link_1_COM::Type_fr_world_X_fr_link_1_COM()
{
    (*this)(0,2) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) =  tx_fr_link_1_COM;    // Maxima DSL: _k__tx_fr_link_1_COM
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_world_X_fr_link_1_COM& iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_world_X_fr_link_1_COM::update(const JState& q)
{
    Scalar sin_q_joint_1  = TRAIT::sin( q(JOINT_1) );
    Scalar cos_q_joint_1  = TRAIT::cos( q(JOINT_1) );
    (*this)(0,0) = cos_q_joint_1;
    (*this)(0,1) = -sin_q_joint_1;
    (*this)(0,3) = - tz_joint_1 * sin_q_joint_1;
    (*this)(0,4) = - tz_joint_1 * cos_q_joint_1;
    (*this)(0,5) =  tx_fr_link_1_COM * sin_q_joint_1;
    (*this)(1,0) = sin_q_joint_1;
    (*this)(1,1) = cos_q_joint_1;
    (*this)(1,3) =  tz_joint_1 * cos_q_joint_1;
    (*this)(1,4) = - tz_joint_1 * sin_q_joint_1;
    (*this)(1,5) = - tx_fr_link_1_COM * cos_q_joint_1;
    (*this)(3,3) = cos_q_joint_1;
    (*this)(3,4) = -sin_q_joint_1;
    (*this)(4,3) = sin_q_joint_1;
    (*this)(4,4) = cos_q_joint_1;
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_link_1_COM_X_fr_world::Type_fr_link_1_COM_X_fr_world()
{
    (*this)(0,2) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,5) =  tx_fr_link_1_COM;    // Maxima DSL: _k__tx_fr_link_1_COM
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_link_1_COM_X_fr_world& iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_link_1_COM_X_fr_world::update(const JState& q)
{
    Scalar sin_q_joint_1  = TRAIT::sin( q(JOINT_1) );
    Scalar cos_q_joint_1  = TRAIT::cos( q(JOINT_1) );
    (*this)(0,0) = cos_q_joint_1;
    (*this)(0,1) = sin_q_joint_1;
    (*this)(0,3) = - tz_joint_1 * sin_q_joint_1;
    (*this)(0,4) =  tz_joint_1 * cos_q_joint_1;
    (*this)(1,0) = -sin_q_joint_1;
    (*this)(1,1) = cos_q_joint_1;
    (*this)(1,3) = - tz_joint_1 * cos_q_joint_1;
    (*this)(1,4) = - tz_joint_1 * sin_q_joint_1;
    (*this)(2,3) =  tx_fr_link_1_COM * sin_q_joint_1;
    (*this)(2,4) = - tx_fr_link_1_COM * cos_q_joint_1;
    (*this)(3,3) = cos_q_joint_1;
    (*this)(3,4) = sin_q_joint_1;
    (*this)(4,3) = -sin_q_joint_1;
    (*this)(4,4) = cos_q_joint_1;
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_world_X_fr_link_2_COM::Type_fr_world_X_fr_link_2_COM()
{
    (*this)(0,2) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_world_X_fr_link_2_COM& iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_world_X_fr_link_2_COM::update(const JState& q)
{
    Scalar sin_q_joint_1  = TRAIT::sin( q(JOINT_1) );
    Scalar cos_q_joint_1  = TRAIT::cos( q(JOINT_1) );
    Scalar sin_q_joint_2  = TRAIT::sin( q(JOINT_2) );
    Scalar cos_q_joint_2  = TRAIT::cos( q(JOINT_2) );
    (*this)(0,0) = (cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2);
    (*this)(0,1) = (-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2);
    (*this)(0,3) = (- tz_joint_1 * cos_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * sin_q_joint_1 * cos_q_joint_2);
    (*this)(0,4) = ( tz_joint_1 * sin_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * cos_q_joint_1 * cos_q_joint_2);
    (*this)(0,5) = ( tx_fr_link_2_COM * cos_q_joint_1 * sin_q_joint_2)+( tx_fr_link_2_COM * sin_q_joint_1 * cos_q_joint_2)+( tx_joint_2 * sin_q_joint_1);
    (*this)(1,0) = (cos_q_joint_1 * sin_q_joint_2)+(sin_q_joint_1 * cos_q_joint_2);
    (*this)(1,1) = (cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2);
    (*this)(1,3) = ( tz_joint_1 * cos_q_joint_1 * cos_q_joint_2)-( tz_joint_1 * sin_q_joint_1 * sin_q_joint_2);
    (*this)(1,4) = (- tz_joint_1 * cos_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * sin_q_joint_1 * cos_q_joint_2);
    (*this)(1,5) = ( tx_fr_link_2_COM * sin_q_joint_1 * sin_q_joint_2)-( tx_fr_link_2_COM * cos_q_joint_1 * cos_q_joint_2)-( tx_joint_2 * cos_q_joint_1);
    (*this)(2,3) =  tx_joint_2 * sin_q_joint_2;
    (*this)(2,4) = ( tx_joint_2 * cos_q_joint_2)+ tx_fr_link_2_COM;
    (*this)(3,3) = (cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2);
    (*this)(3,4) = (-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2);
    (*this)(4,3) = (cos_q_joint_1 * sin_q_joint_2)+(sin_q_joint_1 * cos_q_joint_2);
    (*this)(4,4) = (cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2);
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_link_2_COM_X_fr_world::Type_fr_link_2_COM_X_fr_world()
{
    (*this)(0,2) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_link_2_COM_X_fr_world& iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_link_2_COM_X_fr_world::update(const JState& q)
{
    Scalar sin_q_joint_2  = TRAIT::sin( q(JOINT_2) );
    Scalar cos_q_joint_2  = TRAIT::cos( q(JOINT_2) );
    Scalar sin_q_joint_1  = TRAIT::sin( q(JOINT_1) );
    Scalar cos_q_joint_1  = TRAIT::cos( q(JOINT_1) );
    (*this)(0,0) = (cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2);
    (*this)(0,1) = (cos_q_joint_1 * sin_q_joint_2)+(sin_q_joint_1 * cos_q_joint_2);
    (*this)(0,3) = (- tz_joint_1 * cos_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * sin_q_joint_1 * cos_q_joint_2);
    (*this)(0,4) = ( tz_joint_1 * cos_q_joint_1 * cos_q_joint_2)-( tz_joint_1 * sin_q_joint_1 * sin_q_joint_2);
    (*this)(0,5) =  tx_joint_2 * sin_q_joint_2;
    (*this)(1,0) = (-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2);
    (*this)(1,1) = (cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2);
    (*this)(1,3) = ( tz_joint_1 * sin_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * cos_q_joint_1 * cos_q_joint_2);
    (*this)(1,4) = (- tz_joint_1 * cos_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * sin_q_joint_1 * cos_q_joint_2);
    (*this)(1,5) = ( tx_joint_2 * cos_q_joint_2)+ tx_fr_link_2_COM;
    (*this)(2,3) = ( tx_fr_link_2_COM * cos_q_joint_1 * sin_q_joint_2)+( tx_fr_link_2_COM * sin_q_joint_1 * cos_q_joint_2)+( tx_joint_2 * sin_q_joint_1);
    (*this)(2,4) = ( tx_fr_link_2_COM * sin_q_joint_1 * sin_q_joint_2)-( tx_fr_link_2_COM * cos_q_joint_1 * cos_q_joint_2)-( tx_joint_2 * cos_q_joint_1);
    (*this)(3,3) = (cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2);
    (*this)(3,4) = (cos_q_joint_1 * sin_q_joint_2)+(sin_q_joint_1 * cos_q_joint_2);
    (*this)(4,3) = (-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2);
    (*this)(4,4) = (cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2);
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_world_X_fr_world_COM::Type_fr_world_X_fr_world_COM()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_world_X_fr_world_COM& iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_world_X_fr_world_COM::update(const JState& q)
{
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_world_COM_X_fr_world::Type_fr_world_COM_X_fr_world()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_world_COM_X_fr_world& iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_world_COM_X_fr_world::update(const JState& q)
{
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_world_X_fr_joint_1::Type_fr_world_X_fr_joint_1()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = - tz_joint_1;    // Maxima DSL: -_k__tz_joint_1
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) =  tz_joint_1;    // Maxima DSL: _k__tz_joint_1
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_world_X_fr_joint_1& iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_world_X_fr_joint_1::update(const JState& q)
{
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_world_X_fr_joint_2::Type_fr_world_X_fr_joint_2()
{
    (*this)(0,2) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) =  tx_joint_2;    // Maxima DSL: _k__tx_joint_2
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_world_X_fr_joint_2& iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_world_X_fr_joint_2::update(const JState& q)
{
    Scalar sin_q_joint_1  = TRAIT::sin( q(JOINT_1) );
    Scalar cos_q_joint_1  = TRAIT::cos( q(JOINT_1) );
    (*this)(0,0) = cos_q_joint_1;
    (*this)(0,1) = -sin_q_joint_1;
    (*this)(0,3) = - tz_joint_1 * sin_q_joint_1;
    (*this)(0,4) = - tz_joint_1 * cos_q_joint_1;
    (*this)(0,5) =  tx_joint_2 * sin_q_joint_1;
    (*this)(1,0) = sin_q_joint_1;
    (*this)(1,1) = cos_q_joint_1;
    (*this)(1,3) =  tz_joint_1 * cos_q_joint_1;
    (*this)(1,4) = - tz_joint_1 * sin_q_joint_1;
    (*this)(1,5) = - tx_joint_2 * cos_q_joint_1;
    (*this)(3,3) = cos_q_joint_1;
    (*this)(3,4) = -sin_q_joint_1;
    (*this)(4,3) = sin_q_joint_1;
    (*this)(4,4) = cos_q_joint_1;
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_world_X_fr_joint_3::Type_fr_world_X_fr_joint_3()
{
    (*this)(0,2) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_world_X_fr_joint_3& iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_world_X_fr_joint_3::update(const JState& q)
{
    Scalar sin_q_joint_1  = TRAIT::sin( q(JOINT_1) );
    Scalar cos_q_joint_1  = TRAIT::cos( q(JOINT_1) );
    Scalar sin_q_joint_2  = TRAIT::sin( q(JOINT_2) );
    Scalar cos_q_joint_2  = TRAIT::cos( q(JOINT_2) );
    (*this)(0,0) = (cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2);
    (*this)(0,1) = (-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2);
    (*this)(0,3) = (- tz_joint_1 * cos_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * sin_q_joint_1 * cos_q_joint_2);
    (*this)(0,4) = ( tz_joint_1 * sin_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * cos_q_joint_1 * cos_q_joint_2);
    (*this)(0,5) = ( tx_joint_3 * cos_q_joint_1 * sin_q_joint_2)+( tx_joint_3 * sin_q_joint_1 * cos_q_joint_2)+( tx_joint_2 * sin_q_joint_1);
    (*this)(1,0) = (cos_q_joint_1 * sin_q_joint_2)+(sin_q_joint_1 * cos_q_joint_2);
    (*this)(1,1) = (cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2);
    (*this)(1,3) = ( tz_joint_1 * cos_q_joint_1 * cos_q_joint_2)-( tz_joint_1 * sin_q_joint_1 * sin_q_joint_2);
    (*this)(1,4) = (- tz_joint_1 * cos_q_joint_1 * sin_q_joint_2)-( tz_joint_1 * sin_q_joint_1 * cos_q_joint_2);
    (*this)(1,5) = ( tx_joint_3 * sin_q_joint_1 * sin_q_joint_2)-( tx_joint_3 * cos_q_joint_1 * cos_q_joint_2)-( tx_joint_2 * cos_q_joint_1);
    (*this)(2,3) =  tx_joint_2 * sin_q_joint_2;
    (*this)(2,4) = ( tx_joint_2 * cos_q_joint_2)+ tx_joint_3;
    (*this)(3,3) = (cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2);
    (*this)(3,4) = (-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2);
    (*this)(4,3) = (cos_q_joint_1 * sin_q_joint_2)+(sin_q_joint_1 * cos_q_joint_2);
    (*this)(4,4) = (cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2);
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_link_2_X_fr_link_1::Type_fr_link_2_X_fr_link_1()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = - tx_joint_2;    // Maxima DSL: -_k__tx_joint_2
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_link_2_X_fr_link_1& iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_link_2_X_fr_link_1::update(const JState& q)
{
    Scalar sin_q_joint_2  = TRAIT::sin( q(JOINT_2) );
    Scalar cos_q_joint_2  = TRAIT::cos( q(JOINT_2) );
    (*this)(0,0) = cos_q_joint_2;
    (*this)(0,1) = sin_q_joint_2;
    (*this)(0,5) =  tx_joint_2 * sin_q_joint_2;
    (*this)(1,0) = -sin_q_joint_2;
    (*this)(1,1) = cos_q_joint_2;
    (*this)(1,5) =  tx_joint_2 * cos_q_joint_2;
    (*this)(3,3) = cos_q_joint_2;
    (*this)(3,4) = sin_q_joint_2;
    (*this)(4,3) = -sin_q_joint_2;
    (*this)(4,4) = cos_q_joint_2;
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_link_1_X_fr_link_2::Type_fr_link_1_X_fr_link_2()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = - tx_joint_2;    // Maxima DSL: -_k__tx_joint_2
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_link_1_X_fr_link_2& iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_link_1_X_fr_link_2::update(const JState& q)
{
    Scalar sin_q_joint_2  = TRAIT::sin( q(JOINT_2) );
    Scalar cos_q_joint_2  = TRAIT::cos( q(JOINT_2) );
    (*this)(0,0) = cos_q_joint_2;
    (*this)(0,1) = -sin_q_joint_2;
    (*this)(1,0) = sin_q_joint_2;
    (*this)(1,1) = cos_q_joint_2;
    (*this)(2,3) =  tx_joint_2 * sin_q_joint_2;
    (*this)(2,4) =  tx_joint_2 * cos_q_joint_2;
    (*this)(3,3) = cos_q_joint_2;
    (*this)(3,4) = -sin_q_joint_2;
    (*this)(4,3) = sin_q_joint_2;
    (*this)(4,4) = cos_q_joint_2;
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_gripper_X_fr_link_2::Type_fr_gripper_X_fr_link_2()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = - tx_joint_3;    // Maxima DSL: -_k__tx_joint_3
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_gripper_X_fr_link_2& iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_gripper_X_fr_link_2::update(const JState& q)
{
    Scalar sin_q_joint_3  = TRAIT::sin( q(JOINT_3) );
    Scalar cos_q_joint_3  = TRAIT::cos( q(JOINT_3) );
    (*this)(0,0) = cos_q_joint_3;
    (*this)(0,1) = sin_q_joint_3;
    (*this)(0,5) =  tx_joint_3 * sin_q_joint_3;
    (*this)(1,0) = -sin_q_joint_3;
    (*this)(1,1) = cos_q_joint_3;
    (*this)(1,5) =  tx_joint_3 * cos_q_joint_3;
    (*this)(3,3) = cos_q_joint_3;
    (*this)(3,4) = sin_q_joint_3;
    (*this)(4,3) = -sin_q_joint_3;
    (*this)(4,4) = cos_q_joint_3;
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_link_2_X_fr_gripper::Type_fr_link_2_X_fr_gripper()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = - tx_joint_3;    // Maxima DSL: -_k__tx_joint_3
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_link_2_X_fr_gripper& iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_link_2_X_fr_gripper::update(const JState& q)
{
    Scalar sin_q_joint_3  = TRAIT::sin( q(JOINT_3) );
    Scalar cos_q_joint_3  = TRAIT::cos( q(JOINT_3) );
    (*this)(0,0) = cos_q_joint_3;
    (*this)(0,1) = -sin_q_joint_3;
    (*this)(1,0) = sin_q_joint_3;
    (*this)(1,1) = cos_q_joint_3;
    (*this)(2,3) =  tx_joint_3 * sin_q_joint_3;
    (*this)(2,4) =  tx_joint_3 * cos_q_joint_3;
    (*this)(3,3) = cos_q_joint_3;
    (*this)(3,4) = -sin_q_joint_3;
    (*this)(4,3) = sin_q_joint_3;
    (*this)(4,4) = cos_q_joint_3;
    return *this;
}

template <typename TRAIT>
iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_world_X_fr_gripper::Type_fr_world_X_fr_gripper()
{
    (*this)(0,2) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) =  tz_joint_1;    // Maxima DSL: _k__tz_joint_1
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_world_X_fr_gripper& iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_world_X_fr_gripper::update(const JState& q)
{
    Scalar sin_q_joint_1  = TRAIT::sin( q(JOINT_1) );
    Scalar cos_q_joint_1  = TRAIT::cos( q(JOINT_1) );
    Scalar sin_q_joint_2  = TRAIT::sin( q(JOINT_2) );
    Scalar cos_q_joint_2  = TRAIT::cos( q(JOINT_2) );
    Scalar sin_q_joint_3  = TRAIT::sin( q(JOINT_3) );
    Scalar cos_q_joint_3  = TRAIT::cos( q(JOINT_3) );
    (*this)(0,0) = (((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * cos_q_joint_3);
    (*this)(0,1) = (((sin_q_joint_1 * sin_q_joint_2)-(cos_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(0,3) = (- tx_joint_3 * sin_q_joint_1 * sin_q_joint_2)+( tx_joint_3 * cos_q_joint_1 * cos_q_joint_2)+( tx_joint_2 * cos_q_joint_1);
    (*this)(1,0) = (((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * sin_q_joint_2)+(sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(1,1) = (((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * cos_q_joint_3);
    (*this)(1,3) = ( tx_joint_3 * cos_q_joint_1 * sin_q_joint_2)+( tx_joint_3 * sin_q_joint_1 * cos_q_joint_2)+( tx_joint_2 * sin_q_joint_1);
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_gripper_X_fr_world::Type_fr_gripper_X_fr_world()
{
    (*this)(0,2) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = - tz_joint_1;    // Maxima DSL: -_k__tz_joint_1
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_gripper_X_fr_world& iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_gripper_X_fr_world::update(const JState& q)
{
    Scalar sin_q_joint_3  = TRAIT::sin( q(JOINT_3) );
    Scalar cos_q_joint_3  = TRAIT::cos( q(JOINT_3) );
    Scalar sin_q_joint_2  = TRAIT::sin( q(JOINT_2) );
    Scalar cos_q_joint_2  = TRAIT::cos( q(JOINT_2) );
    Scalar sin_q_joint_1  = TRAIT::sin( q(JOINT_1) );
    Scalar cos_q_joint_1  = TRAIT::cos( q(JOINT_1) );
    (*this)(0,0) = (((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * cos_q_joint_3);
    (*this)(0,1) = (((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * sin_q_joint_2)+(sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(0,3) = ( tx_joint_2 * sin_q_joint_2 * sin_q_joint_3)+(((- tx_joint_2 * cos_q_joint_2)- tx_joint_3) * cos_q_joint_3);
    (*this)(1,0) = (((sin_q_joint_1 * sin_q_joint_2)-(cos_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(1,1) = (((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * cos_q_joint_3);
    (*this)(1,3) = ((( tx_joint_2 * cos_q_joint_2)+ tx_joint_3) * sin_q_joint_3)+( tx_joint_2 * sin_q_joint_2 * cos_q_joint_3);
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_world_X_fr_link_1::Type_fr_world_X_fr_link_1()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) =  tz_joint_1;    // Maxima DSL: _k__tz_joint_1
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_world_X_fr_link_1& iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_world_X_fr_link_1::update(const JState& q)
{
    Scalar sin_q_joint_1  = TRAIT::sin( q(JOINT_1) );
    Scalar cos_q_joint_1  = TRAIT::cos( q(JOINT_1) );
    (*this)(0,0) = cos_q_joint_1;
    (*this)(0,1) = -sin_q_joint_1;
    (*this)(1,0) = sin_q_joint_1;
    (*this)(1,1) = cos_q_joint_1;
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link_1_X_fr_world::Type_fr_link_1_X_fr_world()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = - tz_joint_1;    // Maxima DSL: -_k__tz_joint_1
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link_1_X_fr_world& iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link_1_X_fr_world::update(const JState& q)
{
    Scalar sin_q_joint_1  = TRAIT::sin( q(JOINT_1) );
    Scalar cos_q_joint_1  = TRAIT::cos( q(JOINT_1) );
    (*this)(0,0) = cos_q_joint_1;
    (*this)(0,1) = sin_q_joint_1;
    (*this)(1,0) = -sin_q_joint_1;
    (*this)(1,1) = cos_q_joint_1;
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_world_X_fr_link_2::Type_fr_world_X_fr_link_2()
{
    (*this)(0,2) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) =  tz_joint_1;    // Maxima DSL: _k__tz_joint_1
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_world_X_fr_link_2& iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_world_X_fr_link_2::update(const JState& q)
{
    Scalar sin_q_joint_1  = TRAIT::sin( q(JOINT_1) );
    Scalar cos_q_joint_1  = TRAIT::cos( q(JOINT_1) );
    Scalar sin_q_joint_2  = TRAIT::sin( q(JOINT_2) );
    Scalar cos_q_joint_2  = TRAIT::cos( q(JOINT_2) );
    (*this)(0,0) = (cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2);
    (*this)(0,1) = (-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2);
    (*this)(0,3) =  tx_joint_2 * cos_q_joint_1;
    (*this)(1,0) = (cos_q_joint_1 * sin_q_joint_2)+(sin_q_joint_1 * cos_q_joint_2);
    (*this)(1,1) = (cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2);
    (*this)(1,3) =  tx_joint_2 * sin_q_joint_1;
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link_2_X_fr_world::Type_fr_link_2_X_fr_world()
{
    (*this)(0,2) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = - tz_joint_1;    // Maxima DSL: -_k__tz_joint_1
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link_2_X_fr_world& iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link_2_X_fr_world::update(const JState& q)
{
    Scalar sin_q_joint_2  = TRAIT::sin( q(JOINT_2) );
    Scalar cos_q_joint_2  = TRAIT::cos( q(JOINT_2) );
    Scalar sin_q_joint_1  = TRAIT::sin( q(JOINT_1) );
    Scalar cos_q_joint_1  = TRAIT::cos( q(JOINT_1) );
    (*this)(0,0) = (cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2);
    (*this)(0,1) = (cos_q_joint_1 * sin_q_joint_2)+(sin_q_joint_1 * cos_q_joint_2);
    (*this)(0,3) = - tx_joint_2 * cos_q_joint_2;
    (*this)(1,0) = (-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2);
    (*this)(1,1) = (cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2);
    (*this)(1,3) =  tx_joint_2 * sin_q_joint_2;
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_world_X_fr_base_link::Type_fr_world_X_fr_base_link()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_world_X_fr_base_link& iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_world_X_fr_base_link::update(const JState& q)
{
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_link_X_fr_world::Type_fr_base_link_X_fr_world()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_link_X_fr_world& iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_link_X_fr_world::update(const JState& q)
{
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_world_X_fr_end::Type_fr_world_X_fr_end()
{
    (*this)(0,2) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) =  tz_joint_1;    // Maxima DSL: _k__tz_joint_1
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_world_X_fr_end& iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_world_X_fr_end::update(const JState& q)
{
    Scalar sin_q_joint_1  = TRAIT::sin( q(JOINT_1) );
    Scalar cos_q_joint_1  = TRAIT::cos( q(JOINT_1) );
    Scalar sin_q_joint_2  = TRAIT::sin( q(JOINT_2) );
    Scalar cos_q_joint_2  = TRAIT::cos( q(JOINT_2) );
    Scalar sin_q_joint_3  = TRAIT::sin( q(JOINT_3) );
    Scalar cos_q_joint_3  = TRAIT::cos( q(JOINT_3) );
    (*this)(0,0) = (((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * cos_q_joint_3);
    (*this)(0,1) = (((sin_q_joint_1 * sin_q_joint_2)-(cos_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(0,3) = (((- tx_fr_end * cos_q_joint_1 * sin_q_joint_2)-( tx_fr_end * sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+((( tx_fr_end * cos_q_joint_1 * cos_q_joint_2)-( tx_fr_end * sin_q_joint_1 * sin_q_joint_2)) * cos_q_joint_3)-( tx_joint_3 * sin_q_joint_1 * sin_q_joint_2)+( tx_joint_3 * cos_q_joint_1 * cos_q_joint_2)+( tx_joint_2 * cos_q_joint_1);
    (*this)(1,0) = (((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * sin_q_joint_2)+(sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(1,1) = (((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * cos_q_joint_3);
    (*this)(1,3) = ((( tx_fr_end * cos_q_joint_1 * cos_q_joint_2)-( tx_fr_end * sin_q_joint_1 * sin_q_joint_2)) * sin_q_joint_3)+((( tx_fr_end * cos_q_joint_1 * sin_q_joint_2)+( tx_fr_end * sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3)+( tx_joint_3 * cos_q_joint_1 * sin_q_joint_2)+( tx_joint_3 * sin_q_joint_1 * cos_q_joint_2)+( tx_joint_2 * sin_q_joint_1);
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_end_X_fr_world::Type_fr_end_X_fr_world()
{
    (*this)(0,2) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = - tz_joint_1;    // Maxima DSL: -_k__tz_joint_1
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_end_X_fr_world& iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_end_X_fr_world::update(const JState& q)
{
    Scalar sin_q_joint_3  = TRAIT::sin( q(JOINT_3) );
    Scalar cos_q_joint_3  = TRAIT::cos( q(JOINT_3) );
    Scalar sin_q_joint_2  = TRAIT::sin( q(JOINT_2) );
    Scalar cos_q_joint_2  = TRAIT::cos( q(JOINT_2) );
    Scalar sin_q_joint_1  = TRAIT::sin( q(JOINT_1) );
    Scalar cos_q_joint_1  = TRAIT::cos( q(JOINT_1) );
    (*this)(0,0) = (((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * cos_q_joint_3);
    (*this)(0,1) = (((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * sin_q_joint_2)+(sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(0,3) = ( tx_joint_2 * sin_q_joint_2 * sin_q_joint_3)+(((- tx_joint_2 * cos_q_joint_2)- tx_joint_3) * cos_q_joint_3)- tx_fr_end;
    (*this)(1,0) = (((sin_q_joint_1 * sin_q_joint_2)-(cos_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(1,1) = (((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * cos_q_joint_3);
    (*this)(1,3) = ((( tx_joint_2 * cos_q_joint_2)+ tx_joint_3) * sin_q_joint_3)+( tx_joint_2 * sin_q_joint_2 * cos_q_joint_3);
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_world_X_fr_gripper_COM::Type_fr_world_X_fr_gripper_COM()
{
    (*this)(0,2) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) =  tz_joint_1;    // Maxima DSL: _k__tz_joint_1
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_world_X_fr_gripper_COM& iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_world_X_fr_gripper_COM::update(const JState& q)
{
    Scalar sin_q_joint_1  = TRAIT::sin( q(JOINT_1) );
    Scalar cos_q_joint_1  = TRAIT::cos( q(JOINT_1) );
    Scalar sin_q_joint_2  = TRAIT::sin( q(JOINT_2) );
    Scalar cos_q_joint_2  = TRAIT::cos( q(JOINT_2) );
    Scalar sin_q_joint_3  = TRAIT::sin( q(JOINT_3) );
    Scalar cos_q_joint_3  = TRAIT::cos( q(JOINT_3) );
    (*this)(0,0) = (((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * cos_q_joint_3);
    (*this)(0,1) = (((sin_q_joint_1 * sin_q_joint_2)-(cos_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(0,3) = (((- tx_fr_gripper_COM * cos_q_joint_1 * sin_q_joint_2)-( tx_fr_gripper_COM * sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+((( tx_fr_gripper_COM * cos_q_joint_1 * cos_q_joint_2)-( tx_fr_gripper_COM * sin_q_joint_1 * sin_q_joint_2)) * cos_q_joint_3)-( tx_joint_3 * sin_q_joint_1 * sin_q_joint_2)+( tx_joint_3 * cos_q_joint_1 * cos_q_joint_2)+( tx_joint_2 * cos_q_joint_1);
    (*this)(1,0) = (((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * sin_q_joint_2)+(sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(1,1) = (((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * cos_q_joint_3);
    (*this)(1,3) = ((( tx_fr_gripper_COM * cos_q_joint_1 * cos_q_joint_2)-( tx_fr_gripper_COM * sin_q_joint_1 * sin_q_joint_2)) * sin_q_joint_3)+((( tx_fr_gripper_COM * cos_q_joint_1 * sin_q_joint_2)+( tx_fr_gripper_COM * sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3)+( tx_joint_3 * cos_q_joint_1 * sin_q_joint_2)+( tx_joint_3 * sin_q_joint_1 * cos_q_joint_2)+( tx_joint_2 * sin_q_joint_1);
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_gripper_COM_X_fr_world::Type_fr_gripper_COM_X_fr_world()
{
    (*this)(0,2) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = - tz_joint_1;    // Maxima DSL: -_k__tz_joint_1
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_gripper_COM_X_fr_world& iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_gripper_COM_X_fr_world::update(const JState& q)
{
    Scalar sin_q_joint_3  = TRAIT::sin( q(JOINT_3) );
    Scalar cos_q_joint_3  = TRAIT::cos( q(JOINT_3) );
    Scalar sin_q_joint_2  = TRAIT::sin( q(JOINT_2) );
    Scalar cos_q_joint_2  = TRAIT::cos( q(JOINT_2) );
    Scalar sin_q_joint_1  = TRAIT::sin( q(JOINT_1) );
    Scalar cos_q_joint_1  = TRAIT::cos( q(JOINT_1) );
    (*this)(0,0) = (((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * cos_q_joint_3);
    (*this)(0,1) = (((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * sin_q_joint_2)+(sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(0,3) = ( tx_joint_2 * sin_q_joint_2 * sin_q_joint_3)+(((- tx_joint_2 * cos_q_joint_2)- tx_joint_3) * cos_q_joint_3)- tx_fr_gripper_COM;
    (*this)(1,0) = (((sin_q_joint_1 * sin_q_joint_2)-(cos_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * cos_q_joint_3);
    (*this)(1,1) = (((-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2)) * sin_q_joint_3)+(((cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2)) * cos_q_joint_3);
    (*this)(1,3) = ((( tx_joint_2 * cos_q_joint_2)+ tx_joint_3) * sin_q_joint_3)+( tx_joint_2 * sin_q_joint_2 * cos_q_joint_3);
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_world_X_fr_link_1_COM::Type_fr_world_X_fr_link_1_COM()
{
    (*this)(0,2) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) =  tz_joint_1;    // Maxima DSL: _k__tz_joint_1
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_world_X_fr_link_1_COM& iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_world_X_fr_link_1_COM::update(const JState& q)
{
    Scalar sin_q_joint_1  = TRAIT::sin( q(JOINT_1) );
    Scalar cos_q_joint_1  = TRAIT::cos( q(JOINT_1) );
    (*this)(0,0) = cos_q_joint_1;
    (*this)(0,1) = -sin_q_joint_1;
    (*this)(0,3) =  tx_fr_link_1_COM * cos_q_joint_1;
    (*this)(1,0) = sin_q_joint_1;
    (*this)(1,1) = cos_q_joint_1;
    (*this)(1,3) =  tx_fr_link_1_COM * sin_q_joint_1;
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link_1_COM_X_fr_world::Type_fr_link_1_COM_X_fr_world()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = - tx_fr_link_1_COM;    // Maxima DSL: -_k__tx_fr_link_1_COM
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = - tz_joint_1;    // Maxima DSL: -_k__tz_joint_1
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link_1_COM_X_fr_world& iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link_1_COM_X_fr_world::update(const JState& q)
{
    Scalar sin_q_joint_1  = TRAIT::sin( q(JOINT_1) );
    Scalar cos_q_joint_1  = TRAIT::cos( q(JOINT_1) );
    (*this)(0,0) = cos_q_joint_1;
    (*this)(0,1) = sin_q_joint_1;
    (*this)(1,0) = -sin_q_joint_1;
    (*this)(1,1) = cos_q_joint_1;
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_world_X_fr_link_2_COM::Type_fr_world_X_fr_link_2_COM()
{
    (*this)(0,2) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) =  tz_joint_1;    // Maxima DSL: _k__tz_joint_1
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_world_X_fr_link_2_COM& iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_world_X_fr_link_2_COM::update(const JState& q)
{
    Scalar sin_q_joint_1  = TRAIT::sin( q(JOINT_1) );
    Scalar cos_q_joint_1  = TRAIT::cos( q(JOINT_1) );
    Scalar sin_q_joint_2  = TRAIT::sin( q(JOINT_2) );
    Scalar cos_q_joint_2  = TRAIT::cos( q(JOINT_2) );
    (*this)(0,0) = (cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2);
    (*this)(0,1) = (-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2);
    (*this)(0,3) = (- tx_fr_link_2_COM * sin_q_joint_1 * sin_q_joint_2)+( tx_fr_link_2_COM * cos_q_joint_1 * cos_q_joint_2)+( tx_joint_2 * cos_q_joint_1);
    (*this)(1,0) = (cos_q_joint_1 * sin_q_joint_2)+(sin_q_joint_1 * cos_q_joint_2);
    (*this)(1,1) = (cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2);
    (*this)(1,3) = ( tx_fr_link_2_COM * cos_q_joint_1 * sin_q_joint_2)+( tx_fr_link_2_COM * sin_q_joint_1 * cos_q_joint_2)+( tx_joint_2 * sin_q_joint_1);
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link_2_COM_X_fr_world::Type_fr_link_2_COM_X_fr_world()
{
    (*this)(0,2) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = - tz_joint_1;    // Maxima DSL: -_k__tz_joint_1
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link_2_COM_X_fr_world& iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link_2_COM_X_fr_world::update(const JState& q)
{
    Scalar sin_q_joint_2  = TRAIT::sin( q(JOINT_2) );
    Scalar cos_q_joint_2  = TRAIT::cos( q(JOINT_2) );
    Scalar sin_q_joint_1  = TRAIT::sin( q(JOINT_1) );
    Scalar cos_q_joint_1  = TRAIT::cos( q(JOINT_1) );
    (*this)(0,0) = (cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2);
    (*this)(0,1) = (cos_q_joint_1 * sin_q_joint_2)+(sin_q_joint_1 * cos_q_joint_2);
    (*this)(0,3) = (- tx_joint_2 * cos_q_joint_2)- tx_fr_link_2_COM;
    (*this)(1,0) = (-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2);
    (*this)(1,1) = (cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2);
    (*this)(1,3) =  tx_joint_2 * sin_q_joint_2;
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_world_X_fr_world_COM::Type_fr_world_X_fr_world_COM()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_world_X_fr_world_COM& iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_world_X_fr_world_COM::update(const JState& q)
{
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_world_COM_X_fr_world::Type_fr_world_COM_X_fr_world()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_world_COM_X_fr_world& iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_world_COM_X_fr_world::update(const JState& q)
{
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_world_X_fr_joint_1::Type_fr_world_X_fr_joint_1()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) =  tz_joint_1;    // Maxima DSL: _k__tz_joint_1
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_world_X_fr_joint_1& iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_world_X_fr_joint_1::update(const JState& q)
{
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_world_X_fr_joint_2::Type_fr_world_X_fr_joint_2()
{
    (*this)(0,2) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) =  tz_joint_1;    // Maxima DSL: _k__tz_joint_1
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_world_X_fr_joint_2& iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_world_X_fr_joint_2::update(const JState& q)
{
    Scalar sin_q_joint_1  = TRAIT::sin( q(JOINT_1) );
    Scalar cos_q_joint_1  = TRAIT::cos( q(JOINT_1) );
    (*this)(0,0) = cos_q_joint_1;
    (*this)(0,1) = -sin_q_joint_1;
    (*this)(0,3) =  tx_joint_2 * cos_q_joint_1;
    (*this)(1,0) = sin_q_joint_1;
    (*this)(1,1) = cos_q_joint_1;
    (*this)(1,3) =  tx_joint_2 * sin_q_joint_1;
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_world_X_fr_joint_3::Type_fr_world_X_fr_joint_3()
{
    (*this)(0,2) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) =  tz_joint_1;    // Maxima DSL: _k__tz_joint_1
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_world_X_fr_joint_3& iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_world_X_fr_joint_3::update(const JState& q)
{
    Scalar sin_q_joint_1  = TRAIT::sin( q(JOINT_1) );
    Scalar cos_q_joint_1  = TRAIT::cos( q(JOINT_1) );
    Scalar sin_q_joint_2  = TRAIT::sin( q(JOINT_2) );
    Scalar cos_q_joint_2  = TRAIT::cos( q(JOINT_2) );
    (*this)(0,0) = (cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2);
    (*this)(0,1) = (-cos_q_joint_1 * sin_q_joint_2)-(sin_q_joint_1 * cos_q_joint_2);
    (*this)(0,3) = (- tx_joint_3 * sin_q_joint_1 * sin_q_joint_2)+( tx_joint_3 * cos_q_joint_1 * cos_q_joint_2)+( tx_joint_2 * cos_q_joint_1);
    (*this)(1,0) = (cos_q_joint_1 * sin_q_joint_2)+(sin_q_joint_1 * cos_q_joint_2);
    (*this)(1,1) = (cos_q_joint_1 * cos_q_joint_2)-(sin_q_joint_1 * sin_q_joint_2);
    (*this)(1,3) = ( tx_joint_3 * cos_q_joint_1 * sin_q_joint_2)+( tx_joint_3 * sin_q_joint_1 * cos_q_joint_2)+( tx_joint_2 * sin_q_joint_1);
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link_2_X_fr_link_1::Type_fr_link_2_X_fr_link_1()
{
    (*this)(0,2) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link_2_X_fr_link_1& iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link_2_X_fr_link_1::update(const JState& q)
{
    Scalar sin_q_joint_2  = TRAIT::sin( q(JOINT_2) );
    Scalar cos_q_joint_2  = TRAIT::cos( q(JOINT_2) );
    (*this)(0,0) = cos_q_joint_2;
    (*this)(0,1) = sin_q_joint_2;
    (*this)(0,3) = - tx_joint_2 * cos_q_joint_2;
    (*this)(1,0) = -sin_q_joint_2;
    (*this)(1,1) = cos_q_joint_2;
    (*this)(1,3) =  tx_joint_2 * sin_q_joint_2;
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link_1_X_fr_link_2::Type_fr_link_1_X_fr_link_2()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) =  tx_joint_2;    // Maxima DSL: _k__tx_joint_2
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link_1_X_fr_link_2& iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link_1_X_fr_link_2::update(const JState& q)
{
    Scalar sin_q_joint_2  = TRAIT::sin( q(JOINT_2) );
    Scalar cos_q_joint_2  = TRAIT::cos( q(JOINT_2) );
    (*this)(0,0) = cos_q_joint_2;
    (*this)(0,1) = -sin_q_joint_2;
    (*this)(1,0) = sin_q_joint_2;
    (*this)(1,1) = cos_q_joint_2;
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_gripper_X_fr_link_2::Type_fr_gripper_X_fr_link_2()
{
    (*this)(0,2) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_gripper_X_fr_link_2& iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_gripper_X_fr_link_2::update(const JState& q)
{
    Scalar sin_q_joint_3  = TRAIT::sin( q(JOINT_3) );
    Scalar cos_q_joint_3  = TRAIT::cos( q(JOINT_3) );
    (*this)(0,0) = cos_q_joint_3;
    (*this)(0,1) = sin_q_joint_3;
    (*this)(0,3) = - tx_joint_3 * cos_q_joint_3;
    (*this)(1,0) = -sin_q_joint_3;
    (*this)(1,1) = cos_q_joint_3;
    (*this)(1,3) =  tx_joint_3 * sin_q_joint_3;
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link_2_X_fr_gripper::Type_fr_link_2_X_fr_gripper()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) =  tx_joint_3;    // Maxima DSL: _k__tx_joint_3
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link_2_X_fr_gripper& iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link_2_X_fr_gripper::update(const JState& q)
{
    Scalar sin_q_joint_3  = TRAIT::sin( q(JOINT_3) );
    Scalar cos_q_joint_3  = TRAIT::cos( q(JOINT_3) );
    (*this)(0,0) = cos_q_joint_3;
    (*this)(0,1) = -sin_q_joint_3;
    (*this)(1,0) = sin_q_joint_3;
    (*this)(1,1) = cos_q_joint_3;
    return *this;
}

