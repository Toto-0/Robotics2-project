#ifndef IIT_ROBOT_BALLBOT_INERTIA_PROPERTIES_H_
#define IIT_ROBOT_BALLBOT_INERTIA_PROPERTIES_H_

#include <iit/rbd/rbd.h>
#include <iit/rbd/InertiaMatrix.h>
#include <iit/rbd/utils.h>
#include <iit/rbd/traits/DoubleTrait.h>
#include "model_constants.h"

#include "declarations.h"

namespace iit {
namespace Ballbot {
/**
 * This namespace encloses classes and functions related to the Dynamics
 * of the robot Ballbot.
 */
namespace dyn {

using InertiaMatrix = iit::rbd::InertiaMatrixDense;

namespace tpl {

template <typename TRAIT>
class InertiaProperties {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        typedef typename TRAIT::Scalar Scalar;
        typedef iit::rbd::Core<Scalar> CoreS;
        typedef iit::rbd::tpl::InertiaMatrixDense<Scalar> IMatrix;
        typedef typename CoreS::Vector3 Vec3d;

        InertiaProperties();
        ~InertiaProperties();
        const IMatrix& getTensor_link_1() const;
        const IMatrix& getTensor_link_2() const;
        const IMatrix& getTensor_gripper() const;
        Scalar getMass_link_1() const;
        Scalar getMass_link_2() const;
        Scalar getMass_gripper() const;
        const Vec3d& getCOM_link_1() const;
        const Vec3d& getCOM_link_2() const;
        const Vec3d& getCOM_gripper() const;
        Scalar getTotalMass() const;

    private:

        IMatrix tensor_link_1;
        IMatrix tensor_link_2;
        IMatrix tensor_gripper;
        Vec3d com_link_1;
        Vec3d com_link_2;
        Vec3d com_gripper;
};

template <typename TRAIT>
inline InertiaProperties<TRAIT>::~InertiaProperties() {}

template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_link_1() const {
    return this->tensor_link_1;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_link_2() const {
    return this->tensor_link_2;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_gripper() const {
    return this->tensor_gripper;
}

template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getMass_link_1() const {
    return this->tensor_link_1.getMass();
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getMass_link_2() const {
    return this->tensor_link_2.getMass();
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getMass_gripper() const {
    return this->tensor_gripper.getMass();
}

template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_link_1() const {
    return this->com_link_1;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_link_2() const {
    return this->com_link_2;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_gripper() const {
    return this->com_gripper;
}

template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getTotalMass() const {
    return m_link_1 + m_link_2 + m_gripper;
}

}

using InertiaProperties = tpl::InertiaProperties<rbd::DoubleTrait>;

}
}
}

#include "inertia_properties.impl.h"

#endif
