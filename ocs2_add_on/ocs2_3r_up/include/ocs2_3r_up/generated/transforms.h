#ifndef BALLBOT_TRANSFORMS_H_
#define BALLBOT_TRANSFORMS_H_

#include <Eigen/Dense>
#include <iit/rbd/TransformsBase.h>
#include "declarations.h"
#include <iit/rbd/traits/DoubleTrait.h>
#include "kinematics_parameters.h"
#include "model_constants.h"

namespace iit {
namespace Ballbot {

template<typename SCALAR, class M>
class TransformMotion : public iit::rbd::SpatialTransformBase<tpl::JointState<SCALAR>, M> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

template<typename SCALAR, class M>
class TransformForce : public iit::rbd::SpatialTransformBase<tpl::JointState<SCALAR>, M> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

template<typename SCALAR, class M>
class TransformHomogeneous : public iit::rbd::HomogeneousTransformBase<tpl::JointState<SCALAR>, M> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

namespace tpl {


/**
 * The class for the 6-by-6 coordinates transformation matrices for
 * spatial motion vectors.
 */
template <typename TRAIT>
class MotionTransforms {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef typename TRAIT::Scalar Scalar;

    typedef JointState<Scalar> JState;
    class Dummy {};
    typedef typename TransformMotion<Scalar, Dummy>::MatrixType MatrixType;
public:
    class Type_fr_world_X_fr_gripper : public TransformMotion<Scalar, Type_fr_world_X_fr_gripper>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_gripper();
        const Type_fr_world_X_fr_gripper& update(const JState&);
    protected:
    };
    
    class Type_fr_gripper_X_fr_world : public TransformMotion<Scalar, Type_fr_gripper_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_gripper_X_fr_world();
        const Type_fr_gripper_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_link_1 : public TransformMotion<Scalar, Type_fr_world_X_fr_link_1>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_link_1();
        const Type_fr_world_X_fr_link_1& update(const JState&);
    protected:
    };
    
    class Type_fr_link_1_X_fr_world : public TransformMotion<Scalar, Type_fr_link_1_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link_1_X_fr_world();
        const Type_fr_link_1_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_link_2 : public TransformMotion<Scalar, Type_fr_world_X_fr_link_2>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_link_2();
        const Type_fr_world_X_fr_link_2& update(const JState&);
    protected:
    };
    
    class Type_fr_link_2_X_fr_world : public TransformMotion<Scalar, Type_fr_link_2_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link_2_X_fr_world();
        const Type_fr_link_2_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_base_link : public TransformMotion<Scalar, Type_fr_world_X_fr_base_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_base_link();
        const Type_fr_world_X_fr_base_link& update(const JState&);
    protected:
    };
    
    class Type_fr_base_link_X_fr_world : public TransformMotion<Scalar, Type_fr_base_link_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_link_X_fr_world();
        const Type_fr_base_link_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_end : public TransformMotion<Scalar, Type_fr_world_X_fr_end>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_end();
        const Type_fr_world_X_fr_end& update(const JState&);
    protected:
    };
    
    class Type_fr_end_X_fr_world : public TransformMotion<Scalar, Type_fr_end_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_end_X_fr_world();
        const Type_fr_end_X_fr_world& update(const JState&);
    protected:
    };

    class Type_fr_world_X_fr_gripper_COM : public TransformMotion<Scalar, Type_fr_world_X_fr_gripper_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_gripper_COM();
        const Type_fr_world_X_fr_gripper_COM& update(const JState&);
    protected:
    };

    class Type_fr_gripper_COM_X_fr_world : public TransformMotion<Scalar, Type_fr_gripper_COM_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_gripper_COM_X_fr_world();
        const Type_fr_gripper_COM_X_fr_world& update(const JState&);
    protected:
    };

    class Type_fr_world_X_fr_link_1_COM : public TransformMotion<Scalar, Type_fr_world_X_fr_link_1_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_link_1_COM();
        const Type_fr_world_X_fr_link_1_COM& update(const JState&);
    protected:
    };

    class Type_fr_link_1_COM_X_fr_world : public TransformMotion<Scalar, Type_fr_link_1_COM_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link_1_COM_X_fr_world();
        const Type_fr_link_1_COM_X_fr_world& update(const JState&);
    protected:
    };

    class Type_fr_world_X_fr_link_2_COM : public TransformMotion<Scalar, Type_fr_world_X_fr_link_2_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_link_2_COM();
        const Type_fr_world_X_fr_link_2_COM& update(const JState&);
    protected:
    };

    class Type_fr_link_2_COM_X_fr_world : public TransformMotion<Scalar, Type_fr_link_2_COM_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link_2_COM_X_fr_world();
        const Type_fr_link_2_COM_X_fr_world& update(const JState&);
    protected:
    };

    class Type_fr_world_X_fr_world_COM : public TransformMotion<Scalar, Type_fr_world_X_fr_world_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_world_COM();
        const Type_fr_world_X_fr_world_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_world_COM_X_fr_world : public TransformMotion<Scalar, Type_fr_world_COM_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_COM_X_fr_world();
        const Type_fr_world_COM_X_fr_world& update(const JState&);
    protected:
    };

    class Type_fr_world_X_fr_joint_1 : public TransformMotion<Scalar, Type_fr_world_X_fr_joint_1>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_joint_1();
        const Type_fr_world_X_fr_joint_1& update(const JState&);
    protected:
    };

    class Type_fr_world_X_fr_joint_2 : public TransformMotion<Scalar, Type_fr_world_X_fr_joint_2>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_joint_2();
        const Type_fr_world_X_fr_joint_2& update(const JState&);
    protected:
    };

    class Type_fr_world_X_fr_joint_3 : public TransformMotion<Scalar, Type_fr_world_X_fr_joint_3>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_joint_3();
        const Type_fr_world_X_fr_joint_3& update(const JState&);
    protected:
    };

    class Type_fr_link_2_X_fr_link_1 : public TransformMotion<Scalar, Type_fr_link_2_X_fr_link_1>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link_2_X_fr_link_1();
        const Type_fr_link_2_X_fr_link_1& update(const JState&);
    protected:
    };

    class Type_fr_link_1_X_fr_link_2 : public TransformMotion<Scalar, Type_fr_link_1_X_fr_link_2>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link_1_X_fr_link_2();
        const Type_fr_link_1_X_fr_link_2& update(const JState&);
    protected:
    };

    class Type_fr_gripper_X_fr_link_2 : public TransformMotion<Scalar, Type_fr_gripper_X_fr_link_2>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_gripper_X_fr_link_2();
        const Type_fr_gripper_X_fr_link_2& update(const JState&);
    protected:
    };

    class Type_fr_link_2_X_fr_gripper : public TransformMotion<Scalar, Type_fr_link_2_X_fr_gripper>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link_2_X_fr_gripper();
        const Type_fr_link_2_X_fr_gripper& update(const JState&);
    protected:
    };

public:
    MotionTransforms();
    void updateParameters();
    Type_fr_world_X_fr_gripper fr_world_X_fr_gripper;
    Type_fr_gripper_X_fr_world fr_gripper_X_fr_world;
    Type_fr_world_X_fr_link_1 fr_world_X_fr_link_1;
    Type_fr_link_1_X_fr_world fr_link_1_X_fr_world;
    Type_fr_world_X_fr_link_2 fr_world_X_fr_link_2;
    Type_fr_link_2_X_fr_world fr_link_2_X_fr_world;
    Type_fr_world_X_fr_base_link fr_world_X_fr_base_link;
    Type_fr_base_link_X_fr_world fr_base_link_X_fr_world;
    Type_fr_world_X_fr_end fr_world_X_fr_end;
    Type_fr_end_X_fr_world fr_end_X_fr_world;
    Type_fr_world_X_fr_gripper_COM fr_world_X_fr_gripper_COM;
    Type_fr_gripper_COM_X_fr_world fr_gripper_COM_X_fr_world;
    Type_fr_world_X_fr_link_1_COM fr_world_X_fr_link_1_COM;
    Type_fr_link_1_COM_X_fr_world fr_link_1_COM_X_fr_world;
    Type_fr_world_X_fr_link_2_COM fr_world_X_fr_link_2_COM;
    Type_fr_link_2_COM_X_fr_world fr_link_2_COM_X_fr_world;
    Type_fr_world_X_fr_world_COM fr_world_X_fr_world_COM;
    Type_fr_world_COM_X_fr_world fr_world_COM_X_fr_world;
    Type_fr_world_X_fr_joint_1 fr_world_X_fr_joint_1;
    Type_fr_world_X_fr_joint_2 fr_world_X_fr_joint_2;
    Type_fr_world_X_fr_joint_3 fr_world_X_fr_joint_3;
    Type_fr_link_2_X_fr_link_1 fr_link_2_X_fr_link_1;
    Type_fr_link_1_X_fr_link_2 fr_link_1_X_fr_link_2;
    Type_fr_gripper_X_fr_link_2 fr_gripper_X_fr_link_2;
    Type_fr_link_2_X_fr_gripper fr_link_2_X_fr_gripper;

protected:

}; //class 'MotionTransforms'

/**
 * The class for the 6-by-6 coordinates transformation matrices for
 * spatial force vectors.
 */
template <typename TRAIT>
class ForceTransforms {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef typename TRAIT::Scalar Scalar;

    typedef JointState<Scalar> JState;
    class Dummy {};
    typedef typename TransformForce<Scalar, Dummy>::MatrixType MatrixType;
public:
    class Type_fr_world_X_fr_gripper : public TransformForce<Scalar, Type_fr_world_X_fr_gripper>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_gripper();
        const Type_fr_world_X_fr_gripper& update(const JState&);
    protected:
    };
    
    class Type_fr_gripper_X_fr_world : public TransformForce<Scalar, Type_fr_gripper_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_gripper_X_fr_world();
        const Type_fr_gripper_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_link_1 : public TransformForce<Scalar, Type_fr_world_X_fr_link_1>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_link_1();
        const Type_fr_world_X_fr_link_1& update(const JState&);
    protected:
    };
    
    class Type_fr_link_1_X_fr_world : public TransformForce<Scalar, Type_fr_link_1_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link_1_X_fr_world();
        const Type_fr_link_1_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_link_2 : public TransformForce<Scalar, Type_fr_world_X_fr_link_2>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_link_2();
        const Type_fr_world_X_fr_link_2& update(const JState&);
    protected:
    };
    
    class Type_fr_link_2_X_fr_world : public TransformForce<Scalar, Type_fr_link_2_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link_2_X_fr_world();
        const Type_fr_link_2_X_fr_world& update(const JState&);
    protected:
    };

    class Type_fr_world_X_fr_base_link : public TransformForce<Scalar, Type_fr_world_X_fr_base_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_base_link();
        const Type_fr_world_X_fr_base_link& update(const JState&);
    protected:
    };
    
    class Type_fr_base_link_X_fr_world : public TransformForce<Scalar, Type_fr_base_link_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_link_X_fr_world();
        const Type_fr_base_link_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_end : public TransformForce<Scalar, Type_fr_world_X_fr_end>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_end();
        const Type_fr_world_X_fr_end& update(const JState&);
    protected:
    };
    
    class Type_fr_end_X_fr_world : public TransformForce<Scalar, Type_fr_end_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_end_X_fr_world();
        const Type_fr_end_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_gripper_COM : public TransformForce<Scalar, Type_fr_world_X_fr_gripper_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_gripper_COM();
        const Type_fr_world_X_fr_gripper_COM& update(const JState&);
    protected:
    };

    class Type_fr_gripper_COM_X_fr_world : public TransformForce<Scalar, Type_fr_gripper_COM_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_gripper_COM_X_fr_world();
        const Type_fr_gripper_COM_X_fr_world& update(const JState&);
    protected:
    };

    class Type_fr_world_X_fr_link_1_COM : public TransformForce<Scalar, Type_fr_world_X_fr_link_1_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_link_1_COM();
        const Type_fr_world_X_fr_link_1_COM& update(const JState&);
    protected:
    };

    class Type_fr_link_1_COM_X_fr_world : public TransformForce<Scalar, Type_fr_link_1_COM_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link_1_COM_X_fr_world();
        const Type_fr_link_1_COM_X_fr_world& update(const JState&);
    protected:
    };

    class Type_fr_world_X_fr_link_2_COM : public TransformForce<Scalar, Type_fr_world_X_fr_link_2_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_link_2_COM();
        const Type_fr_world_X_fr_link_2_COM& update(const JState&);
    protected:
    };

    class Type_fr_link_2_COM_X_fr_world : public TransformForce<Scalar, Type_fr_link_2_COM_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link_2_COM_X_fr_world();
        const Type_fr_link_2_COM_X_fr_world& update(const JState&);
    protected:
    };

    class Type_fr_world_X_fr_world_COM : public TransformForce<Scalar, Type_fr_world_X_fr_world_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_world_COM();
        const Type_fr_world_X_fr_world_COM& update(const JState&);
    protected:
    };

    class Type_fr_world_COM_X_fr_world : public TransformForce<Scalar, Type_fr_world_COM_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_COM_X_fr_world();
        const Type_fr_world_COM_X_fr_world& update(const JState&);
    protected:
    };

    class Type_fr_world_X_fr_joint_1 : public TransformForce<Scalar, Type_fr_world_X_fr_joint_1>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_joint_1();
        const Type_fr_world_X_fr_joint_1& update(const JState&);
    protected:
    };

    class Type_fr_world_X_fr_joint_2 : public TransformForce<Scalar, Type_fr_world_X_fr_joint_2>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_joint_2();
        const Type_fr_world_X_fr_joint_2& update(const JState&);
    protected:
    };

    class Type_fr_world_X_fr_joint_3 : public TransformForce<Scalar, Type_fr_world_X_fr_joint_3>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_joint_3();
        const Type_fr_world_X_fr_joint_3& update(const JState&);
    protected:
    };

    class Type_fr_link_2_X_fr_link_1 : public TransformForce<Scalar, Type_fr_link_2_X_fr_link_1>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link_2_X_fr_link_1();
        const Type_fr_link_2_X_fr_link_1& update(const JState&);
    protected:
    };

    class Type_fr_link_1_X_fr_link_2 : public TransformForce<Scalar, Type_fr_link_1_X_fr_link_2>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link_1_X_fr_link_2();
        const Type_fr_link_1_X_fr_link_2& update(const JState&);
    protected:
    };

    class Type_fr_gripper_X_fr_link_2 : public TransformForce<Scalar, Type_fr_gripper_X_fr_link_2>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_gripper_X_fr_link_2();
        const Type_fr_gripper_X_fr_link_2& update(const JState&);
    protected:
    };

    class Type_fr_link_2_X_fr_gripper : public TransformForce<Scalar, Type_fr_link_2_X_fr_gripper>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link_2_X_fr_gripper();
        const Type_fr_link_2_X_fr_gripper& update(const JState&);
    protected:
    };
    
public:
    ForceTransforms();
    void updateParameters();
    Type_fr_world_X_fr_gripper fr_world_X_fr_gripper;
    Type_fr_gripper_X_fr_world fr_gripper_X_fr_world;
    Type_fr_world_X_fr_link_1 fr_world_X_fr_link_1;
    Type_fr_link_1_X_fr_world fr_link_1_X_fr_world;
    Type_fr_world_X_fr_link_2 fr_world_X_fr_link_2;
    Type_fr_link_2_X_fr_world fr_link_2_X_fr_world;
    Type_fr_world_X_fr_base_link fr_world_X_fr_base_link;
    Type_fr_base_link_X_fr_world fr_base_link_X_fr_world;
    Type_fr_world_X_fr_end fr_world_X_fr_end;
    Type_fr_end_X_fr_world fr_end_X_fr_world;
    Type_fr_world_X_fr_gripper_COM fr_world_X_fr_gripper_COM;
    Type_fr_gripper_COM_X_fr_world fr_gripper_COM_X_fr_world;
    Type_fr_world_X_fr_link_1_COM fr_world_X_fr_link_1_COM;
    Type_fr_link_1_COM_X_fr_world fr_link_1_COM_X_fr_world;
    Type_fr_world_X_fr_link_2_COM fr_world_X_fr_link_2_COM;
    Type_fr_link_2_COM_X_fr_world fr_link_2_COM_X_fr_world;
    Type_fr_world_X_fr_world_COM fr_world_X_fr_world_COM;
    Type_fr_world_COM_X_fr_world fr_world_COM_X_fr_world;
    Type_fr_world_X_fr_joint_1 fr_world_X_fr_joint_1;
    Type_fr_world_X_fr_joint_2 fr_world_X_fr_joint_2;
    Type_fr_world_X_fr_joint_3 fr_world_X_fr_joint_3;
    Type_fr_link_2_X_fr_link_1 fr_link_2_X_fr_link_1;
    Type_fr_link_1_X_fr_link_2 fr_link_1_X_fr_link_2;
    Type_fr_gripper_X_fr_link_2 fr_gripper_X_fr_link_2;
    Type_fr_link_2_X_fr_gripper fr_link_2_X_fr_gripper;

protected:

}; //class 'ForceTransforms'

/**
 * The class with the homogeneous (4x4) coordinates transformation
 * matrices.
 */
template <typename TRAIT>
class HomogeneousTransforms {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef typename TRAIT::Scalar Scalar;

    typedef JointState<Scalar> JState;
    class Dummy {};
    typedef typename TransformHomogeneous<Scalar, Dummy>::MatrixType MatrixType;
public:
    class Type_fr_world_X_fr_gripper : public TransformHomogeneous<Scalar, Type_fr_world_X_fr_gripper>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_gripper();
        const Type_fr_world_X_fr_gripper& update(const JState&);
    protected:
    };
    
    class Type_fr_gripper_X_fr_world : public TransformHomogeneous<Scalar, Type_fr_gripper_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_gripper_X_fr_world();
        const Type_fr_gripper_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_link_1 : public TransformHomogeneous<Scalar, Type_fr_world_X_fr_link_1>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_link_1();
        const Type_fr_world_X_fr_link_1& update(const JState&);
    protected:
    };
    
    class Type_fr_link_1_X_fr_world : public TransformHomogeneous<Scalar, Type_fr_link_1_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link_1_X_fr_world();
        const Type_fr_link_1_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_link_2 : public TransformHomogeneous<Scalar, Type_fr_world_X_fr_link_2>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_link_2();
        const Type_fr_world_X_fr_link_2& update(const JState&);
    protected:
    };
    
    class Type_fr_link_2_X_fr_world : public TransformHomogeneous<Scalar, Type_fr_link_2_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link_2_X_fr_world();
        const Type_fr_link_2_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_base_link : public TransformHomogeneous<Scalar, Type_fr_world_X_fr_base_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_base_link();
        const Type_fr_world_X_fr_base_link& update(const JState&);
    protected:
    };
    
    class Type_fr_base_link_X_fr_world : public TransformHomogeneous<Scalar, Type_fr_base_link_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_link_X_fr_world();
        const Type_fr_base_link_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_end : public TransformHomogeneous<Scalar, Type_fr_world_X_fr_end>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_end();
        const Type_fr_world_X_fr_end& update(const JState&);
    protected:
    };
    
    class Type_fr_end_X_fr_world : public TransformHomogeneous<Scalar, Type_fr_end_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_end_X_fr_world();
        const Type_fr_end_X_fr_world& update(const JState&);
    protected:
    };

    class Type_fr_world_X_fr_gripper_COM : public TransformHomogeneous<Scalar, Type_fr_world_X_fr_gripper_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_gripper_COM();
        const Type_fr_world_X_fr_gripper_COM& update(const JState&);
    protected:
    };

    class Type_fr_gripper_COM_X_fr_world : public TransformHomogeneous<Scalar, Type_fr_gripper_COM_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_gripper_COM_X_fr_world();
        const Type_fr_gripper_COM_X_fr_world& update(const JState&);
    protected:
    };

    class Type_fr_world_X_fr_link_1_COM : public TransformHomogeneous<Scalar, Type_fr_world_X_fr_link_1_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_link_1_COM();
        const Type_fr_world_X_fr_link_1_COM& update(const JState&);
    protected:
    };

    class Type_fr_link_1_COM_X_fr_world : public TransformHomogeneous<Scalar, Type_fr_link_1_COM_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link_1_COM_X_fr_world();
        const Type_fr_link_1_COM_X_fr_world& update(const JState&);
    protected:
    };

    class Type_fr_world_X_fr_link_2_COM : public TransformHomogeneous<Scalar, Type_fr_world_X_fr_link_2_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_link_2_COM();
        const Type_fr_world_X_fr_link_2_COM& update(const JState&);
    protected:
    };

    class Type_fr_link_2_COM_X_fr_world : public TransformHomogeneous<Scalar, Type_fr_link_2_COM_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link_2_COM_X_fr_world();
        const Type_fr_link_2_COM_X_fr_world& update(const JState&);
    protected:
    };

    class Type_fr_world_X_fr_world_COM : public TransformHomogeneous<Scalar, Type_fr_world_X_fr_world_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_world_COM();
        const Type_fr_world_X_fr_world_COM& update(const JState&);
    protected:
    };

    class Type_fr_world_COM_X_fr_world : public TransformHomogeneous<Scalar, Type_fr_world_COM_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_COM_X_fr_world();
        const Type_fr_world_COM_X_fr_world& update(const JState&);
    protected:
    };

    class Type_fr_world_X_fr_joint_1 : public TransformHomogeneous<Scalar, Type_fr_world_X_fr_joint_1>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_joint_1();
        const Type_fr_world_X_fr_joint_1& update(const JState&);
    protected:
    };

    class Type_fr_world_X_fr_joint_2 : public TransformHomogeneous<Scalar, Type_fr_world_X_fr_joint_2>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_joint_2();
        const Type_fr_world_X_fr_joint_2& update(const JState&);
    protected:
    };

    class Type_fr_world_X_fr_joint_3 : public TransformHomogeneous<Scalar, Type_fr_world_X_fr_joint_3>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_joint_3();
        const Type_fr_world_X_fr_joint_3& update(const JState&);
    protected:
    };

    class Type_fr_link_2_X_fr_link_1 : public TransformHomogeneous<Scalar, Type_fr_link_2_X_fr_link_1>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link_2_X_fr_link_1();
        const Type_fr_link_2_X_fr_link_1& update(const JState&);
    protected:
    };

    class Type_fr_link_1_X_fr_link_2 : public TransformHomogeneous<Scalar, Type_fr_link_1_X_fr_link_2>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link_1_X_fr_link_2();
        const Type_fr_link_1_X_fr_link_2& update(const JState&);
    protected:
    };

    class Type_fr_gripper_X_fr_link_2 : public TransformHomogeneous<Scalar, Type_fr_gripper_X_fr_link_2>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_gripper_X_fr_link_2();
        const Type_fr_gripper_X_fr_link_2& update(const JState&);
    protected:
    };

    class Type_fr_link_2_X_fr_gripper : public TransformHomogeneous<Scalar, Type_fr_link_2_X_fr_gripper>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link_2_X_fr_gripper();
        const Type_fr_link_2_X_fr_gripper& update(const JState&);
    protected:
    };


public:
    HomogeneousTransforms();
    void updateParameters();
    Type_fr_world_X_fr_gripper fr_world_X_fr_gripper;
    Type_fr_gripper_X_fr_world fr_gripper_X_fr_world;
    Type_fr_world_X_fr_link_1 fr_world_X_fr_link_1;
    Type_fr_link_1_X_fr_world fr_link_1_X_fr_world;
    Type_fr_world_X_fr_link_2 fr_world_X_fr_link_2;
    Type_fr_link_2_X_fr_world fr_link_2_X_fr_world;
    Type_fr_world_X_fr_base_link fr_world_X_fr_base_link;
    Type_fr_base_link_X_fr_world fr_base_link_X_fr_world;
    Type_fr_world_X_fr_end fr_world_X_fr_end;
    Type_fr_end_X_fr_world fr_end_X_fr_world;
    Type_fr_world_X_fr_gripper_COM fr_world_X_fr_gripper_COM;
    Type_fr_gripper_COM_X_fr_world fr_gripper_COM_X_fr_world;
    Type_fr_world_X_fr_link_1_COM fr_world_X_fr_link_1_COM;
    Type_fr_link_1_COM_X_fr_world fr_link_1_COM_X_fr_world;
    Type_fr_world_X_fr_link_2_COM fr_world_X_fr_link_2_COM;
    Type_fr_link_2_COM_X_fr_world fr_link_2_COM_X_fr_world;
    Type_fr_world_X_fr_world_COM fr_world_X_fr_world_COM;
    Type_fr_world_COM_X_fr_world fr_world_COM_X_fr_world;
    Type_fr_world_X_fr_joint_1 fr_world_X_fr_joint_1;
    Type_fr_world_X_fr_joint_2 fr_world_X_fr_joint_2;
    Type_fr_world_X_fr_joint_3 fr_world_X_fr_joint_3;
    Type_fr_link_2_X_fr_link_1 fr_link_2_X_fr_link_1;
    Type_fr_link_1_X_fr_link_2 fr_link_1_X_fr_link_2;
    Type_fr_gripper_X_fr_link_2 fr_gripper_X_fr_link_2;
    Type_fr_link_2_X_fr_gripper fr_link_2_X_fr_gripper;

protected:

}; //class 'HomogeneousTransforms'

}

using MotionTransforms = tpl::MotionTransforms<rbd::DoubleTrait>;
using ForceTransforms = tpl::ForceTransforms<rbd::DoubleTrait>;
using HomogeneousTransforms = tpl::HomogeneousTransforms<rbd::DoubleTrait>;

}
}

#include "transforms.impl.h"

#endif
