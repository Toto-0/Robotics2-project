#ifndef IIT_ROBOT_BALLBOT_DECLARATIONS_H_
#define IIT_ROBOT_BALLBOT_DECLARATIONS_H_

#include <iit/rbd/rbd.h>

namespace iit {
namespace Ballbot {

static const int JointSpaceDimension = 3;
static const int jointsCount = 3;
/** The total number of rigid bodies of this robot, including the base */
static const int linksCount  = 4;

namespace tpl {
template <typename SCALAR>
using Column3d = iit::rbd::PlainMatrix<SCALAR, 3, 1>;

template <typename SCALAR>
using JointState = Column3d<SCALAR>;
}

using Column3d = tpl::Column3d<double>;
typedef Column3d JointState;

enum JointIdentifiers {
    JOINT_1 = 0
    , JOINT_2
    , JOINT_3
};

enum LinkIdentifiers {
    WORLD = 0
    , LINK_1
    , LINK_2
    , GRIPPER
};

static const JointIdentifiers orderedJointIDs[jointsCount] =
    {JOINT_1,JOINT_2,JOINT_3};

static const LinkIdentifiers orderedLinkIDs[linksCount] =
    {WORLD,LINK_1,LINK_2,GRIPPER};

}
}
#endif
