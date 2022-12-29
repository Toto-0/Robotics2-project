#ifndef RCG_R3_MODEL_CONSTANTS_H_
#define RCG_R3_MODEL_CONSTANTS_H_


/**
 * \file
 * This file contains the definitions of all the non-zero numerical
 * constants of the robot model (i.e. the numbers appearing in the
 * .kindsl file).
 *
 * Varying these values (and recompiling) is a quick & dirty
 * way to vary the kinematics/dynamics model. For a much more
 * flexible way of exploring variations of the model, consider
 * using the parametrization feature of RobCoGen (see the wiki).
 *
 * Beware of inconsistencies when changing any of the inertia
 * properties.
 */

namespace iit {
namespace Ballbot {

// Do not use 'constexpr' to allow for non-literal scalar types

const double tz_joint_1 = 0.05000000074505806;
const double tx_joint_2 = 0.5;
const double tx_joint_3 = 0.5;
const double tx_fr_link_1_COM = 0.25;
const double tx_fr_link_2_COM = 0.25;
const double tx_fr_end = 0.5;
const double tx_fr_gripper_COM = 0.3235294222831726;
const double m_link_1 = 3.0;
const double comx_link_1 = 0.25;
const double ix_link_1 = 0.0037499999161809683;
const double iy_link_1 = 0.1912499964237213;
const double iz_link_1 = 0.4375;
const double m_link_2 = 3.0;
const double comx_link_2 = 0.25;
const double ix_link_2 = 0.0037499999161809683;
const double iy_link_2 = 0.1912499964237213;
const double iz_link_2 = 0.4375;
const double m_gripper = 4.25;
const double comx_gripper = 0.3235294222831726;
const double ix_gripper = 0.0038499999791383743;
const double iy_gripper = 0.503849983215332;
const double iz_gripper = 0.7501000165939331;

}
}
#endif
