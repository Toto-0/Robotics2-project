/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <iostream>
#include <string>

#include "ocs2_3r_nom/BallbotInterface.h"

#include <ocs2_core/cost/QuadraticStateCost.h>
#include <ocs2_core/cost/QuadraticStateInputCost.h>
#include <ocs2_core/augmented_lagrangian/StateInputAugmentedLagrangian.h>
#include <ocs2_core/penalties/augmented/ModifiedRelaxedBarrierPenalty.h>
#include <ocs2_core/constraint/LinearStateInputConstraint.h>
#include <ocs2_core/soft_constraint/StateInputSoftBoxConstraint.h>
#include <ocs2_core/penalties/penalties/RelaxedBarrierPenalty.h>
#include <ocs2_core/initialization/DefaultInitializer.h>
#include <ocs2_core/misc/LoadData.h>

// Boost
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

namespace ocs2 {
namespace ballbot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
BallbotInterface::BallbotInterface(const std::string& taskFile, const std::string& libraryFolder) {
  // check that task file exists
  boost::filesystem::path taskFilePath(taskFile);
  if (boost::filesystem::exists(taskFilePath)) {
    std::cerr << "[BallbotInterface] Loading task file: " << taskFilePath << std::endl;
  } else {
    throw std::invalid_argument("[BallbotInterface] Task file not found: " + taskFilePath.string());
  }
  // create library folder if it does not exist
  boost::filesystem::path libraryFolderPath(libraryFolder);
  boost::filesystem::create_directories(libraryFolderPath);
  std::cerr << "[BallbotInterface] Generated library path: " << libraryFolderPath << std::endl;

  // Default initial condition
  loadData::loadEigenMatrix(taskFile, "initialState", initialState_);
  std::cerr << "x_init:   " << initialState_.transpose() << std::endl;

  // DDP SQP MPC settings
  ddpSettings_ = ddp::loadSettings(taskFile, "ddp");
  mpcSettings_ = mpc::loadSettings(taskFile, "mpc");
  sqpSettings_ = multiple_shooting::loadSettings(taskFile, "multiple_shooting");

  /*
   * ReferenceManager & SolverSynchronizedModule
   */
  referenceManagerPtr_.reset(new ReferenceManager);

  /*
   * Optimal control problem
   */
  // Cost
  matrix_t Q(STATE_DIM, STATE_DIM);
  matrix_t R(INPUT_DIM, INPUT_DIM);
  matrix_t Qf(STATE_DIM, STATE_DIM);
  matrix_t F(INPUT_DIM, STATE_DIM);
  matrix_t D(INPUT_DIM, INPUT_DIM);
  loadData::loadEigenMatrix(taskFile, "Q", Q);
  loadData::loadEigenMatrix(taskFile, "R", R);
  loadData::loadEigenMatrix(taskFile, "Q_final", Qf);
  loadData::loadEigenMatrix(taskFile, "F", F);
  loadData::loadEigenMatrix(taskFile, "D", D);
  loadData::loadEigenMatrix(taskFile, "final_constraint", final_constraints);
  std::cerr << "Q:  \n" << Q << "\n";
  std::cerr << "R:  \n" << R << "\n";
  std::cerr << "Q_final:\n" << Qf << "\n";
  std::cerr << "F:  \n" << F << "\n";
  std::cerr << "D:  \n" << D << "\n";
  std::cerr << "final_constraint:  \n" << final_constraints << "\n";

  problem_.costPtr->add("cost", std::unique_ptr<StateInputCost>(new QuadraticStateInputCost(Q, R)));
  problem_.finalCostPtr->add("finalCost", std::unique_ptr<StateCost>(new QuadraticStateCost(Qf)));

  
  //std::unique_ptr<StateConstraint> scon = std::unique_ptr<StateConstraint>(new LinearStateConstraint(final_constraints, F));
  ocs2::RelaxedBarrierPenalty::Config config = ocs2::RelaxedBarrierPenalty::Config();
  ocs2::RelaxedBarrierPenalty pen = ocs2::RelaxedBarrierPenalty(config);
  //problem_.finalEqualityLagrangianPtr->add("finalInputConstr", std::unique_ptr<ocs2::StateAugmentedLagrangianInterface>(new StateAugmentedLagrangian(std::unique_ptr<StateConstraint>(new LinearStateConstraint(final_constraints, F)), std::unique_ptr<augmented::ModifiedRelaxedBarrierPenalty>(new augmented::ModifiedRelaxedBarrierPenalty(std::move(config))))));
  //problem_.inequalityLagrangianPtr->add("inputConstr", std::unique_ptr<ocs2::StateInputAugmentedLagrangianInterface>(new StateInputAugmentedLagrangian(std::unique_ptr<StateInputConstraint>(new LinearStateInputConstraint(final_constraints, F, D)), std::unique_ptr<augmented::ModifiedRelaxedBarrierPenalty>(new augmented::ModifiedRelaxedBarrierPenalty(std::move(config))))));
  //new LinearStateInputConstraint()

  //problem_.finalEqualityLagrangianPtr->add("finalInputConstr", std::unique_ptr<ocs2::StateAugmentedLagrangianInterface>(new StateAugmentedLagrangian(std::unique_ptr<StateConstraint>(new LinearStateConstraint(final_constraints, F)), std::unique_ptr<augmented::ModifiedRelaxedBarrierPenalty>(new augmented::ModifiedRelaxedBarrierPenalty(std::move(config))))));
  //problem_.inequalityLagrangianPtr->add("inputConstr2", std::unique_ptr<ocs2::StateInputAugmentedLagrangianInterface>(new StateInputAugmentedLagrangian(std::unique_ptr<StateInputConstraint>(new LinearStateInputConstraint(-final_constraints, -F, -D)), std::unique_ptr<augmented::ModifiedRelaxedBarrierPenalty>(new augmented::ModifiedRelaxedBarrierPenalty(std::move(config))))));


  //std::vector<ocs2::StateInputSoftBoxConstraint::BoxConstraint> v_bc_state;
  //std::vector<ocs2::StateInputSoftBoxConstraint::BoxConstraint> v_bc_input;
  //for (int i = 0; i<6; i++){
  //  ocs2::StateInputSoftBoxConstraint::BoxConstraint bc = ocs2::StateInputSoftBoxConstraint::BoxConstraint();
  //  bc.index = i;
  //  bc.lowerBound = -3.15;
  //  bc.upperBound = 3.15;
  //  bc.penaltyPtr = std::unique_ptr<ocs2::RelaxedBarrierPenalty>(new ocs2::RelaxedBarrierPenalty(config));
  //  v_bc_state.push_back(bc);
  //}
  //for (int i = 0; i<3; i++){
  //  ocs2::StateInputSoftBoxConstraint::BoxConstraint bc = ocs2::StateInputSoftBoxConstraint::BoxConstraint();
  //  bc.index = i;
  //  bc.lowerBound = -100;
  //  bc.upperBound = 100;
  //  bc.penaltyPtr = std::unique_ptr<ocs2::RelaxedBarrierPenalty>(new ocs2::RelaxedBarrierPenalty(config));
  //  v_bc_state.push_back(bc);
  //}
  //problem_.costPtr->add("cost_box", std::unique_ptr<StateInputCost>(new StateInputSoftBoxConstraint(v_bc_state, v_bc_input)));
  

  // Dynamics
  bool recompileLibraries;  // load the flag to generate library files from taskFile
  ocs2::loadData::loadCppDataType(taskFile, "ballbot_interface.recompileLibraries", recompileLibraries);
  problem_.dynamicsPtr.reset(new BallbotSystemDynamics(libraryFolder, recompileLibraries));

  // Rollout
  auto rolloutSettings = rollout::loadSettings(taskFile, "rollout");
  rolloutPtr_.reset(new TimeTriggeredRollout(*problem_.dynamicsPtr, rolloutSettings));

  // Initialization
  ballbotInitializerPtr_.reset(new DefaultInitializer(INPUT_DIM));
}

}  // namespace ballbot
}  // namespace ocs2
