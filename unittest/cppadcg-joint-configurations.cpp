//
// Copyright (c) 2020 CNRS INRIA
//

#include "pinocchio/codegen/cppadcg.hpp"
#include "pinocchio/codegen/code-generator-algo.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"

#include "utils/model-generator.hpp"

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

using namespace pinocchio;

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(test_joint_configuration_code_generation)
{
  typedef double Scalar;

  typedef pinocchio::ModelTpl<Scalar> Model;   

  Model model; buildAllJointsModel(model);
  Eigen::VectorXd qs = Eigen::VectorXd::Random(model.nq);
  Eigen::VectorXd qs2 = Eigen::VectorXd::Random(model.nq);
  normalize(model,qs);
  normalize(model,qs2);
  
  Eigen::VectorXd vs = Eigen::VectorXd::Random(model.nv);
  std::vector<Eigen::VectorXd> results_q(2,Eigen::VectorXd::Zero(model.nq));
  std::vector<Eigen::VectorXd> results_v(2,Eigen::VectorXd::Zero(model.nv));

  //Integrate
  CodeGenIntegrate<double> cg_integrate(model);
  cg_integrate.initLib();
  cg_integrate.loadLib();
  
  cg_integrate.evalFunction(qs,vs, results_q[0]);
  pinocchio::integrate(model, qs,vs,results_q[1]);
  BOOST_CHECK(results_q[1].isApprox(results_q[0]));

  cg_integrate.evalFunction(qs,0*vs,results_q[0]);
  BOOST_CHECK(results_q[0].isApprox(qs));
  
  //Difference
  CodeGenDifference<double> cg_difference(model);
  cg_difference.initLib();
  cg_difference.loadLib();
  
  cg_difference.evalFunction(qs,qs2, results_v[0]);
  pinocchio::difference(model,qs,qs2,results_v[1]);
  BOOST_CHECK(results_v[1].isApprox(results_v[0]));

  cg_difference.evalFunction(qs,qs,results_v[0]);
  BOOST_CHECK(results_v[0].isZero());

  //dDifference
  CodeGenDDifference<double> cg_dDifference(model);
  cg_dDifference.initLib();
  cg_dDifference.loadLib();
  
  //ARG0
  std::vector<Eigen::MatrixXd> results_J(2,Eigen::MatrixXd::Zero(model.nv,model.nv));
  cg_dDifference.evalFunction(qs,qs2, results_J[0], pinocchio::ARG0);
  pinocchio::dDifference(model,qs,qs2,results_J[1], pinocchio::ARG0);
  BOOST_CHECK(results_J[1].isApprox(results_J[0]));

  //ARG1
  cg_dDifference.evalFunction(qs,qs2, results_J[0], pinocchio::ARG1);
  pinocchio::dDifference(model,qs,qs2,results_J[1], pinocchio::ARG1);
  BOOST_CHECK(results_J[1].isApprox(results_J[0]));
  
}

BOOST_AUTO_TEST_SUITE_END()
