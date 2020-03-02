//
// Copyright (c) 2015-2020 CNRS INRIA
// Copyright (c) 2015 Wandercraft, 86 rue de Paris 91400 Orsay, France.
//

#include <iostream>

#include "pinocchio/math/fwd.hpp"
#include "pinocchio/spatial/force.hpp"
#include "pinocchio/spatial/motion.hpp"
#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/spatial/inertia.hpp"
#include "pinocchio/multibody/joint/joints.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/compute-all-terms.hpp"

#include <boost/test/unit_test.hpp>

//#define VERBOSE

using namespace pinocchio;

template<typename JointData>
void printOutJointData(const Eigen::VectorXd & q,
                       const Eigen::VectorXd & v,
                       const JointDataBase<JointData> & joint_data)
{
  PINOCCHIO_UNUSED_VARIABLE(q);
  PINOCCHIO_UNUSED_VARIABLE(v);
  PINOCCHIO_UNUSED_VARIABLE(joint_data);
  
#ifdef VERBOSE
  using namespace std;
  
  cout << "q: " << q.transpose () << endl;
  cout << "v: " << v.transpose () << endl;
  cout << "Joint configuration:" << endl << joint_data.M() << endl;
  cout << "v_J:\n" << (Motion) joint_data.v() << endl;
  cout << "c_J:\n" << (Motion) joint_data.c() << endl;
#endif
}


template<typename D>
void addJointAndBody(Model & model,
                     const JointModelBase<D> & jmodel,
                     const Model::JointIndex parent_id,
                     const SE3 & joint_placement,
                     const std::string & joint_name,
                     const Inertia & Y)
{
  Model::JointIndex idx;
  
  idx = model.addJoint(parent_id,jmodel,joint_placement,joint_name);
  model.appendBodyToJoint(idx,Y);
}

BOOST_AUTO_TEST_SUITE(JointRevoluteUnaligned)

BOOST_AUTO_TEST_CASE(vsRX)
{
  using namespace pinocchio;
  typedef Eigen::Matrix <double, 3, 1> Vector3;
  typedef Eigen::Matrix <double, 3, 3> Matrix3;

  Eigen::Vector3d axis;
  axis << 1.0, 0.0, 0.0;

  Model modelRX, modelRevoluteUnaligned;

  Inertia inertia (1., Vector3 (0.5, 0., 0.0), Matrix3::Identity ());
  SE3 pos(1); pos.translation() = SE3::LinearType(1.,0.,0.);

  JointModelRevoluteUnaligned joint_model_RU(axis);
  
  addJointAndBody(modelRX,JointModelRX(),0,pos,"rx",inertia);
  addJointAndBody(modelRevoluteUnaligned,joint_model_RU,0,pos,"revolute-unaligned",inertia);

  Data dataRX(modelRX);
  Data dataRevoluteUnaligned(modelRevoluteUnaligned);

  Eigen::VectorXd q = Eigen::VectorXd::Ones (modelRX.nq);
  Eigen::VectorXd v = Eigen::VectorXd::Ones (modelRX.nv);
  Eigen::VectorXd tauRX = Eigen::VectorXd::Ones (modelRX.nv);
  Eigen::VectorXd tauRevoluteUnaligned = Eigen::VectorXd::Ones (modelRevoluteUnaligned.nv);
  Eigen::VectorXd aRX = Eigen::VectorXd::Ones (modelRX.nv);
  Eigen::VectorXd aRevoluteUnaligned(aRX);

  forwardKinematics(modelRX, dataRX, q, v);
  forwardKinematics(modelRevoluteUnaligned, dataRevoluteUnaligned, q, v);

  computeAllTerms(modelRX, dataRX, q, v);
  computeAllTerms(modelRevoluteUnaligned, dataRevoluteUnaligned, q, v);

  BOOST_CHECK(dataRevoluteUnaligned.oMi[1].isApprox(dataRX.oMi[1]));
  BOOST_CHECK(dataRevoluteUnaligned.liMi[1].isApprox(dataRX.liMi[1]));
  BOOST_CHECK(dataRevoluteUnaligned.Ycrb[1].matrix().isApprox(dataRX.Ycrb[1].matrix()));
  BOOST_CHECK(dataRevoluteUnaligned.f[1].toVector().isApprox(dataRX.f[1].toVector()));
  
  BOOST_CHECK(dataRevoluteUnaligned.nle.isApprox(dataRX.nle));
  BOOST_CHECK(dataRevoluteUnaligned.com[0].isApprox(dataRX.com[0]));

  // InverseDynamics == rnea
  tauRX = rnea(modelRX, dataRX, q, v, aRX);
  tauRevoluteUnaligned = rnea(modelRevoluteUnaligned, dataRevoluteUnaligned, q, v, aRevoluteUnaligned);

  BOOST_CHECK(tauRX.isApprox(tauRevoluteUnaligned));

  // ForwardDynamics == aba
  Eigen::VectorXd aAbaRX = aba(modelRX,dataRX, q, v, tauRX);
  Eigen::VectorXd aAbaRevoluteUnaligned = aba(modelRevoluteUnaligned,dataRevoluteUnaligned, q, v, tauRevoluteUnaligned);

  BOOST_CHECK(aAbaRX.isApprox(aAbaRevoluteUnaligned));

  // CRBA
  crba(modelRX, dataRX,q);
  crba(modelRevoluteUnaligned, dataRevoluteUnaligned, q);

  BOOST_CHECK(dataRX.M.isApprox(dataRevoluteUnaligned.M));
   
  // Jacobian
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobianRX;jacobianRX.resize(6,1); jacobianRX.setZero();
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobianRevoluteUnaligned;jacobianRevoluteUnaligned.resize(6,1);jacobianRevoluteUnaligned.setZero();
  computeJointJacobians(modelRX, dataRX, q);
  computeJointJacobians(modelRevoluteUnaligned, dataRevoluteUnaligned, q);
  getJointJacobian(modelRX, dataRX, 1, LOCAL, jacobianRX);
  getJointJacobian(modelRevoluteUnaligned, dataRevoluteUnaligned, 1, LOCAL, jacobianRevoluteUnaligned);


  BOOST_CHECK(jacobianRX.isApprox(jacobianRevoluteUnaligned));
}
BOOST_AUTO_TEST_SUITE_END ()
  
BOOST_AUTO_TEST_SUITE(JointRevoluteUnboundedUnaligned)
  
  BOOST_AUTO_TEST_CASE(vsRUX)
  {
    using namespace pinocchio;
    typedef SE3::Vector3 Vector3;
    typedef SE3::Matrix3 Matrix3;
    
    Vector3 axis;
    axis << 1.0, 0.0, 0.0;
    
    Model modelRUX, modelRevoluteUboundedUnaligned;
    
    Inertia inertia (1., Vector3 (0.5, 0., 0.0), Matrix3::Identity ());
    SE3 pos(1); pos.translation() = SE3::LinearType(1.,0.,0.);
    
    JointModelRevoluteUnboundedUnaligned joint_model_RUU(axis);
    typedef traits< JointRevoluteUnboundedUnalignedTpl<double> >::TangentVector_t TangentVector;
    
    addJointAndBody(modelRUX,JointModelRUBX(),0,pos,"rux",inertia);
    addJointAndBody(modelRevoluteUboundedUnaligned,joint_model_RUU,0,pos,"revolute-unbounded-unaligned",inertia);
    
    Data dataRUX(modelRUX);
    Data dataRevoluteUnboundedUnaligned(modelRevoluteUboundedUnaligned);
    
    Eigen::VectorXd q = Eigen::VectorXd::Ones(modelRUX.nq);
    q.head<2>().normalize();
    TangentVector v = TangentVector::Ones (modelRUX.nv);
    Eigen::VectorXd tauRX = Eigen::VectorXd::Ones (modelRUX.nv);
    Eigen::VectorXd tauRevoluteUnaligned = Eigen::VectorXd::Ones (modelRevoluteUboundedUnaligned.nv);
    Eigen::VectorXd aRX = Eigen::VectorXd::Ones (modelRUX.nv);
    Eigen::VectorXd aRevoluteUnaligned(aRX);
    
    forwardKinematics(modelRUX, dataRUX, q, v);
    forwardKinematics(modelRevoluteUboundedUnaligned, dataRevoluteUnboundedUnaligned, q, v);
    
    computeAllTerms(modelRUX, dataRUX, q, v);
    computeAllTerms(modelRevoluteUboundedUnaligned, dataRevoluteUnboundedUnaligned, q, v);
    
    BOOST_CHECK(dataRevoluteUnboundedUnaligned.oMi[1].isApprox(dataRUX.oMi[1]));
    BOOST_CHECK(dataRevoluteUnboundedUnaligned.liMi[1].isApprox(dataRUX.liMi[1]));
    BOOST_CHECK(dataRevoluteUnboundedUnaligned.Ycrb[1].matrix().isApprox(dataRUX.Ycrb[1].matrix()));
    BOOST_CHECK(dataRevoluteUnboundedUnaligned.f[1].toVector().isApprox(dataRUX.f[1].toVector()));
    
    BOOST_CHECK(dataRevoluteUnboundedUnaligned.nle.isApprox(dataRUX.nle));
    BOOST_CHECK(dataRevoluteUnboundedUnaligned.com[0].isApprox(dataRUX.com[0]));
    
    // InverseDynamics == rnea
    tauRX = rnea(modelRUX, dataRUX, q, v, aRX);
    tauRevoluteUnaligned = rnea(modelRevoluteUboundedUnaligned, dataRevoluteUnboundedUnaligned, q, v, aRevoluteUnaligned);
    
    BOOST_CHECK(tauRX.isApprox(tauRevoluteUnaligned));
    
    // ForwardDynamics == aba
    Eigen::VectorXd aAbaRX = aba(modelRUX, dataRUX, q, v, tauRX);
    Eigen::VectorXd aAbaRevoluteUnaligned = aba(modelRevoluteUboundedUnaligned,dataRevoluteUnboundedUnaligned, q, v, tauRevoluteUnaligned);
    
    BOOST_CHECK(aAbaRX.isApprox(aAbaRevoluteUnaligned));
    
    // CRBA
    crba(modelRUX, dataRUX,q);
    crba(modelRevoluteUboundedUnaligned, dataRevoluteUnboundedUnaligned, q);
    
    BOOST_CHECK(dataRUX.M.isApprox(dataRevoluteUnboundedUnaligned.M));
    
    // Jacobian
    Data::Matrix6x jacobianRUX;jacobianRUX.resize(6,1); jacobianRUX.setZero();
    Data::Matrix6x jacobianRevoluteUnboundedUnaligned;
    jacobianRevoluteUnboundedUnaligned.resize(6,1); jacobianRevoluteUnboundedUnaligned.setZero();
    
    computeJointJacobians(modelRUX, dataRUX, q);
    computeJointJacobians(modelRevoluteUboundedUnaligned, dataRevoluteUnboundedUnaligned, q);
    getJointJacobian(modelRUX, dataRUX, 1, LOCAL, jacobianRUX);
    getJointJacobian(modelRevoluteUboundedUnaligned, dataRevoluteUnboundedUnaligned, 1, LOCAL, jacobianRevoluteUnboundedUnaligned);
    
    BOOST_CHECK(jacobianRUX.isApprox(jacobianRevoluteUnboundedUnaligned));
  }
  BOOST_AUTO_TEST_SUITE_END ()

BOOST_AUTO_TEST_SUITE (JointPrismaticUnaligned)
  
  BOOST_AUTO_TEST_CASE(spatial)
  {
    SE3 M(SE3::Random());
    Motion v(Motion::Random());
    
    MotionPrismaticUnaligned mp(MotionPrismaticUnaligned::Vector3(1.,2.,3.),6.);
    Motion mp_dense(mp);
    
    BOOST_CHECK(M.act(mp).isApprox(M.act(mp_dense)));
    BOOST_CHECK(M.actInv(mp).isApprox(M.actInv(mp_dense)));
    
    BOOST_CHECK(v.cross(mp).isApprox(v.cross(mp_dense)));
  }

BOOST_AUTO_TEST_CASE (vsPX)
{
  using namespace pinocchio;
  typedef Eigen::Matrix <double, 3, 1> Vector3;
  typedef Eigen::Matrix <double, 3, 3> Matrix3;

  Eigen::Vector3d axis;
  axis << 1.0, 0.0, 0.0;

  Model modelPX, modelPrismaticUnaligned;

  Inertia inertia (1., Vector3 (0.5, 0., 0.0), Matrix3::Identity ());
  SE3 pos(1); pos.translation() = SE3::LinearType(1.,0.,0.);

  JointModelPrismaticUnaligned joint_model_PU(axis);
  
  addJointAndBody(modelPX,JointModelPX(),0,pos,"px",inertia);
  addJointAndBody(modelPrismaticUnaligned,joint_model_PU,0,pos,"prismatic-unaligned",inertia);

  Data dataPX(modelPX);
  Data dataPrismaticUnaligned(modelPrismaticUnaligned);

  Eigen::VectorXd q = Eigen::VectorXd::Ones (modelPX.nq);
  Eigen::VectorXd v = Eigen::VectorXd::Ones (modelPX.nv);
  Eigen::VectorXd tauPX = Eigen::VectorXd::Ones (modelPX.nv);
  Eigen::VectorXd tauPrismaticUnaligned = Eigen::VectorXd::Ones (modelPrismaticUnaligned.nv);
  Eigen::VectorXd aPX = Eigen::VectorXd::Ones (modelPX.nv);
  Eigen::VectorXd aPrismaticUnaligned(aPX);
  
  forwardKinematics(modelPX, dataPX, q, v);
  forwardKinematics(modelPrismaticUnaligned, dataPrismaticUnaligned, q, v);

  computeAllTerms(modelPX, dataPX, q, v);
  computeAllTerms(modelPrismaticUnaligned, dataPrismaticUnaligned, q, v);

  BOOST_CHECK(dataPrismaticUnaligned.oMi[1].isApprox(dataPX.oMi[1]));
  BOOST_CHECK(dataPrismaticUnaligned.liMi[1].isApprox(dataPX.liMi[1]));
  BOOST_CHECK(dataPrismaticUnaligned.Ycrb[1].matrix().isApprox(dataPX.Ycrb[1].matrix()));
  BOOST_CHECK(dataPrismaticUnaligned.f[1].toVector().isApprox(dataPX.f[1].toVector()));
  
  BOOST_CHECK(dataPrismaticUnaligned.nle.isApprox(dataPX.nle));
  BOOST_CHECK(dataPrismaticUnaligned.com[0].isApprox(dataPX.com[0]));

  // InverseDynamics == rnea
  tauPX = rnea(modelPX, dataPX, q, v, aPX);
  tauPrismaticUnaligned = rnea(modelPrismaticUnaligned, dataPrismaticUnaligned, q, v, aPrismaticUnaligned);

  BOOST_CHECK(tauPX.isApprox(tauPrismaticUnaligned));

  // ForwardDynamics == aba
  Eigen::VectorXd aAbaPX = aba(modelPX,dataPX, q, v, tauPX);
  Eigen::VectorXd aAbaPrismaticUnaligned = aba(modelPrismaticUnaligned,dataPrismaticUnaligned, q, v, tauPrismaticUnaligned);

  BOOST_CHECK(aAbaPX.isApprox(aAbaPrismaticUnaligned));

  // crba
  crba(modelPX, dataPX,q);
  crba(modelPrismaticUnaligned, dataPrismaticUnaligned, q);

  BOOST_CHECK(dataPX.M.isApprox(dataPrismaticUnaligned.M));
   
  // Jacobian
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobianPX;jacobianPX.resize(6,1); jacobianPX.setZero();
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobianPrismaticUnaligned;jacobianPrismaticUnaligned.resize(6,1);jacobianPrismaticUnaligned.setZero();
  computeJointJacobians(modelPX, dataPX, q);
  computeJointJacobians(modelPrismaticUnaligned, dataPrismaticUnaligned, q);
  getJointJacobian(modelPX, dataPX, 1, LOCAL, jacobianPX);
  getJointJacobian(modelPrismaticUnaligned, dataPrismaticUnaligned, 1, LOCAL, jacobianPrismaticUnaligned);

  BOOST_CHECK(jacobianPX.isApprox(jacobianPrismaticUnaligned));
}
BOOST_AUTO_TEST_SUITE_END ()

BOOST_AUTO_TEST_SUITE (JointSpherical)
  
  BOOST_AUTO_TEST_CASE(spatial)
  {
    SE3 M(SE3::Random());
    Motion v(Motion::Random());
    
    MotionSpherical mp(MotionSpherical::Vector3(1.,2.,3.));
    Motion mp_dense(mp);
    
    BOOST_CHECK(M.act(mp).isApprox(M.act(mp_dense)));
    BOOST_CHECK(M.actInv(mp).isApprox(M.actInv(mp_dense)));
    
    BOOST_CHECK(v.cross(mp).isApprox(v.cross(mp_dense)));
  }

BOOST_AUTO_TEST_CASE (vsFreeFlyer)
{
  using namespace pinocchio;
  typedef Eigen::Matrix <double, 3, 1> Vector3;
  typedef Eigen::Matrix <double, 6, 1> Vector6;
  typedef Eigen::Matrix <double, 7, 1> VectorFF;
  typedef Eigen::Matrix <double, 3, 3> Matrix3;

  Model modelSpherical, modelFreeflyer;

  Inertia inertia (1., Vector3 (0.5, 0., 0.0), Matrix3::Identity ());
  SE3 pos(1); pos.translation() = SE3::LinearType(1.,0.,0.);

  addJointAndBody(modelSpherical,JointModelSpherical(),0,pos,"spherical",inertia);
  addJointAndBody(modelFreeflyer,JointModelFreeFlyer(),0,pos,"free-flyer",inertia);
  
  Data dataSpherical(modelSpherical);
  Data dataFreeFlyer(modelFreeflyer);

  Eigen::VectorXd q = Eigen::VectorXd::Ones (modelSpherical.nq);q.normalize();
  VectorFF qff; qff << 0, 0, 0, q[0], q[1], q[2], q[3];
  Eigen::VectorXd v = Eigen::VectorXd::Ones (modelSpherical.nv);
  Vector6 vff; vff << 0, 0, 0, 1, 1, 1;
  Eigen::VectorXd tauSpherical = Eigen::VectorXd::Ones (modelSpherical.nv);
  Eigen::VectorXd tauff; tauff.resize(7); tauff << 0,0,0,1,1,1,1;
  Eigen::VectorXd aSpherical = Eigen::VectorXd::Ones (modelSpherical.nv);
  Eigen::VectorXd aff(vff);
  
  forwardKinematics(modelSpherical, dataSpherical, q, v);
  forwardKinematics(modelFreeflyer, dataFreeFlyer, qff, vff);

  computeAllTerms(modelSpherical, dataSpherical, q, v);
  computeAllTerms(modelFreeflyer, dataFreeFlyer, qff, vff);

  BOOST_CHECK(dataFreeFlyer.oMi[1].isApprox(dataSpherical.oMi[1]));
  BOOST_CHECK(dataFreeFlyer.liMi[1].isApprox(dataSpherical.liMi[1]));
  BOOST_CHECK(dataFreeFlyer.Ycrb[1].matrix().isApprox(dataSpherical.Ycrb[1].matrix()));
  BOOST_CHECK(dataFreeFlyer.f[1].toVector().isApprox(dataSpherical.f[1].toVector()));
  
  Eigen::VectorXd nle_expected_ff(3); nle_expected_ff << dataFreeFlyer.nle[3],
                                                         dataFreeFlyer.nle[4],
                                                         dataFreeFlyer.nle[5]
                                                         ;
  BOOST_CHECK(nle_expected_ff.isApprox(dataSpherical.nle));
  BOOST_CHECK(dataFreeFlyer.com[0].isApprox(dataSpherical.com[0]));

  // InverseDynamics == rnea
  tauSpherical = rnea(modelSpherical, dataSpherical, q, v, aSpherical);
  tauff = rnea(modelFreeflyer, dataFreeFlyer, qff, vff, aff);

  Vector3 tau_expected; tau_expected << tauff(3), tauff(4), tauff(5);
  BOOST_CHECK(tauSpherical.isApprox(tau_expected));

  // ForwardDynamics == aba
  Eigen::VectorXd aAbaSpherical = aba(modelSpherical,dataSpherical, q, v, tauSpherical);
  Eigen::VectorXd aAbaFreeFlyer = aba(modelFreeflyer,dataFreeFlyer, qff, vff, tauff);
  Vector3 a_expected; a_expected << aAbaFreeFlyer[3],
                                    aAbaFreeFlyer[4],
                                    aAbaFreeFlyer[5]
                                    ;
  BOOST_CHECK(aAbaSpherical.isApprox(a_expected));

  // crba
  crba(modelSpherical, dataSpherical,q);
  crba(modelFreeflyer, dataFreeFlyer, qff);

  Eigen::Matrix<double, 3, 3> M_expected(dataFreeFlyer.M.bottomRightCorner<3,3>());

  BOOST_CHECK(dataSpherical.M.isApprox(M_expected));
   
  // Jacobian
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian_planar;jacobian_planar.resize(6,3); jacobian_planar.setZero();
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian_ff;jacobian_ff.resize(6,6);jacobian_ff.setZero();
  computeJointJacobians(modelSpherical, dataSpherical, q);
  computeJointJacobians(modelFreeflyer, dataFreeFlyer, qff);
  getJointJacobian(modelSpherical, dataSpherical, 1, LOCAL, jacobian_planar);
  getJointJacobian(modelFreeflyer, dataFreeFlyer, 1, LOCAL, jacobian_ff);


  Eigen::Matrix<double, 6, 3> jacobian_expected; jacobian_expected << jacobian_ff.col(3),
                                                                      jacobian_ff.col(4),
                                                                      jacobian_ff.col(5)
                                                                      ;

  BOOST_CHECK(jacobian_planar.isApprox(jacobian_expected));

}
BOOST_AUTO_TEST_SUITE_END ()


BOOST_AUTO_TEST_SUITE (JointSphericalZYX)
  
  BOOST_AUTO_TEST_CASE(spatial)
  {
    SE3 M(SE3::Random());
    Motion v(Motion::Random());
    
    MotionSpherical mp(MotionSpherical::Vector3(1.,2.,3.));
    Motion mp_dense(mp);
    
    BOOST_CHECK(M.act(mp).isApprox(M.act(mp_dense)));
    BOOST_CHECK(M.actInv(mp).isApprox(M.actInv(mp_dense)));
    
    BOOST_CHECK(v.cross(mp).isApprox(v.cross(mp_dense)));
  }

BOOST_AUTO_TEST_CASE (vsFreeFlyer)
{
  // WARNIG : Dynamic algorithm's results cannot be compared to FreeFlyer's ones because 
  // of the representation of the rotation and the ConstraintSubspace difference.
  using namespace pinocchio;
  typedef Eigen::Matrix <double, 3, 1> Vector3;
  typedef Eigen::Matrix <double, 6, 1> Vector6;
  typedef Eigen::Matrix <double, 7, 1> VectorFF;
  typedef Eigen::Matrix <double, 3, 3> Matrix3;

  Model modelSphericalZYX, modelFreeflyer;

  Inertia inertia (1., Vector3 (0.5, 0., 0.0), Matrix3::Identity ());
  SE3 pos(1); pos.translation() = SE3::LinearType(1.,0.,0.);

  addJointAndBody(modelSphericalZYX,JointModelSphericalZYX(),0,pos,"spherical-zyx",inertia);
  addJointAndBody(modelFreeflyer,JointModelFreeFlyer(),0,pos,"free-flyer",inertia);

  Data dataSphericalZYX(modelSphericalZYX);
  Data dataFreeFlyer(modelFreeflyer);

  Eigen::AngleAxisd rollAngle(1, Eigen::Vector3d::UnitZ());
  Eigen::AngleAxisd yawAngle(1, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd pitchAngle(1, Eigen::Vector3d::UnitX());
  Eigen::Quaterniond q_sph = rollAngle * yawAngle * pitchAngle;
  
  Eigen::VectorXd q = Eigen::VectorXd::Ones (modelSphericalZYX.nq);
  VectorFF qff; qff << 0, 0, 0, q_sph.x(), q_sph.y(), q_sph.z(), q_sph.w();
  Eigen::VectorXd v = Eigen::VectorXd::Ones (modelSphericalZYX.nv);
  Vector6 vff; vff << 0, 0, 0, 1, 1, 1;
  Eigen::VectorXd tauSpherical = Eigen::VectorXd::Ones (modelSphericalZYX.nv);
  Eigen::VectorXd tauff; tauff.resize(6); tauff << 0,0,0,1,1,1;
  Eigen::VectorXd aSpherical = Eigen::VectorXd::Ones (modelSphericalZYX.nv);
  Eigen::VectorXd aff(vff);
  
  forwardKinematics(modelSphericalZYX, dataSphericalZYX, q, v);
  forwardKinematics(modelFreeflyer, dataFreeFlyer, qff, vff);

  computeAllTerms(modelSphericalZYX, dataSphericalZYX, q, v);
  computeAllTerms(modelFreeflyer, dataFreeFlyer, qff, vff);

  BOOST_CHECK(dataFreeFlyer.oMi[1].isApprox(dataSphericalZYX.oMi[1]));
  BOOST_CHECK(dataFreeFlyer.liMi[1].isApprox(dataSphericalZYX.liMi[1]));
  BOOST_CHECK(dataFreeFlyer.Ycrb[1].matrix().isApprox(dataSphericalZYX.Ycrb[1].matrix()));

  BOOST_CHECK(dataFreeFlyer.com[0].isApprox(dataSphericalZYX.com[0]));
}

BOOST_AUTO_TEST_CASE ( test_rnea )
{
  using namespace pinocchio;
  typedef Eigen::Matrix <double, 3, 1> Vector3;
  typedef Eigen::Matrix <double, 3, 3> Matrix3;

  Model model;
  Inertia inertia (1., Vector3 (0.5, 0., 0.0), Matrix3::Identity ());

  addJointAndBody(model,JointModelSphericalZYX(),model.getJointId("universe"),SE3::Identity(),"root",inertia);

  Data data (model);

  Eigen::VectorXd q = Eigen::VectorXd::Zero (model.nq);
  Eigen::VectorXd v = Eigen::VectorXd::Zero (model.nv);
  Eigen::VectorXd a = Eigen::VectorXd::Zero (model.nv);

  rnea (model, data, q, v, a);
  Vector3 tau_expected (0., -4.905, 0.);

  BOOST_CHECK (tau_expected.isApprox(data.tau, 1e-14));

  q = Eigen::VectorXd::Ones (model.nq);
  v = Eigen::VectorXd::Ones (model.nv);
  a = Eigen::VectorXd::Ones (model.nv);

  rnea (model, data, q, v, a);
  tau_expected << -0.53611600195085, -0.74621832606188, -0.38177329067604;

  BOOST_CHECK (tau_expected.isApprox(data.tau, 1e-12));

  q << 3, 2, 1;
  v = Eigen::VectorXd::Ones (model.nv);
  a = Eigen::VectorXd::Ones (model.nv);

  rnea (model, data, q, v, a);
  tau_expected << 0.73934458094049,  2.7804530848031, 0.50684940972146;

  BOOST_CHECK (tau_expected.isApprox(data.tau, 1e-12));
}

BOOST_AUTO_TEST_CASE ( test_crba )
{
  using namespace pinocchio;
  using namespace std;
  typedef Eigen::Matrix <double, 3, 1> Vector3;
  typedef Eigen::Matrix <double, 3, 3> Matrix3;

  Model model;
  Inertia inertia (1., Vector3 (0.5, 0., 0.0), Matrix3::Identity ());

  addJointAndBody(model,JointModelSphericalZYX(),model.getJointId("universe"),SE3::Identity(),"root",inertia);

  Data data (model);

  Eigen::VectorXd q (Eigen::VectorXd::Zero (model.nq));
  Eigen::MatrixXd M_expected (model.nv,model.nv);

  crba (model, data, q);
  M_expected <<
  1.25,    0,    0,
  0, 1.25,    0,
  0,    0,    1;

  BOOST_CHECK (M_expected.isApprox(data.M, 1e-14));

  q = Eigen::VectorXd::Ones (model.nq);

  crba (model, data, q);
  M_expected <<
  1.0729816454316, -5.5511151231258e-17,     -0.8414709848079,
  -5.5511151231258e-17,                 1.25,                    0,
  -0.8414709848079,                    0,                    1;

  BOOST_CHECK (M_expected.isApprox(data.M, 1e-12));

  q << 3, 2, 1;

  crba (model, data, q);
  M_expected <<
  1.043294547392, 2.7755575615629e-17,   -0.90929742682568,
  0,                1.25,                   0,
  -0.90929742682568,                   0,                  1;

  BOOST_CHECK (M_expected.isApprox(data.M, 1e-10));
}

BOOST_AUTO_TEST_SUITE_END ()

BOOST_AUTO_TEST_SUITE ( JointPrismatic )
  
BOOST_AUTO_TEST_CASE(spatial)
{
  typedef TransformPrismaticTpl<double,0,0> TransformX;
  typedef TransformPrismaticTpl<double,0,1> TransformY;
  typedef TransformPrismaticTpl<double,0,2> TransformZ;
  
  typedef SE3::Vector3 Vector3;
  
  const double displacement = 0.2;
  SE3 Mplain, Mrand(SE3::Random());
  
  TransformX Mx(displacement);
  Mplain = Mx;
  BOOST_CHECK(Mplain.translation().isApprox(Vector3(displacement,0,0)));
  BOOST_CHECK(Mplain.rotation().isIdentity());
  BOOST_CHECK((Mrand*Mplain).isApprox(Mrand*Mx));
  
  TransformY My(displacement);
  Mplain = My;
  BOOST_CHECK(Mplain.translation().isApprox(Vector3(0,displacement,0)));
  BOOST_CHECK(Mplain.rotation().isIdentity());
  BOOST_CHECK((Mrand*Mplain).isApprox(Mrand*My));
  
  TransformZ Mz(displacement);
  Mplain = Mz;
  BOOST_CHECK(Mplain.translation().isApprox(Vector3(0,0,displacement)));
  BOOST_CHECK(Mplain.rotation().isIdentity());
  BOOST_CHECK((Mrand*Mplain).isApprox(Mrand*Mz));
  
  SE3 M(SE3::Random());
  Motion v(Motion::Random());
  
  MotionPrismaticTpl<double,0,0> mp_x(2.);
  Motion mp_dense_x(mp_x);
  
  BOOST_CHECK(M.act(mp_x).isApprox(M.act(mp_dense_x)));
  BOOST_CHECK(M.actInv(mp_x).isApprox(M.actInv(mp_dense_x)));
  
  BOOST_CHECK(v.cross(mp_x).isApprox(v.cross(mp_dense_x)));
  
  MotionPrismaticTpl<double,0,1> mp_y(2.);
  Motion mp_dense_y(mp_y);
  
  BOOST_CHECK(M.act(mp_y).isApprox(M.act(mp_dense_y)));
  BOOST_CHECK(M.actInv(mp_y).isApprox(M.actInv(mp_dense_y)));
  
  BOOST_CHECK(v.cross(mp_y).isApprox(v.cross(mp_dense_y)));
  
  MotionPrismaticTpl<double,0,2> mp_z(2.);
  Motion mp_dense_z(mp_z);
  
  BOOST_CHECK(M.act(mp_z).isApprox(M.act(mp_dense_z)));
  BOOST_CHECK(M.actInv(mp_z).isApprox(M.actInv(mp_dense_z)));
  
  BOOST_CHECK(v.cross(mp_z).isApprox(v.cross(mp_dense_z)));
}

BOOST_AUTO_TEST_CASE ( test_kinematics )
{
  using namespace pinocchio;


  Motion expected_v_J (Motion::Zero ());
  Motion expected_c_J (Motion::Zero ());

  SE3 expected_configuration (SE3::Identity ());

  JointDataPX joint_data;
  JointModelPX joint_model;

  joint_model.setIndexes (0, 0, 0);

  Eigen::VectorXd q (Eigen::VectorXd::Zero (1));
  Eigen::VectorXd q_dot (Eigen::VectorXd::Zero (1));

  // -------
  q << 0. ;
  q_dot << 0.;

  joint_model.calc (joint_data, q, q_dot);

  printOutJointData <JointDataPX> (q, q_dot, joint_data);

  BOOST_CHECK (expected_configuration.rotation ().isApprox(joint_data.M.rotation(), 1e-12));
  BOOST_CHECK (expected_configuration.translation ().isApprox(joint_data.M.translation (), 1e-12));
  BOOST_CHECK (expected_v_J.toVector ().isApprox(((Motion) joint_data.v).toVector(), 1e-12));
  BOOST_CHECK (expected_c_J.toVector ().isApprox(((Motion) joint_data.c).toVector(), 1e-12));

  // -------
  q << 1.;
  q_dot << 1.;


  joint_model.calc (joint_data, q, q_dot);

  printOutJointData <JointDataPX> (q, q_dot, joint_data);

  expected_configuration.translation () << 1, 0, 0;

  expected_v_J.linear () << 1., 0., 0.;

  BOOST_CHECK (expected_configuration.rotation ().isApprox(joint_data.M.rotation(), 1e-12));
  BOOST_CHECK (expected_configuration.translation ().isApprox(joint_data.M.translation (), 1e-12));
  BOOST_CHECK (expected_v_J.toVector ().isApprox(((Motion) joint_data.v).toVector(), 1e-12));
  BOOST_CHECK (expected_c_J.toVector ().isApprox(((Motion) joint_data.c).toVector(), 1e-12));
}

BOOST_AUTO_TEST_CASE ( test_rnea )
{
  using namespace pinocchio;
  typedef Eigen::Matrix <double, 3, 1> Vector3;
  typedef Eigen::Matrix <double, 3, 3> Matrix3;

  Model model;
  Inertia inertia (1., Vector3 (0.5, 0., 0.0), Matrix3::Identity ());

  addJointAndBody(model,JointModelPX(),model.getJointId("universe"),SE3::Identity(),"root",inertia);

  Data data (model);

  Eigen::VectorXd q (Eigen::VectorXd::Zero (model.nq));
  Eigen::VectorXd v (Eigen::VectorXd::Zero (model.nv));
  Eigen::VectorXd a (Eigen::VectorXd::Zero (model.nv));

  rnea (model, data, q, v, a);

  Eigen::VectorXd tau_expected (Eigen::VectorXd::Zero (model.nq));
  tau_expected  << 0;

  BOOST_CHECK (tau_expected.isApprox(data.tau, 1e-14));

  // -----
  q = Eigen::VectorXd::Ones (model.nq);
  v = Eigen::VectorXd::Ones (model.nv);
  a = Eigen::VectorXd::Ones (model.nv);

  rnea (model, data, q, v, a);
  tau_expected << 1;

  BOOST_CHECK (tau_expected.isApprox(data.tau, 1e-12));

  q << 3;
  v = Eigen::VectorXd::Ones (model.nv);
  a = Eigen::VectorXd::Ones (model.nv);

  rnea (model, data, q, v, a);
  tau_expected << 1;

  BOOST_CHECK (tau_expected.isApprox(data.tau, 1e-12));
}

BOOST_AUTO_TEST_CASE ( test_crba )
{
  using namespace pinocchio;
  using namespace std;
  typedef Eigen::Matrix <double, 3, 1> Vector3;
  typedef Eigen::Matrix <double, 3, 3> Matrix3;

  Model model;
  Inertia inertia (1., Vector3 (0.5, 0., 0.0), Matrix3::Identity ());

  addJointAndBody(model,JointModelPX(),model.getJointId("universe"),SE3::Identity(),"root",inertia);

  Data data (model);

  Eigen::VectorXd q (Eigen::VectorXd::Zero (model.nq));
  Eigen::MatrixXd M_expected (model.nv,model.nv);

  crba (model, data, q);
  M_expected << 1.0;

  BOOST_CHECK (M_expected.isApprox(data.M, 1e-14));

  q = Eigen::VectorXd::Ones (model.nq);

  crba (model, data, q);

  BOOST_CHECK (M_expected.isApprox(data.M, 1e-12));

  q << 3;

  crba (model, data, q);
  
  BOOST_CHECK (M_expected.isApprox(data.M, 1e-10));
}

BOOST_AUTO_TEST_SUITE_END ()

BOOST_AUTO_TEST_SUITE (JointPlanar)
  
BOOST_AUTO_TEST_CASE(spatial)
{
  SE3 M(SE3::Random());
  Motion v(Motion::Random());
  
  MotionPlanar mp(1.,2.,3.);
  Motion mp_dense(mp);
  
  BOOST_CHECK(M.act(mp).isApprox(M.act(mp_dense)));
  BOOST_CHECK(M.actInv(mp).isApprox(M.actInv(mp_dense)));
  
  BOOST_CHECK(v.cross(mp).isApprox(v.cross(mp_dense)));
}

BOOST_AUTO_TEST_CASE (vsFreeFlyer)
{
  using namespace pinocchio;
  typedef Eigen::Matrix <double, 3, 1> Vector3;
  typedef Eigen::Matrix <double, 6, 1> Vector6;
  typedef Eigen::Matrix <double, 4, 1> VectorPl;
  typedef Eigen::Matrix <double, 7, 1> VectorFF;
  typedef Eigen::Matrix <double, 3, 3> Matrix3;

  Model modelPlanar, modelFreeflyer;

  Inertia inertia (1., Vector3 (0.5, 0., 0.0), Matrix3::Identity ());
  SE3 pos(1); pos.translation() = SE3::LinearType(1.,0.,0.);

  addJointAndBody(modelPlanar,JointModelPlanar(),0,SE3::Identity(),"planar",inertia);
  addJointAndBody(modelFreeflyer,JointModelFreeFlyer(),0,SE3::Identity(),"free-flyer",inertia);

  Data dataPlanar(modelPlanar);
  Data dataFreeFlyer(modelFreeflyer);

  VectorPl q; q << 1, 1, 0, 1; // Angle is PI /2;
  VectorFF qff; qff << 1, 1, 0, 0, 0, sqrt(2)/2, sqrt(2)/2 ;
  Eigen::VectorXd v = Eigen::VectorXd::Ones (modelPlanar.nv);
  Vector6 vff; vff << 1, 1, 0, 0, 0, 1;
  Eigen::VectorXd tauPlanar = Eigen::VectorXd::Ones (modelPlanar.nv);
  Eigen::VectorXd tauff = Eigen::VectorXd::Ones (modelFreeflyer.nv);
  Eigen::VectorXd aPlanar = Eigen::VectorXd::Ones (modelPlanar.nv);
  Eigen::VectorXd aff(vff);
  
  forwardKinematics(modelPlanar, dataPlanar, q, v);
  forwardKinematics(modelFreeflyer, dataFreeFlyer, qff, vff);

  computeAllTerms(modelPlanar, dataPlanar, q, v);
  computeAllTerms(modelFreeflyer, dataFreeFlyer, qff, vff);

  BOOST_CHECK(dataFreeFlyer.oMi[1].isApprox(dataPlanar.oMi[1]));
  BOOST_CHECK(dataFreeFlyer.liMi[1].isApprox(dataPlanar.liMi[1]));
  BOOST_CHECK(dataFreeFlyer.Ycrb[1].matrix().isApprox(dataPlanar.Ycrb[1].matrix()));
  BOOST_CHECK(dataFreeFlyer.f[1].toVector().isApprox(dataPlanar.f[1].toVector()));
  
  Eigen::VectorXd nle_expected_ff(3); nle_expected_ff << dataFreeFlyer.nle[0],
                                                         dataFreeFlyer.nle[1],
                                                         dataFreeFlyer.nle[5]
                                                         ;
  BOOST_CHECK(nle_expected_ff.isApprox(dataPlanar.nle));
  BOOST_CHECK(dataFreeFlyer.com[0].isApprox(dataPlanar.com[0]));

  // InverseDynamics == rnea
  tauPlanar = rnea(modelPlanar, dataPlanar, q, v, aPlanar);
  tauff = rnea(modelFreeflyer, dataFreeFlyer, qff, vff, aff);

  Vector3 tau_expected; tau_expected << tauff(0), tauff(1), tauff(5);
  BOOST_CHECK(tauPlanar.isApprox(tau_expected));

  // ForwardDynamics == aba
  Eigen::VectorXd aAbaPlanar = aba(modelPlanar,dataPlanar, q, v, tauPlanar);
  Eigen::VectorXd aAbaFreeFlyer = aba(modelFreeflyer,dataFreeFlyer, qff, vff, tauff);
  Vector3 a_expected; a_expected << aAbaFreeFlyer[0],
                                    aAbaFreeFlyer[1],
                                    aAbaFreeFlyer[5]
                                    ;
  BOOST_CHECK(aAbaPlanar.isApprox(a_expected));

  // crba
  crba(modelPlanar, dataPlanar,q);
  crba(modelFreeflyer, dataFreeFlyer, qff);

  Eigen::Matrix<double, 3, 3> M_expected;
  M_expected.block<2,2>(0,0) = dataFreeFlyer.M.block<2,2>(0,0);
  M_expected.block<1,2>(2,0) = dataFreeFlyer.M.block<1,2>(5,0);
  M_expected.block<2,1>(0,2) = dataFreeFlyer.M.col(5).head<2>();
  M_expected.block<1,1>(2,2) = dataFreeFlyer.M.col(5).tail<1>();

  BOOST_CHECK(dataPlanar.M.isApprox(M_expected));
   
  // Jacobian
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian_planar;jacobian_planar.resize(6,3); jacobian_planar.setZero();
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian_ff;jacobian_ff.resize(6,6);jacobian_ff.setZero();
  computeJointJacobians(modelPlanar, dataPlanar, q);
  computeJointJacobians(modelFreeflyer, dataFreeFlyer, qff);
  getJointJacobian(modelPlanar, dataPlanar, 1, LOCAL, jacobian_planar);
  getJointJacobian(modelFreeflyer, dataFreeFlyer, 1, LOCAL, jacobian_ff);

  Eigen::Matrix<double, 6, 3> jacobian_expected; jacobian_expected << jacobian_ff.col(0),
                                                                      jacobian_ff.col(1),
                                                                      jacobian_ff.col(5)
                                                                      ;

  BOOST_CHECK(jacobian_planar.isApprox(jacobian_expected));
}
BOOST_AUTO_TEST_SUITE_END ()

BOOST_AUTO_TEST_SUITE (JointTranslation)
  
BOOST_AUTO_TEST_CASE(spatial)
{
  typedef TransformTranslationTpl<double,0> TransformTranslation;
  typedef SE3::Vector3 Vector3;
  
  const Vector3 displacement(Vector3::Random());
  SE3 Mplain, Mrand(SE3::Random());
  
  TransformTranslation Mtrans(displacement);
  Mplain = Mtrans;
  BOOST_CHECK(Mplain.translation().isApprox(displacement));
  BOOST_CHECK(Mplain.rotation().isIdentity());
  BOOST_CHECK((Mrand*Mplain).isApprox(Mrand*Mtrans));
  
  SE3 M(SE3::Random());
  Motion v(Motion::Random());
  
  MotionTranslation mp(MotionTranslation::Vector3(1.,2.,3.));
  Motion mp_dense(mp);
  
  BOOST_CHECK(M.act(mp).isApprox(M.act(mp_dense)));
  BOOST_CHECK(M.actInv(mp).isApprox(M.actInv(mp_dense)));
  
  BOOST_CHECK(v.cross(mp).isApprox(v.cross(mp_dense)));
}

BOOST_AUTO_TEST_CASE (vsFreeFlyer)
{
  using namespace pinocchio;
  typedef Eigen::Matrix <double, 3, 1> Vector3;
  typedef Eigen::Matrix <double, 6, 1> Vector6;
  typedef Eigen::Matrix <double, 7, 1> VectorFF;
  typedef Eigen::Matrix <double, 3, 3> Matrix3;

  Model modelTranslation, modelFreeflyer;

  Inertia inertia (1., Vector3 (0.5, 0., 0.0), Matrix3::Identity ());
  SE3 pos(1); pos.translation() = SE3::LinearType(1.,0.,0.);

  addJointAndBody(modelTranslation,JointModelTranslation(),0,SE3::Identity(),"translation",inertia);
  addJointAndBody(modelFreeflyer,JointModelFreeFlyer(),0,SE3::Identity(),"free-flyer",inertia);

  Data dataTranslation(modelTranslation);
  Data dataFreeFlyer(modelFreeflyer);

  Eigen::VectorXd q = Eigen::VectorXd::Ones (modelTranslation.nq);               VectorFF qff; qff << 1, 1, 1, 0, 0, 0, 1 ;
  Eigen::VectorXd v = Eigen::VectorXd::Ones (modelTranslation.nv);               Vector6 vff; vff << 1, 1, 1, 0, 0, 0;
  Eigen::VectorXd tauTranslation = Eigen::VectorXd::Ones (modelTranslation.nv);       Eigen::VectorXd tauff(6); tauff << 1, 1, 1, 0, 0, 0;
  Eigen::VectorXd aTranslation = Eigen::VectorXd::Ones (modelTranslation.nv);         Eigen::VectorXd aff(vff);
  
  forwardKinematics(modelTranslation, dataTranslation, q, v);
  forwardKinematics(modelFreeflyer, dataFreeFlyer, qff, vff);

  computeAllTerms(modelTranslation, dataTranslation, q, v);
  computeAllTerms(modelFreeflyer, dataFreeFlyer, qff, vff);

  BOOST_CHECK(dataFreeFlyer.oMi[1].isApprox(dataTranslation.oMi[1]));
  BOOST_CHECK(dataFreeFlyer.liMi[1].isApprox(dataTranslation.liMi[1]));
  BOOST_CHECK(dataFreeFlyer.Ycrb[1].matrix().isApprox(dataTranslation.Ycrb[1].matrix()));
  BOOST_CHECK(dataFreeFlyer.f[1].toVector().isApprox(dataTranslation.f[1].toVector()));
  
  Eigen::VectorXd nle_expected_ff(3); nle_expected_ff << dataFreeFlyer.nle[0],
                                                         dataFreeFlyer.nle[1],
                                                         dataFreeFlyer.nle[2]
                                                         ;
  BOOST_CHECK(nle_expected_ff.isApprox(dataTranslation.nle));
  BOOST_CHECK(dataFreeFlyer.com[0].isApprox(dataTranslation.com[0]));

  // InverseDynamics == rnea
  tauTranslation = rnea(modelTranslation, dataTranslation, q, v, aTranslation);
  tauff = rnea(modelFreeflyer, dataFreeFlyer, qff, vff, aff);

  Vector3 tau_expected; tau_expected << tauff(0), tauff(1), tauff(2);
  BOOST_CHECK(tauTranslation.isApprox(tau_expected));

  // ForwardDynamics == aba
  Eigen::VectorXd aAbaTranslation = aba(modelTranslation,dataTranslation, q, v, tauTranslation);
  Eigen::VectorXd aAbaFreeFlyer = aba(modelFreeflyer,dataFreeFlyer, qff, vff, tauff);
  Vector3 a_expected; a_expected << aAbaFreeFlyer[0],
                                    aAbaFreeFlyer[1],
                                    aAbaFreeFlyer[2]
                                    ;
  BOOST_CHECK(aAbaTranslation.isApprox(a_expected));

  // crba
  crba(modelTranslation, dataTranslation,q);
  crba(modelFreeflyer, dataFreeFlyer, qff);

  Eigen::Matrix<double, 3, 3> M_expected(dataFreeFlyer.M.topLeftCorner<3,3>());

  BOOST_CHECK(dataTranslation.M.isApprox(M_expected));
   
  // Jacobian
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian_planar;jacobian_planar.resize(6,3); jacobian_planar.setZero();
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian_ff;jacobian_ff.resize(6,6);jacobian_ff.setZero();
  computeJointJacobians(modelTranslation, dataTranslation, q);
  computeJointJacobians(modelFreeflyer, dataFreeFlyer, qff);
  getJointJacobian(modelTranslation, dataTranslation, 1, LOCAL, jacobian_planar);
  getJointJacobian(modelFreeflyer, dataFreeFlyer, 1, LOCAL, jacobian_ff);


  Eigen::Matrix<double, 6, 3> jacobian_expected; jacobian_expected << jacobian_ff.col(0),
                                                                      jacobian_ff.col(1),
                                                                      jacobian_ff.col(2)
                                                                      ;

  BOOST_CHECK(jacobian_planar.isApprox(jacobian_expected));
}
BOOST_AUTO_TEST_SUITE_END ()

BOOST_AUTO_TEST_SUITE (JointRevoluteUnbounded)
  
  BOOST_AUTO_TEST_CASE(spatial)
  {
    SE3 M(SE3::Random());
    Motion v(Motion::Random());
    
    MotionRevoluteTpl<double,0,0> mp_x(2.);
    Motion mp_dense_x(mp_x);
    
    BOOST_CHECK(M.act(mp_x).isApprox(M.act(mp_dense_x)));
    BOOST_CHECK(M.actInv(mp_x).isApprox(M.actInv(mp_dense_x)));
    
    BOOST_CHECK(v.cross(mp_x).isApprox(v.cross(mp_dense_x)));
    
    MotionRevoluteTpl<double,0,1> mp_y(2.);
    Motion mp_dense_y(mp_y);
    
    BOOST_CHECK(M.act(mp_y).isApprox(M.act(mp_dense_y)));
    BOOST_CHECK(M.actInv(mp_y).isApprox(M.actInv(mp_dense_y)));
    
    BOOST_CHECK(v.cross(mp_y).isApprox(v.cross(mp_dense_y)));
    
    MotionRevoluteTpl<double,0,2> mp_z(2.);
    Motion mp_dense_z(mp_z);
    
    BOOST_CHECK(M.act(mp_z).isApprox(M.act(mp_dense_z)));
    BOOST_CHECK(M.actInv(mp_z).isApprox(M.actInv(mp_dense_z)));
    
    BOOST_CHECK(v.cross(mp_z).isApprox(v.cross(mp_dense_z)));
  }

BOOST_AUTO_TEST_CASE (vsRX)
{
  typedef Eigen::Matrix <double, 3, 1> Vector3;
  typedef Eigen::Matrix <double, 3, 3> Matrix3;


  Model modelRX, modelRevoluteUnbounded;

  Inertia inertia (1., Vector3 (0.5, 0., 0.0), Matrix3::Identity ());
  SE3 pos(1); pos.translation() = SE3::LinearType(1.,0.,0.);

  JointModelRUBX joint_model_RUX;
  addJointAndBody(modelRX,JointModelRX(),0,SE3::Identity(),"rx",inertia);
  addJointAndBody(modelRevoluteUnbounded,joint_model_RUX,0,SE3::Identity(),"revolute unbounded x",inertia);

  Data dataRX(modelRX);
  Data dataRevoluteUnbounded(modelRevoluteUnbounded);


  Eigen::VectorXd q_rx = Eigen::VectorXd::Ones (modelRX.nq);
  Eigen::VectorXd q_rubx = Eigen::VectorXd::Ones (modelRevoluteUnbounded.nq);
  double ca, sa; double alpha = q_rx(0); SINCOS (alpha, &sa, &ca);
  q_rubx(0) = ca;
  q_rubx(1) = sa;
  Eigen::VectorXd v_rx = Eigen::VectorXd::Ones (modelRX.nv);
  Eigen::VectorXd v_rubx = v_rx;
  Eigen::VectorXd tauRX = Eigen::VectorXd::Ones (modelRX.nv);       Eigen::VectorXd tauRevoluteUnbounded = Eigen::VectorXd::Ones (modelRevoluteUnbounded.nv);
  Eigen::VectorXd aRX = Eigen::VectorXd::Ones (modelRX.nv);         Eigen::VectorXd aRevoluteUnbounded = aRX;
  


  forwardKinematics(modelRX, dataRX, q_rx, v_rx);
  forwardKinematics(modelRevoluteUnbounded, dataRevoluteUnbounded, q_rubx, v_rubx);

  computeAllTerms(modelRX, dataRX, q_rx, v_rx);
  computeAllTerms(modelRevoluteUnbounded, dataRevoluteUnbounded, q_rubx, v_rubx);

  BOOST_CHECK(dataRevoluteUnbounded.oMi[1].isApprox(dataRX.oMi[1]));
  BOOST_CHECK(dataRevoluteUnbounded.liMi[1].isApprox(dataRX.liMi[1]));
  BOOST_CHECK(dataRevoluteUnbounded.Ycrb[1].matrix().isApprox(dataRX.Ycrb[1].matrix()));
  BOOST_CHECK(dataRevoluteUnbounded.f[1].toVector().isApprox(dataRX.f[1].toVector()));
  
  BOOST_CHECK(dataRevoluteUnbounded.nle.isApprox(dataRX.nle));
  BOOST_CHECK(dataRevoluteUnbounded.com[0].isApprox(dataRX.com[0]));



  // InverseDynamics == rnea
  tauRX = rnea(modelRX, dataRX, q_rx, v_rx, aRX);
  tauRevoluteUnbounded = rnea(modelRevoluteUnbounded, dataRevoluteUnbounded, q_rubx, v_rubx, aRevoluteUnbounded);

  BOOST_CHECK(tauRX.isApprox(tauRevoluteUnbounded));

  // ForwardDynamics == aba
  Eigen::VectorXd aAbaRX= aba(modelRX,dataRX, q_rx, v_rx, tauRX);
  Eigen::VectorXd aAbaRevoluteUnbounded = aba(modelRevoluteUnbounded,dataRevoluteUnbounded, q_rubx, v_rubx, tauRevoluteUnbounded);


  BOOST_CHECK(aAbaRX.isApprox(aAbaRevoluteUnbounded));

  // crba
  crba(modelRX, dataRX,q_rx);
  crba(modelRevoluteUnbounded, dataRevoluteUnbounded, q_rubx);

  BOOST_CHECK(dataRX.M.isApprox(dataRevoluteUnbounded.M));
   
  // Jacobian
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobianPX;jacobianPX.resize(6,1); jacobianPX.setZero();
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobianPrismaticUnaligned;jacobianPrismaticUnaligned.resize(6,1);jacobianPrismaticUnaligned.setZero();
  computeJointJacobians(modelRX, dataRX, q_rx);
  computeJointJacobians(modelRevoluteUnbounded, dataRevoluteUnbounded, q_rubx);
  getJointJacobian(modelRX, dataRX, 1, LOCAL, jacobianPX);
  getJointJacobian(modelRevoluteUnbounded, dataRevoluteUnbounded, 1, LOCAL, jacobianPrismaticUnaligned);


  BOOST_CHECK(jacobianPX.isApprox(jacobianPrismaticUnaligned));


}
BOOST_AUTO_TEST_SUITE_END ()
  
BOOST_AUTO_TEST_SUITE(JointRevolute)
  
  BOOST_AUTO_TEST_CASE(spatial)
  {
    typedef TransformRevoluteTpl<double,0,0> TransformX;
    typedef TransformRevoluteTpl<double,0,1> TransformY;
    typedef TransformRevoluteTpl<double,0,2> TransformZ;
    
    typedef SE3::Vector3 Vector3;
    
    const double alpha = 0.2;
    double sin_alpha, cos_alpha; SINCOS(alpha,&sin_alpha,&cos_alpha);
    SE3 Mplain, Mrand(SE3::Random());
    
    TransformX Mx(sin_alpha,cos_alpha);
    Mplain = Mx;
    BOOST_CHECK(Mplain.translation().isZero());
    BOOST_CHECK(Mplain.rotation().isApprox(Eigen::AngleAxisd(alpha,Vector3::UnitX()).toRotationMatrix()));
    BOOST_CHECK((Mrand*Mplain).isApprox(Mrand*Mx));
    
    TransformY My(sin_alpha,cos_alpha);
    Mplain = My;
    BOOST_CHECK(Mplain.translation().isZero());
    BOOST_CHECK(Mplain.rotation().isApprox(Eigen::AngleAxisd(alpha,Vector3::UnitY()).toRotationMatrix()));
    BOOST_CHECK((Mrand*Mplain).isApprox(Mrand*My));
    
    TransformZ Mz(sin_alpha,cos_alpha);
    Mplain = Mz;
    BOOST_CHECK(Mplain.translation().isZero());
    BOOST_CHECK(Mplain.rotation().isApprox(Eigen::AngleAxisd(alpha,Vector3::UnitZ()).toRotationMatrix()));
    BOOST_CHECK((Mrand*Mplain).isApprox(Mrand*Mz));
    
    SE3 M(SE3::Random());
    Motion v(Motion::Random());
    
    MotionRevoluteTpl<double,0,0> mp_x(2.);
    Motion mp_dense_x(mp_x);
    
    BOOST_CHECK(M.act(mp_x).isApprox(M.act(mp_dense_x)));
    BOOST_CHECK(M.actInv(mp_x).isApprox(M.actInv(mp_dense_x)));
    
    BOOST_CHECK(v.cross(mp_x).isApprox(v.cross(mp_dense_x)));
    
    MotionRevoluteTpl<double,0,1> mp_y(2.);
    Motion mp_dense_y(mp_y);
    
    BOOST_CHECK(M.act(mp_y).isApprox(M.act(mp_dense_y)));
    BOOST_CHECK(M.actInv(mp_y).isApprox(M.actInv(mp_dense_y)));
    
    BOOST_CHECK(v.cross(mp_y).isApprox(v.cross(mp_dense_y)));
    
    MotionRevoluteTpl<double,0,2> mp_z(2.);
    Motion mp_dense_z(mp_z);
    
    BOOST_CHECK(M.act(mp_z).isApprox(M.act(mp_dense_z)));
    BOOST_CHECK(M.actInv(mp_z).isApprox(M.actInv(mp_dense_z)));
    
    BOOST_CHECK(v.cross(mp_z).isApprox(v.cross(mp_dense_z)));
  }
  
BOOST_AUTO_TEST_SUITE_END()
  
BOOST_AUTO_TEST_SUITE(JointRevoluteUnaligned)
  
  BOOST_AUTO_TEST_CASE(spatial)
  {
    SE3 M(SE3::Random());
    Motion v(Motion::Random());
    
    MotionRevoluteUnaligned mp(MotionRevoluteUnaligned::Vector3(1.,2.,3.),6.);
    Motion mp_dense(mp);
    
    BOOST_CHECK(M.act(mp).isApprox(M.act(mp_dense)));
    BOOST_CHECK(M.actInv(mp).isApprox(M.actInv(mp_dense)));
    
    BOOST_CHECK(v.cross(mp).isApprox(v.cross(mp_dense)));
  }
  
BOOST_AUTO_TEST_SUITE_END()
  
  template<typename JointModel_> struct init;
  
  template<typename JointModel_>
  struct init
  {
    static JointModel_ run()
    {
      JointModel_ jmodel;
      jmodel.setIndexes(0,0,0);
      return jmodel;
    }
  };
  
  template<typename Scalar, int Options>
  struct init<pinocchio::JointModelRevoluteUnalignedTpl<Scalar,Options> >
  {
    typedef pinocchio::JointModelRevoluteUnalignedTpl<Scalar,Options> JointModel;
    
    static JointModel run()
    {
      typedef typename JointModel::Vector3 Vector3;
      JointModel jmodel(Vector3::Random().normalized());
      
      jmodel.setIndexes(0,0,0);
      return jmodel;
    }
  };

  template<typename Scalar, int Options>
  struct init<pinocchio::JointModelRevoluteUnboundedUnalignedTpl<Scalar,Options> >
  {
    typedef pinocchio::JointModelRevoluteUnboundedUnalignedTpl<Scalar,Options> JointModel;
    
    static JointModel run()
    {    
      typedef typename JointModel::Vector3 Vector3;
      JointModel jmodel(Vector3::Random().normalized());
           
      jmodel.setIndexes(0,0,0);
      return jmodel;
    }    
  };
  
  template<typename Scalar, int Options>
  struct init<pinocchio::JointModelPrismaticUnalignedTpl<Scalar,Options> >
  {
    typedef pinocchio::JointModelPrismaticUnalignedTpl<Scalar,Options> JointModel;
    
    static JointModel run()
    {
      typedef typename JointModel::Vector3 Vector3;
      JointModel jmodel(Vector3::Random().normalized());
      
      jmodel.setIndexes(0,0,0);
      return jmodel;
    }
  };
  
  template<typename Scalar, int Options, template<typename,int> class JointCollection>
  struct init<pinocchio::JointModelTpl<Scalar,Options,JointCollection> >
  {
    typedef pinocchio::JointModelTpl<Scalar,Options,JointCollection> JointModel;
    
    static JointModel run()
    {
      typedef pinocchio::JointModelRevoluteTpl<Scalar,Options,0> JointModelRX;
      JointModel jmodel((JointModelRX()));
      
      jmodel.setIndexes(0,0,0);
      return jmodel;
    }
  };
  
  template<typename Scalar, int Options, template<typename,int> class JointCollection>
  struct init<pinocchio::JointModelCompositeTpl<Scalar,Options,JointCollection> >
  {
    typedef pinocchio::JointModelCompositeTpl<Scalar,Options,JointCollection> JointModel;
    
    static JointModel run()
    {
      typedef pinocchio::JointModelRevoluteTpl<Scalar,Options,0> JointModelRX;
      typedef pinocchio::JointModelRevoluteTpl<Scalar,Options,1> JointModelRY;
      JointModel jmodel((JointModelRX()));
      jmodel.addJoint(JointModelRY());
      
      jmodel.setIndexes(0,0,0);
      return jmodel;
    }
  };
  
  template<typename JointModel_>
  struct init<pinocchio::JointModelMimic<JointModel_> >
  {
    typedef pinocchio::JointModelMimic<JointModel_> JointModel;
    
    static JointModel run()
    {
      JointModel_ jmodel_ref = init<JointModel_>::run();
      
      JointModel jmodel(jmodel_ref,1.,0.);
      jmodel.setIndexes(0,0,0);
      
      return jmodel;
    }
  };
  
BOOST_AUTO_TEST_SUITE(JointModelBase_test)
  
  struct TestJointModelIsEqual
  {
    template<typename JointModel>
    void operator()(const pinocchio::JointModelBase<JointModel> &) const
    {
      JointModel jmodel = init<JointModel>::run();
      
      test(jmodel);
    }
    
    template<typename JointModel>
    static void test(const JointModelBase<JointModel> & jmodel)
    {
      JointModel jmodel_copy = jmodel.derived();
      BOOST_CHECK(jmodel_copy == jmodel.derived());
      
      JointModel jmodel_any;
      BOOST_CHECK(jmodel_any != jmodel.derived());
      BOOST_CHECK(!jmodel_any.isEqual(jmodel.derived()));
    }
  };
  
  BOOST_AUTO_TEST_CASE(isEqual)
  {
    typedef JointCollectionDefault::JointModelVariant JointModelVariant;
    boost::mpl::for_each<JointModelVariant::types>(TestJointModelIsEqual());
    
    JointModelRX joint_revolutex;
    JointModelRY joint_revolutey;
    
    BOOST_CHECK(joint_revolutex != joint_revolutey);
    
    JointModel jmodelx(joint_revolutex);
    jmodelx.setIndexes(0,0,0);
    TestJointModelIsEqual()(JointModel());
    
    JointModel jmodel_any;
    BOOST_CHECK(jmodel_any != jmodelx);
  }
  
  template<typename TestDerived>
  struct TestJointModel
  {
    template<typename JointModel>
    void operator()(const pinocchio::JointModelBase<JointModel> &) const
    {
      JointModel jmodel;
      jmodel.setIndexes(0,0,0);
      
      test(jmodel);
    }
    
    template<typename Scalar, int Options>
    void operator()(const JointModelRevoluteUnalignedTpl<Scalar,Options> & ) const
    {
      typedef JointModelRevoluteUnalignedTpl<Scalar,Options> JointModel;
      typedef typename JointModel::Vector3 Vector3;
      JointModel jmodel(Vector3::Random().normalized());
      jmodel.setIndexes(0,0,0);
      
      test(jmodel);
    }
    
    template<typename Scalar, int Options>
    void operator()(const JointModelRevoluteUnboundedUnalignedTpl<Scalar,Options> & ) const
    {
      typedef JointModelRevoluteUnboundedUnalignedTpl<Scalar,Options> JointModel;
      typedef typename JointModel::Vector3 Vector3;
      JointModel jmodel(Vector3::Random().normalized());
      jmodel.setIndexes(0,0,0);
      
      test(jmodel);
    }
    
    template<typename Scalar, int Options>
    void operator()(const JointModelPrismaticUnalignedTpl<Scalar,Options> & ) const
    {
      typedef JointModelPrismaticUnalignedTpl<Scalar,Options> JointModel;
      typedef typename JointModel::Vector3 Vector3;
      JointModel jmodel(Vector3::Random().normalized());
      jmodel.setIndexes(0,0,0);
      
      test(jmodel);
    }
    
    template<typename Scalar, int Options, template<typename,int> class JointCollection>
    void operator()(const JointModelTpl<Scalar,Options,JointCollection> & ) const
    {
      typedef JointModelRevoluteTpl<Scalar,Options,0> JointModelRX;
      typedef JointModelTpl<Scalar,Options,JointCollection> JointModel;
      JointModel jmodel((JointModelRX()));
      jmodel.setIndexes(0,0,0);
      
      test(jmodel);
    }
    
    template<typename JointModel>
    static void test(const JointModelBase<JointModel> & jmodel)
    {
      return TestDerived::test(jmodel);
    }
  };

  struct TestJointModelCast : TestJointModel<TestJointModelCast>
  {
    template<typename JointModel>
    static void test(const JointModelBase<JointModel> & jmodel)
    {
      typedef typename JointModel::Scalar Scalar;
      BOOST_CHECK(jmodel.template cast<Scalar>() == jmodel);
      BOOST_CHECK(jmodel.template cast<long double>().template cast<double>() == jmodel);
    }
  };
  
  BOOST_AUTO_TEST_CASE(cast)
  {
    typedef JointCollectionDefault::JointModelVariant JointModelVariant;
    boost::mpl::for_each<JointModelVariant::types>(TestJointModelCast());
    
    TestJointModelCast()(JointModel());
  }

  struct TestJointModelDisp : TestJointModel<TestJointModelDisp>
  {
    template<typename JointModel>
    static void test(const JointModelBase<JointModel> & jmodel)
    {
      typedef typename JointModel::JointDataDerived JointData;
      
      std::cout << "shortname: " << jmodel.shortname() << std::endl;
      std::cout << "classname: " << jmodel.classname() << std::endl;
      std::cout << "disp:\n" << jmodel << std::endl;
      
      JointData jdata = jmodel.createData();
      
      std::cout << "shortname: " << jdata.shortname() << std::endl;
      std::cout << "classname: " << jdata.classname() << std::endl;
      std::cout << "disp:\n" << jdata << std::endl;
    }
  };

  BOOST_AUTO_TEST_CASE(test_disp)
  {
    typedef JointCollectionDefault::JointModelVariant JointModelVariant;
    boost::mpl::for_each<JointModelVariant::types>(TestJointModelDisp());
    
    TestJointModelDisp()(JointModel());
  }
  
BOOST_AUTO_TEST_SUITE_END()
  
