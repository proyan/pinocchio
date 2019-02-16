//
// Copyright (c) 2015-2018 CNRS
//

#ifndef __pinocchio_python_data_hpp__
#define __pinocchio_python_data_hpp__

#include "pinocchio/multibody/data.hpp"

#include <eigenpy/memory.hpp>
#include <eigenpy/eigenpy.hpp>
#include "pinocchio/bindings/python/utils/std-vector.hpp"
#include "pinocchio/bindings/python/utils/std-aligned-vector.hpp"
#include <boost/python/tuple.hpp>
#include <boost/python/list.hpp>

EIGENPY_DEFINE_STRUCT_ALLOCATOR_SPECIALIZATION(pinocchio::Data)

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;

    struct DataPythonVisitor
      : public boost::python::def_visitor< DataPythonVisitor >
    {
      typedef Data::Matrix6x Matrix6x;
      typedef Data::Matrix3x Matrix3x;
      typedef Data::Vector3 Vector3;

    public:

#define ADD_DATA_PROPERTY(TYPE,NAME,DOC)				 \
      def_readwrite(#NAME,						 \
      &Data::NAME,		 \
      DOC)
      
#define ADD_DATA_PROPERTY_READONLY(TYPE,NAME,DOC)				 \
      def_readonly(#NAME,						 \
      &Data::NAME,		 \
      DOC)
      
#define ADD_DATA_PROPERTY_READONLY_BYVALUE(TYPE,NAME,DOC)				 \
      add_property(#NAME,						 \
      make_getter(&Data::NAME,bp::return_value_policy<bp::return_by_value>()), \
      DOC)
      
	 
      /* --- Exposing C++ API to python through the handler ----------------- */
      template<class PyClass>
      void visit(PyClass& cl) const 
      {
        cl
        .def(bp::init<Model>(bp::arg("model"),"Constructs a data structure from a given model."))     
        .ADD_DATA_PROPERTY(container::aligned_vector<Motion>,a,"Joint spatial acceleration")
        .ADD_DATA_PROPERTY(container::aligned_vector<Motion>,a_gf,"Joint spatial acceleration containing also the contribution of the gravity acceleration")
        .ADD_DATA_PROPERTY(container::aligned_vector<Motion>,v,"Joint spatial velocity expressed in the joint frame.")
        .ADD_DATA_PROPERTY(container::aligned_vector<Force>,f,"Joint spatial force expresssed in the joint frame.")
        .ADD_DATA_PROPERTY(container::aligned_vector<SE3>,oMi,"Body absolute placement (wrt world)")
        .ADD_DATA_PROPERTY(container::aligned_vector<SE3>,oMf,"frames absolute placement (wrt world)")
        .ADD_DATA_PROPERTY(container::aligned_vector<SE3>,liMi,"Body relative placement (wrt parent)")
        .ADD_DATA_PROPERTY_READONLY_BYVALUE(Eigen::VectorXd,tau,"Joint torques (output of RNEA)")
        .ADD_DATA_PROPERTY_READONLY_BYVALUE(Eigen::VectorXd,nle,"Non Linear Effects (output of nle algorithm)")
        .ADD_DATA_PROPERTY_READONLY_BYVALUE(Eigen::VectorXd,ddq,"Joint accelerations (output of ABA)")
        .ADD_DATA_PROPERTY(container::aligned_vector<Inertia>,Ycrb,"Inertia of the sub-tree composit rigid body")
        .ADD_DATA_PROPERTY_READONLY_BYVALUE(Eigen::MatrixXd,M,"The joint space inertia matrix")
        .ADD_DATA_PROPERTY_READONLY_BYVALUE(Data::RowMatrixXs,Minv,"The inverse of the joint space inertia matrix")
        .ADD_DATA_PROPERTY_READONLY_BYVALUE(Eigen::MatrixXd,C,"The Coriolis C(q,v) matrix such that the Coriolis effects are given by c(q,v) = C(q,v)v")
        .ADD_DATA_PROPERTY(container::aligned_vector<Matrix6x>,Fcrb,"Spatial forces set, used in CRBA")
        .ADD_DATA_PROPERTY(std::vector<int>,lastChild,"Index of the last child (for CRBA)")
        .ADD_DATA_PROPERTY(std::vector<int>,nvSubtree,"Dimension of the subtree motion space (for CRBA)")
        .ADD_DATA_PROPERTY_READONLY_BYVALUE(Eigen::MatrixXd,U,"Joint Inertia square root (upper triangle)")
        .ADD_DATA_PROPERTY_READONLY_BYVALUE(Eigen::VectorXd,D,"Diagonal of UDUT inertia decomposition")
        .ADD_DATA_PROPERTY(std::vector<int>,parents_fromRow,"First previous non-zero row in M (used in Cholesky)")
        .ADD_DATA_PROPERTY(std::vector<int>,nvSubtree_fromRow,"Subtree of the current row index (used in Cholesky)")
        .ADD_DATA_PROPERTY_READONLY_BYVALUE(Matrix6x,J,"Jacobian of joint placement")
        .ADD_DATA_PROPERTY_READONLY_BYVALUE(Matrix6x,dJ,"Time variation of the Jacobian of joint placement (data.J).")
        .ADD_DATA_PROPERTY(container::aligned_vector<SE3>,iMf,"Body placement wrt to algorithm end effector.")
        
        .ADD_DATA_PROPERTY_READONLY_BYVALUE(Matrix6x,Ag,
                                            "Centroidal matrix which maps from joint velocity to the centroidal momentum.")
        .ADD_DATA_PROPERTY_READONLY_BYVALUE(Matrix6x,dAg,
                                            "Time derivative of the centroidal momentum matrix Ag.")
        .ADD_DATA_PROPERTY_READONLY(Force,hg,
                                    "Centroidal momentum (expressed in the frame centered at the CoM and aligned with the inertial frame).")
        .ADD_DATA_PROPERTY_READONLY(Inertia,Ig,
                                    "Centroidal Composite Rigid Body Inertia.")
        
        .ADD_DATA_PROPERTY(container::aligned_vector<Vector3>,com,"CoM position of the subtree starting at joint index i.")
        .ADD_DATA_PROPERTY(container::aligned_vector<Vector3>,vcom,"CoM velocity of the subtree starting at joint index i.")
        .ADD_DATA_PROPERTY(container::aligned_vector<Vector3>,acom,"CoM acceleration of the subtree starting at joint index i..")
        .ADD_DATA_PROPERTY(std::vector<double>,mass,"Mass of the subtree starting at joint index i.")
        .ADD_DATA_PROPERTY_READONLY_BYVALUE(Matrix3x,Jcom,"Jacobian of center of mass.")

        .ADD_DATA_PROPERTY_READONLY_BYVALUE(Eigen::MatrixXd,C,"Joint space Coriolis matrix.")
        .ADD_DATA_PROPERTY_READONLY_BYVALUE(Eigen::MatrixXd,dtau_dq,"Partial derivative of the joint torque vector with respect to the joint configuration.")
        .ADD_DATA_PROPERTY_READONLY_BYVALUE(Eigen::MatrixXd,dtau_dv,"Partial derivative of the joint torque vector with respect to the joint velocity.")
        .ADD_DATA_PROPERTY_READONLY_BYVALUE(Eigen::MatrixXd,ddq_dq,"Partial derivative of the joint acceleration vector with respect to the joint configuration.")
        .ADD_DATA_PROPERTY_READONLY_BYVALUE(Eigen::MatrixXd,ddq_dv,"Partial derivative of the joint acceleration vector with respect to the joint velocity.")
        
        .ADD_DATA_PROPERTY_READONLY_BYVALUE(double,kinetic_energy,"Kinetic energy in [J] computed by kineticEnergy(model,data,q,v,True/False)")
        .ADD_DATA_PROPERTY_READONLY_BYVALUE(double,potential_energy,"Potential energy in [J] computed by potentialEnergy(model,data,q,True/False)")
        
        .ADD_DATA_PROPERTY_READONLY_BYVALUE(Eigen::VectorXd,lambda_c,"Lagrange Multipliers linked to contact forces")
        .ADD_DATA_PROPERTY_READONLY_BYVALUE(Eigen::VectorXd,impulse_c,"Lagrange Multipliers linked to contact impulses")
        
        .ADD_DATA_PROPERTY_READONLY_BYVALUE(Eigen::VectorXd,dq_after,"Generalized velocity after the impact.")
        ;
      }

      /* --- Expose --------------------------------------------------------- */
      static void expose()
      {
        bp::class_<Data>("Data",
                         "Articulated rigid body data.\n"
                         "It contains all the data that can be modified by the algorithms.",
                         bp::no_init)
        .def("__init__", +[](DataPythonVisitor){
            return bp::make_tuple();})
        .def(DataPythonVisitor())
        .def_pickle(Pickle());
        StdAlignedVectorPythonVisitor<Vector3, true>::expose("StdVec_vec3d");
        StdAlignedVectorPythonVisitor<Matrix6x, true>::expose("StdMat_Matrix6x");
        StdVectorPythonVisitor<int>::expose("StdVec_int");
        
        eigenpy::enableEigenPySpecific<Data::RowMatrixXs>();
      }

    private:

      struct Pickle : bp::pickle_suite
      { 
        //static bp::tuple getinitargs(const Model&) {    return bp::make_tuple();      }
        static bp::tuple getstate(bp::object op)
        {
          const Data& d = bp::extract<const Data&>(op)();
          bp::list lt;
          lt.append(d.a);
          lt.append(d.a_gf);
          lt.append(d.v);
          lt.append(d.f);
          lt.append(d.oMi);
          lt.append(d.oMf);
          lt.append(d.liMi);
          lt.append(d.tau);
          lt.append(d.nle);
          lt.append(d.ddq);
          lt.append(d.Ycrb);
          lt.append(d.M);
          lt.append(d.Minv);
          lt.append(d.C);
          lt.append(d.Fcrb);
          lt.append(d.lastChild);
          lt.append(d.nvSubtree);
          lt.append(d.U);
          lt.append(d.D);
          lt.append(d.parents_fromRow);
          lt.append(d.nvSubtree_fromRow);
          lt.append(d.J);
          lt.append(d.dJ);
          lt.append(d.iMf);
          lt.append(d.Ag);
          lt.append(d.dAg);
          lt.append(d.hg);
          lt.append(d.Ig);
          lt.append(d.com);
          lt.append(d.vcom);
          lt.append(d.acom);
          lt.append(d.mass);
          lt.append(d.Jcom);
          lt.append(d.C);
          lt.append(d.dtau_dq);
          lt.append(d.dtau_dv);
          lt.append(d.ddq_dq);
          lt.append(d.ddq_dv);
          lt.append(d.kinetic_energy);
          lt.append(d.potential_energy);
          lt.append(d.lambda_c);
          lt.append(d.impulse_c);
          lt.append(d.dq_after);
          return bp::make_tuple(lt);
        }
        static void setstate(bp::object op, bp::tuple tup)
        {
          Data& d = bp::extract<Data&>(op)();
          d.a = bp::extract<container::aligned_vector<Motion> >(tup[0]);
          d.a_gf = bp::extract<container::aligned_vector<Motion> >(tup[1]);
          d.v = bp::extract<container::aligned_vector<Motion> >(tup[2]);
          d.f = bp::extract<container::aligned_vector<Force> >(tup[3]);
          d.oMi = bp::extract<container::aligned_vector<SE3> >(tup[4]);
          d.oMf = bp::extract<container::aligned_vector<SE3> >(tup[5]);
          d.liMi = bp::extract<container::aligned_vector<SE3> >(tup[6]);
          d.tau = bp::extract<Eigen::VectorXd >(tup[7]);
          d.nle = bp::extract<Eigen::VectorXd >(tup[8]);
          d.ddq = bp::extract<Eigen::VectorXd >(tup[9]);
          d.Ycrb = bp::extract<container::aligned_vector<Inertia> >(tup[10]);
          d.M = bp::extract<Eigen::MatrixXd >(tup[11]);
          d.Minv = bp::extract<Data::RowMatrixXs >(tup[12]);
          d.C = bp::extract<Eigen::MatrixXd >(tup[13]);
          d.Fcrb = bp::extract<container::aligned_vector<Matrix6x> >(tup[14]);
          d.lastChild = bp::extract<std::vector<int> >(tup[15]);
          d.nvSubtree = bp::extract<std::vector<int> >(tup[16]);
          d.U = bp::extract<Eigen::MatrixXd >(tup[17]);
          d.D = bp::extract<Eigen::VectorXd >(tup[18]);
          d.parents_fromRow = bp::extract<std::vector<int> >(tup[19]);
          d.nvSubtree_fromRow = bp::extract<std::vector<int> >(tup[20]);
          d.J = bp::extract<Matrix6x >(tup[21]);
          d.dJ = bp::extract<Matrix6x >(tup[22]);
          d.iMf = bp::extract<container::aligned_vector<SE3> >(tup[23]);
          d.Ag = bp::extract<Matrix6x >(tup[24]);
          d.dAg = bp::extract<Matrix6x >(tup[25]);
          d.hg = bp::extract<Force >(tup[26]);
          d.Ig = bp::extract<Inertia >(tup[27]);
          d.com = bp::extract<container::aligned_vector<Vector3> >(tup[28]);
          d.vcom = bp::extract<container::aligned_vector<Vector3> >(tup[29]);
          d.acom = bp::extract<container::aligned_vector<Vector3> >(tup[30]);
          d.mass = bp::extract<std::vector<double> >(tup[31]);
          d.Jcom = bp::extract<Matrix3x >(tup[32]);
          d.C = bp::extract<Eigen::MatrixXd >(tup[33]);
          d.dtau_dq = bp::extract<Eigen::MatrixXd >(tup[34]);
          d.dtau_dv = bp::extract<Eigen::MatrixXd >(tup[35]);
          d.ddq_dq = bp::extract<Eigen::MatrixXd >(tup[36]);
          d.ddq_dv = bp::extract<Eigen::MatrixXd >(tup[37]);
          d.kinetic_energy = bp::extract<double >(tup[38]);
          d.potential_energy = bp::extract<double >(tup[39]);
          d.lambda_c = bp::extract<Eigen::VectorXd >(tup[40]);
          d.impulse_c = bp::extract<Eigen::VectorXd >(tup[41]);
          d.dq_after = bp::extract<Eigen::VectorXd >(tup[42]);

        }
      };
    };
  }} // namespace pinocchio::python

#endif // ifndef __pinocchio_python_data_hpp__

