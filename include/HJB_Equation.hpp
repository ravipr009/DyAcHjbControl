#ifndef HJB_EQUATION_HPP_
#define HJB_EQUATION_HPP_

#include <barrett/math/traits.h>
//#include <list>
#include <eigen3/Eigen/Core>
#include <Eigen/QR>
#include <barrett/units.h>
#include <barrett/detail/ca_macro.h>
#include <barrett/systems/abstract/system.h>
#include <cmath>
#include <complex>


#include <libconfig.h++>

#include <barrett/detail/ca_macro.h>
#include <barrett/math/traits.h>
#include <barrett/systems/abstract/execution_manager.h>
#include <barrett/systems/abstract/controller.h>
#include <samlibs.h>


#include <math.h>

using namespace barrett;
using namespace systems;


template<size_t DOF>
class HJB_Equation: public System {

	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	/*The HJB equations requires the following inputs
	 * Reference Cp
	 * Actual System Jp for feedback
	 * M matrix
	 * J matrix
	*/

	/*Output of this system is
	 * Jp
	*/
public:
//	Input<bool> Status;
	
public:
	Input<jp_type> feedbackjpInput;
	Input<cp_type> feedbackcpInput;
	cp_type DesiredCP;
public:
	Input<Eigen::Matrix4d> M;
	Input<Eigen::MatrixXd> J;
public:
	Output<jp_type> JointPositionOutput;

protected:
	typename Output<jp_type>::Value* JointPositionOutputValue;

public:
	HJB_Equation(/*systems::ExecutionManager* em*/
			const Eigen::VectorXd dersiredcp, const Eigen::VectorXd startpos,
			const std::string& sysName = "HJB_Equation"):
					System(sysName), 
					feedbackjpInput(this),
					feedbackcpInput(this),
					M(this),
					J(this),
					JointPositionOutput(this, &JointPositionOutputValue)
	{
		DesiredCP[0] = dersiredcp[0];
		DesiredCP[1] = dersiredcp[1];
		DesiredCP[2] = dersiredcp[2];

		StartPos[0] = startpos[0];
		StartPos[1] = startpos[1];
		StartPos[2] = startpos[2];
		StartPos[3] = startpos[3];
		
		//J_inside.resize(3,4);
	}
	virtual ~HJB_Equation()
	{
		this->mandatoryCleanUp();
	}

protected:
	
	jp_type jp_sys;
	jp_type jp_temp;
	cp_type cp_sys;
	cp_type cp_err;
	Eigen::Matrix4d M_inside;
	Eigen::MatrixXd J_inside;
	Eigen::Matrix4d M_inv;
	Eigen::Matrix3d C1;
	Eigen::Matrix3d C2;
	Eigen::EigenSolver<Eigen::MatrixXd> es;
	Eigen::Matrix3d V;
	Eigen::Matrix3d SIG;
	Eigen::Vector4d U;
	Eigen::Vector4d StartPos;

	virtual void operate() 
	{
		unsigned short int i = 0;
		unsigned short int j = 0;
		/*Taking feedback values from the input terminal of this system*/
		jp_sys = this->feedbackjpInput.getValue();
		cp_sys = this->feedbackcpInput.getValue();
		/*Taking M and C values from the input terminal of this system*/
		M_inside = this->M.getValue();
		J_inside = this->J.getValue();
/*
		cp_err = DesiredCP - cp_sys;
		M_inv = M_inside.inverse();

		C1 = J_inside*M_inv*J_inside.transpose();
		es.compute(C1);

		for(i=0;i<3;i++)
		{
			for(j=0;j<3;j++)
			{
				V(i,j) = es.eigenvectors()(i,j).real();
				if(i == j)
				{
					SIG(i,j) = 1/sqrt(es.eigenvalues()(i).real());
				}
				else
				{
					SIG(i,j) = 0.0;
				}
			}
		}

		C2 = V*SIG*V.transpose();
		U = M_inv*J_inside.transpose()*C2*cp_err;
		*/
		jp_temp[0] = StartPos[0];
		jp_temp[1] = StartPos[1];
		jp_temp[2] = StartPos[2];
		jp_temp[3] = StartPos[3];

		JointPositionOutputValue->setData(&jp_temp);
	}

private:
	DISALLOW_COPY_AND_ASSIGN(HJB_Equation);
};

#endif /* HJB_EQUATION_HPP_ */
