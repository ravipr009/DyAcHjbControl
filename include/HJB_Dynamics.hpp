#ifndef HJB_DYNAMICS_H_
#define HJB_DYNAMICS_H_

#include <barrett/math/traits.h>
#include <list>
#include <barrett/units.h>
#include <barrett/detail/ca_macro.h>
#include <barrett/systems/abstract/system.h>

#include <eigen3/Eigen/Core>
#include <libconfig.h++>

#include <barrett/detail/ca_macro.h>
#include <barrett/math/traits.h>
#include <barrett/systems/abstract/execution_manager.h>
#include <barrett/systems/abstract/controller.h>
#include <samlibs.h>

#include <M_4D.hpp>
#include <J_3x4D.hpp>

using namespace barrett;
using namespace systems;

template<size_t DOF>
class HJB_Dynamics: public System {

	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	/* Torque*/
public:
	Input<jp_type> jpInputDynamics;
public:
	Output<Eigen::Matrix4d> MassMAtrixOutput;
	Output<Eigen::MatrixXd> JacobianMatrixOutput;
protected:
	typename Output<Eigen::Matrix4d>::Value* MassMAtrixOutputValue;
	typename Output<Eigen::MatrixXd>::Value* JacobianMatrixOutputValue;

public:
	HJB_Dynamics(/*systems::ExecutionManager* em*/) : 	jpInputDynamics(this), 
							MassMAtrixOutput(this, &MassMAtrixOutputValue),
							JacobianMatrixOutput(this, &JacobianMatrixOutputValue) 
	{
		jacobian.resize(3,4);
//		if (em != NULL){
//		      em->startManaging(*this);
//		    }
	}

	virtual ~HJB_Dynamics() 
	{
		this->mandatoryCleanUp();
	}

protected:
	jp_type tmp_theta_pos;
	
	Eigen::Matrix4d massMatrix;
	Eigen::MatrixXd jacobian;
	Eigen::Vector4d ThetaInput;

	virtual void operate() 
	{
		tmp_theta_pos = this->jpInputDynamics.getValue();
		ThetaInput << tmp_theta_pos[0], tmp_theta_pos[1], tmp_theta_pos[2], tmp_theta_pos[3];
		massMatrix = M_4D(ThetaInput);
		jacobian = J_3x4D(ThetaInput);

		this->MassMAtrixOutputValue->setData(&massMatrix);
		this->JacobianMatrixOutputValue->setData(&jacobian);
	}

};
/* namespace Sam */
#endif /* HJB_DYNAMICS_H_ */
