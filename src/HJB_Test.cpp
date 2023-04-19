

#include <unistd.h>
#include <iostream>
#include <string>
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>
#include <barrett/detail/stl_utils.h>
#include <barrett/log.h>
#include <barrett/standard_main_function.h>

//#include <boost/thread.hpp>
//#include <barrett/thread/null_mutex.h>

using namespace barrett;
using detail::waitForEnter;
#include <samlibs.h>


#include <HJB_Equation.hpp>
#include <HJB_Dynamics.hpp>
#include <tool_details.hpp>

template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm,
		systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	Eigen::VectorXd StartPos;
	Eigen::VectorXd CPDesired;


	tool_details<DOF> tool_values(wam);

	Sam::initEigenVec<double>(StartPos, Sam::readFile<double>("start.txt"));
	Sam::initEigenVec<double>(CPDesired, Sam::readFile<double>("desired.txt"));

	typedef boost::tuple<double, jp_type, jp_type, cp_type> tuple_type;
	typedef systems::TupleGrouper<double, jp_type, jp_type, cp_type> tg_type;
	tg_type tg;
	
	char tmpFile[] = "btXXXXXX";
	if (mkstemp(tmpFile) == -1)
	{
		printf("ERROR: Couldn't create temporary file!\n");
		return 1;
	}
	
	const double TRANSITION_DURATION = 0.5;
	
	jp_type startpos(0.0);

	HJB_Equation<DOF> HJB(CPDesired, StartPos);
	HJB_Dynamics<DOF> HJB_dynamics;

	startpos[0] = StartPos[0];
	startpos[1] = StartPos[1];
	startpos[2] = StartPos[2];
	startpos[3] = StartPos[3];

	

	wam.gravityCompensate();
	printf("Press [Enter] to turn on torque control to go to zero position");
	waitForEnter();

	wam.moveTo(startpos);
	printf("Press [Enter] to START...");
	waitForEnter();
	printf("Error 1 \n");

	systems::Ramp time(pm.getExecutionManager(), 1.0);
	const size_t PERIOD_MULTIPLIER = 1;
	systems::PeriodicDataLogger<tuple_type> logger(pm.getExecutionManager(),
							new log::RealTimeWriter<tuple_type>(tmpFile,
							PERIOD_MULTIPLIER * pm.getExecutionManager()->getPeriod()),
							PERIOD_MULTIPLIER);
	printf("Error 2 \n");



//--------------------------------------------------
// CONNECTION CODE
//--------------------------------------------------
	systems::connect(tg.output, logger.input);

	systems::connect(wam.jpOutput, HJB_dynamics.jpInputDynamics);

	systems::connect(HJB_dynamics.MassMAtrixOutput, HJB.M);
	systems::connect(HJB_dynamics.JacobianMatrixOutput, HJB.J);
	systems::connect(wam.jpOutput, HJB.feedbackjpInput);
	systems::connect(tool_values.Cartesian_pos_value, HJB.feedbackcpInput);

	
	wam.trackReferenceSignal(HJB.JointPositionOutput);
	printf("Error 3 \n");



//--------------------------------------------------
// LOGGER CODE
//--------------------------------------------------
	systems::connect(time.output, tg.template getInput<0>());
	systems::connect(HJB.JointPositionOutput, tg.template getInput<1>());
	systems::connect(wam.jpOutput, tg.template getInput<2>());
	systems::connect(tool_values.Cartesian_pos_value, tg.template getInput<3>());
	printf("Error 4 \n");



	time.smoothStart(TRANSITION_DURATION);
	printf("Press [Enter] to stop.");
	waitForEnter();
	logger.closeLog();
	time.smoothStop(TRANSITION_DURATION);
	wam.idle();
	printf("Error 5 \n");

	pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
	log::Reader<boost::tuple<tuple_type> > lr(tmpFile);
	lr.exportCSV(argv[1]);
	printf("Error 6 \n");

	printf("Output written to %s.\n", argv[1]);
	std::remove(tmpFile);

	return 0;
}

