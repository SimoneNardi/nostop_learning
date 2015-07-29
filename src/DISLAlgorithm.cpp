#include "DISLAlgorithm.h"
#include "discretizedArea.h"
#include "guard.h"
#include "thief.h"
#include "sink.h"
#include "probability.h"
#include "coverageUtility.h"

#include <sstream>
#include <string>
#include <set>

using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;

//////////////////////////////////////////////////////////////////////////
DISLAlgorithm::DISLAlgorithm(std::shared_ptr<DiscretizedArea> _space) 
	: LearningAlgorithm(_space)
{}

//////////////////////////////////////////////////////////////////////////
void DISLAlgorithm::updateWithoutMoving(std::shared_ptr<Guard> _agent)
{
	if(!_agent->isRunning())
		//	Inizia una nuova traiettoria (sperimentale o no!)
	{
		//	ogni agente guardia sceglie il proprio tasso di esplorazione:
		double l_explorationRate = this->computeExplorationRate();

		//	ogni agente guardia estrae se sperimentare nuove azioni o no
		bool l_agentHasToExperiments = agentHasToExperiments(l_explorationRate);
		if(l_agentHasToExperiments)
		{
			_agent->startExperiment(l_explorationRate);
		}
		else
		{
			_agent->followBestTrajectory(l_explorationRate);
		}
	}

	_agent->selectNextAction(m_space);

	return;
}