#include "CoarseCorrelatedAlgorithm.h"
#include "discretizedArea.h"
#include "guard.h"
#include "thief.h"
#include "probability.h"
#include "coverageUtility.h"

#include <sstream>
#include <string>
#include <set>

using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;

//////////////////////////////////////////////////////////////////////////
CoarseCorrelatedAlgorithm::CoarseCorrelatedAlgorithm(std::shared_ptr<DiscretizedArea> _space) 
	: LearningAlgorithm(_space)
{}

//////////////////////////////////////////////////////////////////////////
void CoarseCorrelatedAlgorithm::updateWithoutMoving(std::shared_ptr<Guard> _agent)
{
	if(!_agent->isRunning())
		//	Inizia una nuova traiettoria (sperimentale o no!)
	{
		//	ogni agente guardia sceglie il proprio tasso di esplorazione:
		double l_explorationRate = this->computeExplorationRate();

		double l_maxValue = m_guards.size();

		switch(_agent->getCurrentMood())
		{
		case Content:
			{
				double l_powExplorationRate = pow( l_explorationRate, l_maxValue );

				//	ogni agente guardia estrae se sperimentare nuove azioni o no
				bool l_agentHasToExperiments = agentHasToExperiments(l_powExplorationRate);
				if(l_agentHasToExperiments)
				{
					_agent->startExperiment(l_explorationRate);
				}
				else
				{
					_agent->followBestTrajectory(l_explorationRate);
				}
				break;
			}
		case Discontent:
		default:
			{
				_agent->startExperiment(l_explorationRate);
				break;
			}
		}
	}

	_agent->selectNextAction(m_space);

	return;
}