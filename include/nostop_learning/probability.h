#pragma once

namespace Robotics 
{
	namespace GameTheory 
	{
		/// Function to get value with a specified distribution.
		int getRandomValue();

		/// Return value between 0 and maxValue-1
		int getRandomValue(int maxValue);

		bool agentHasToExperiments(double _explorationRate);
	}
}
