///////////////////////////////////////////////////////////
//  PIPIPAlgorithm.h
//  Created on:      13-may-2013 10.14.52
//  Original author: s.nardi
///////////////////////////////////////////////////////////
#ifndef PIPIP_ALGORITHM_H
#define PIPIP_ALGORITHM_H
#pragma once

//	Coverage
#include "learningAlgorithm.h"

#include <memory>
#include <set>

namespace Robotics 
{
	namespace GameTheory 
	{
		class Guard;
		class DiscretizedArea;
		class Thief;

		/**
		*	\brief	Inhomogeneous Synchronous Learning Algorithm
		*	see M.Zhu and S.Martinez, 
		*	Distributed coverage game for mobile visual sensors (I): Reaching the set of Nash equilibria.
		*	in Proceedings of the 48th IEEE Conference on Decision and Control.
		*/
		class PIPIPAlgorithm : public LearningAlgorithm
		{
		public:
			PIPIPAlgorithm(std::shared_ptr<DiscretizedArea> _space);

		protected:
			virtual void updateWithoutMoving(std::shared_ptr<Guard> _agent);

			double computeIrrationalRate();

			double computeDeltaMemoryBenefit(std::shared_ptr<Guard> _agent);
		};
	}
}
#endif