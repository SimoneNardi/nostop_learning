///////////////////////////////////////////////////////////
//  Neutral.h
//  Created on:      13-may-2013 10.14.52
//  Original author: s.nardi
///////////////////////////////////////////////////////////
#ifndef NEUTRAL_H
#define NEUTRAL_H
#pragma once

#include "agent.h"

namespace Robotics 
{
	namespace GameTheory 
	{
		class Neutral : public Agent
		{
		public:
			Neutral(int _id, AgentPosition _position ) : Agent(_id, _position) {}

			~Neutral() {}

			virtual bool isNeutral() const {return true;}
		};

		typedef std::shared_ptr<Neutral> NeutralPtr;
	}
}
#endif