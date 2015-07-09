///////////////////////////////////////////////////////////
//  Thief.h
//  Created on:      13-may-2013 10.14.52
//  Original author: s.nardi
///////////////////////////////////////////////////////////
#ifndef THIEF_H
#define THIEF_H
#pragma once

#include "agent.h"

namespace Robotics 
{
	namespace GameTheory 
	{
		class Thief : public Agent
		{

		public:
			Thief(int _id, AgentPosition _position ) : Agent(_id, _position) {}

			~Thief() {}

			virtual bool isThief() const {return true;}
		};

		typedef std::shared_ptr<Thief> ThiefPtr;
	}
}
#endif