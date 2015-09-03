///////////////////////////////////////////////////////////
//  Area.h
//  Created on:      13-may-2013 10.14.52
//  Original author: s.nardi
///////////////////////////////////////////////////////////
#ifndef AREA_H
#define AREA_H
#pragma once

#include "shape2D.h"
#include "Box.h"

#include <memory>

namespace Robotics 
{
	namespace GameTheory 
	{
		class DiscretizedArea;
		class AgentPosition;

		class Area : public std::enable_shared_from_this<Area>
		{
			// TODO:

		public:
			virtual std::shared_ptr<DiscretizedArea> discretize() = 0;

			virtual Real2D randomPosition() const = 0;

			virtual bool isInside( Box const& _box) const = 0;

			virtual double getDistance() const = 0;
		};

		typedef std::shared_ptr<Area> AreaPtr;
	}
}
#endif