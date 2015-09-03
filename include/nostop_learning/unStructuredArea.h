///////////////////////////////////////////////////////////
//  UnStructuredArea.h
//  Created on:      13-may-2013 10.14.52
//  Original author: s.nardi
///////////////////////////////////////////////////////////
#ifndef UNSTRUCTURED_AREA_H
#define UNSTRUCTURED_AREA_H
#pragma once

#include "area.h"

#include <memory>

namespace Robotics 
{
	namespace GameTheory 
	{
		class UnStructuredArea: public Area
		{
			/// Discretize the Area
			virtual std::shared_ptr<DiscretizedArea> discretize();

			/// Get a point inside the area
			virtual Real2D randomPosition() const;

			/// True if a corner or the center is inside the area, False otherwise
			virtual bool isInside( Box const& _box) const {return true;}

			double getDistance() const;
		};
	}
}
#endif