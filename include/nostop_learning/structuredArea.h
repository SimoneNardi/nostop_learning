///////////////////////////////////////////////////////////
//  StructuredArea.h
//  Created on:      13-may-2013 10.14.52
//  Original author: s.nardi
///////////////////////////////////////////////////////////
#ifndef STRUCTURED_AREA_H
#define STRUCTURED_AREA_H
#pragma once

#include "area.h"

#include "shape2D.h"
#include "Real2D.h"

#include <memory>
#include <set>

namespace Robotics 
{
	namespace GameTheory 
	{
		class DiscretizedArea;

		class StructuredArea: public Area
		{
			std::shared_ptr<Shape2D> m_external;

			std::set<std::shared_ptr<Shape2D> > m_obstacles;
		public:
			/// Create an area without obstacles
			StructuredArea(std::vector<Real2D> const& points);
			
			/// Create an area with obstacles
			StructuredArea(std::vector<Real2D> const& points, std::vector< std::vector<Real2D> > const& obstacles);

			/// Discretize the Area
			virtual std::shared_ptr<DiscretizedArea> discretize();

			/// Get a point inside the area
			virtual Real2D randomPosition() const;

			/// True if a corner or the center is inside the area, False otherwise
			virtual bool isInside( Box const& _box) const;

			Box getBoundingBox() const;

			/// 
			double getDistance() const;
		};
	}
}
#endif