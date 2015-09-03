#pragma once

#include "Real2D.h"
#include "shape2D.h"

#include <vector>

namespace Robotics
{
	namespace GameTheory
	{
		class Polygon : public Shape2D
		{
			std::vector<Real2D> m_points;

		public:
			Polygon() : Shape2D(), m_points() {}

			Polygon(std::vector<Real2D> const& _points);

			~Polygon();

			virtual bool isValid() const {return !m_points.empty();}

			virtual bool contains(Real3D const& point) const;
			virtual bool contains(Real2D const& point) const;

			virtual std::vector<Real2D> getBoundaryPoints() const {return m_points;}
		};
	}
}