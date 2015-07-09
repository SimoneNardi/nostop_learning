#pragma once

#include "IDSReal2D.h"
#include "shape2D.h"

#include <vector>

namespace Robotics
{
	namespace GameTheory
	{
		class Polygon : public Shape2D
		{
			std::vector<IDSReal2D> m_points;

		public:
			Polygon() : Shape2D(), m_points() {}

			Polygon(std::vector<IDSReal2D> const& _points);

			~Polygon();

			virtual bool isValid() const {return !m_points.empty();}

			virtual bool contains(IDSReal3D const& point) const;
			virtual bool contains(IDSReal2D const& point) const;

			virtual std::vector<IDSReal2D> getBoundaryPoints() const {return m_points;}
		};
	}
}