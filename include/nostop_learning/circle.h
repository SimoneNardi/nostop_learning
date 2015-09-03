#pragma once

#include "Real2D.h"

#include "shape2D.h"

#include <vector>

namespace Robotics
{
	namespace GameTheory
	{
		class Circle : public Shape2D
		{
		public:
			Real2D m_center;
			double m_radius;

			Circle() : m_center(), m_radius(-1) {}

			Circle(Real2D const& _center, double radius_);

			~Circle();

			inline bool isValid() const {return !(m_radius<0.);}

			bool contains(Real3D const& point) const;
			bool contains(Real2D const& point) const;

			std::vector<Real2D> getBoundaryPoints() const {return getCircumPolygon();}
		protected:
			std::vector<Real2D> getCircumPolygon() const;
		};
	}
}