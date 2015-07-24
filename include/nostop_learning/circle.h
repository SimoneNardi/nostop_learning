#pragma once

#include "IDSReal2D.h"

#include "shape2D.h"

#include <vector>

namespace Robotics
{
	namespace GameTheory
	{
		class Circle : public Shape2D
		{
		public:
			IDSReal2D m_center;
			double m_radius;

			Circle() : m_center(), m_radius(-1) {}

			Circle(IDSReal2D const& _center, double radius_);

			~Circle();

			inline bool isValid() const {return !(m_radius<0.);}

			bool contains(IDSReal3D const& point) const;
			bool contains(IDSReal2D const& point) const;

			std::vector<IDSReal2D> getBoundaryPoints() const {return getCircumPolygon();}
		protected:
			std::vector<IDSReal2D> getCircumPolygon() const;
		};
	}
}