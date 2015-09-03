#pragma once

#include "Box.h"
#include <vector>

class Real2D;
class Real3D;

namespace Robotics
{
	namespace GameTheory
	{
		class Shape2D
		{
		protected:
			Box m_box;
		public:
			Shape2D();

			~Shape2D();

			virtual bool isValid() const = 0;

			virtual bool contains(Real3D const& point) const = 0;
			virtual bool contains(Real2D const& point) const = 0;

			Box getBoundingBox() const {return m_box;}

			virtual std::vector<Real2D> getBoundaryPoints() const = 0;

		};
	}
}