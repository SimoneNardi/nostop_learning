#pragma once

#include "Real2D.h"

namespace Robotics
{
	namespace GameTheory
	{
		class Line2D
		{
		public:
			Real2D m_origin;
			Real2D m_direction;

		public:
			Line2D(Real2D const & m_origin, Real2D const &m_end);

			double parameterAt(Real2D const & point_) const;
			Real2D projectPoint(Real2D const & point_) const;
			Real2D pointFromOrigin(double dist_) const;

		};
	}
}