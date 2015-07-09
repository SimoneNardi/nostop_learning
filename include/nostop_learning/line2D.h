#pragma once

#include "IDSReal2D.h"

namespace Robotics
{
	namespace GameTheory
	{
		class Line2D
		{
		public:
			IDSReal2D m_origin;
			IDSReal2D m_direction;

		public:
			Line2D(IDSReal2D const & m_origin, IDSReal2D const &m_end);

			double parameterAt(IDSReal2D const & point_) const;
			IDSReal2D projectPoint(IDSReal2D const & point_) const;
			IDSReal2D pointFromOrigin(double dist_) const;

		};
	}
}