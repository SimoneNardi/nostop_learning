#pragma once

#include "IDSBox.h"
#include <vector>

class IDSReal2D;
class IDSReal3D;

namespace Robotics
{
	namespace GameTheory
	{
		class Shape2D
		{
		protected:
			IDSBox m_box;
		public:
			Shape2D();

			~Shape2D();

			virtual bool isValid() const = 0;

			virtual bool contains(IDSReal3D const& point) const = 0;
			virtual bool contains(IDSReal2D const& point) const = 0;

			IDSBox getBoundingBox() const {return m_box;}

			virtual std::vector<IDSReal2D> getBoundaryPoints() const = 0;

		};
	}
}