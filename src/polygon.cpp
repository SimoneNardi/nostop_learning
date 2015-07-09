#include "polygon.h"
#include "shape2D.h"

using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;

/////////////////////////////////////////////
Polygon::Polygon(std::vector<IDSReal2D> const& points_)
	: Shape2D()
	, m_points(points_)
{
	for(size_t i = 0; i < m_points.size(); ++i)
	{
		m_box.extend(m_points[i]);
	}
}

/////////////////////////////////////////////
Polygon::~Polygon()
{}

/////////////////////////////////////////////
bool Polygon::contains(IDSReal3D const& point) const
{
	IDSReal2D l_point(point(0), point(1));
	return this->contains(l_point);
}

/////////////////////////////////////////////
bool Polygon::contains(IDSReal2D const& point) const
{
	return point.belongsToPolygon(m_points);
}