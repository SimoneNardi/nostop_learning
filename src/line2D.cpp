#include "line2D.h"

using namespace Robotics;
using namespace Robotics::GameTheory;

Line2D::Line2D(IDSReal2D const & origin_, IDSReal2D const &end_) 
	: m_origin(origin_), m_direction()
{
	m_direction = (end_ - origin_).vers();
}

double Line2D::parameterAt(IDSReal2D const & point_) const
{
	IDSReal2D l_ext = point_- m_origin;
	return l_ext * m_direction;
}

IDSReal2D Line2D::projectPoint(IDSReal2D const & point_) const
{
	IDSReal2D l_ext = point_- m_origin;
	double l_param = l_ext * m_direction;

	return m_origin + l_param * m_direction;
}

IDSReal2D Line2D::pointFromOrigin(double dist_) const
{
	return m_origin + dist_ * m_direction;
}