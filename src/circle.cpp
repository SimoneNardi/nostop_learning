#include "circle.h"
#include "shape2D.h"

using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;

/////////////////////////////////////////////
Circle::Circle(Real2D const& center_, double radius_) 
	: Shape2D()
	, m_center(center_)
	, m_radius(radius_)
{
	vector<Real2D> l_points = this->getCircumPolygon();

	for(size_t i = 0; i < l_points.size(); ++i)
		m_box.extend(l_points[i]);
}

/////////////////////////////////////////////
Circle::~Circle()
{}

/////////////////////////////////////////////
vector<Real2D> Circle::getCircumPolygon() const
{
	// Compute a good number of arc subdivisions
	double l_sweepAngle = Math::TwoPi;
	double l_startAngle = 0;

	int num_arcs = (int)ceil(fabs(l_sweepAngle) / (Math::PiDiv4));
	if (num_arcs < 1) num_arcs = 1;
	double deltaAngle = l_sweepAngle / (num_arcs * 2);
	double circumRadius = 1. / cos(deltaAngle);

	// Initialize the BB with the arc's endpoints
	vector<Real2D> l_points;

	double phi0 = l_startAngle;
	Real2D p0 = Real2D(cos(phi0)*m_radius, sin(phi0)*m_radius);
	l_points.push_back(m_center + p0);

	phi0 += l_sweepAngle;
	p0 = Real2D(cos(phi0)*m_radius, sin(phi0)*m_radius);
	l_points.push_back(m_center + p0);

	// Compute the polygon's vertices
	for (int i = 0; i < num_arcs; ++i) 
	{
		double phi = l_startAngle + (2*i+1)*deltaAngle;
		Real2D p = circumRadius * Real2D(cos(phi), sin(phi));
		p[0] *= m_radius;
		p[1] *= m_radius;
		p = m_center + p;
		l_points.push_back(p);
	}

	return l_points;
}

/////////////////////////////////////////////
bool Circle::contains(Real3D const& point) const
{
	Real2D l_point(point(0), point(1));
	return this->contains(l_point);
}

/////////////////////////////////////////////
bool Circle::contains(Real2D const& point) const
{
	return m_center.distance(point) < m_radius + Math::TOLERANCE;
}