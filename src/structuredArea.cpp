#include "structuredArea.h"
#include "discretizedArea.h"
#include "coverageUtility.h"
#include "shape2D.h"
#include "polygon.h"
#include "line2D.h"

#include "Real2D.h"

#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

using namespace std;
using namespace Robotics::GameTheory;

std::set<std::shared_ptr<Shape2D>> createRandomObstacles()
{
	return std::set<std::shared_ptr<Shape2D> >();
}

//////////////////////////////////////////////////////////////////////////
StructuredArea::StructuredArea(std::vector<Real2D> const& points) : m_external(nullptr)
{
	m_external = std::make_shared<Polygon>(points);
	m_obstacles = createRandomObstacles();
}

//////////////////////////////////////////////////////////////////////////
StructuredArea::StructuredArea(std::vector<Real2D> const& points, std::vector< std::vector<Real2D> > const& obstacles)
{
  	m_external = std::make_shared<Polygon>(points);
	
	std::set< std::shared_ptr<Shape2D> > l_obstacles;
	for(size_t i= 0; i < obstacles.size(); ++i)
	{
	  std::shared_ptr<Polygon> l_poly = std::make_shared<Polygon>(obstacles[i]);
	  l_obstacles.insert(l_poly);
	}
	m_obstacles = l_obstacles;
}

//////////////////////////////////////////////////////////////////////////
std::shared_ptr<DiscretizedArea> StructuredArea::discretize()
{
	std::shared_ptr<Area> l_area = this->shared_from_this();
	std::shared_ptr<StructuredArea> l_struct = dynamic_pointer_cast<StructuredArea>(l_area);
	return std::make_shared<DiscretizedArea>( l_struct );
}

const int N_MAX = 15;

//////////////////////////////////////////////////////////////////////////
Real2D StructuredArea::randomPosition() const
{
	/// Compute Random Position:
	srand ( (unsigned int) time(NULL) );

	Real2D l_point;
	int k = 0;
	bool l_continue = false;
	do
	{
		int xSecret = rand() % DISCRETIZATION_COL;
		int ySecret = rand() % DISCRETIZATION_ROW;

		Box l_box = m_external->getBoundingBox();
		Real2D l_bottomLeft = l_box.corner(0);
		Real2D	l_bottomRight = l_box.corner(1);
		Real2D	l_topLeft = l_box.corner(2);
		Real2D	l_topRight = l_box.corner(3);

		double l_xdist = l_bottomLeft.distance(l_bottomRight);
		double l_ydist = l_bottomLeft.distance(l_topLeft);

		double l_xstep = l_xdist / double(DISCRETIZATION_COL);
		double l_ystep = l_ydist / double(DISCRETIZATION_ROW);

		Line2D l_xlineBottom (l_bottomLeft, l_bottomRight);
		Line2D l_xlineTop (l_topLeft, l_topRight);

		Real2D l_bottom = l_xlineBottom.pointFromOrigin( l_xstep * double(xSecret) );
		Real2D l_top = l_xlineTop.pointFromOrigin( l_xstep * double(xSecret) );

		Line2D l_yline (l_bottom,l_top);
		l_point = l_yline.pointFromOrigin( l_ystep * double(ySecret) );
		++k;
		
		l_continue = !m_external->contains(l_point) /*&& k < N_MAX*/;
		for(auto it = m_obstacles.begin(); it != m_obstacles.end() && !l_continue; ++it)
		{
		    l_continue = (*it)->contains(l_point);
		}
	} 
	while (l_continue);

	return l_point;
}

//////////////////////////////////////////////////////////////////////////
bool StructuredArea::isInside( Box const& _box) const
{
	bool l_inside = false;
	for(int i = 0; i < 4; ++i)
	{
		l_inside = m_external->contains(_box.corner(i));
		if(l_inside)
		{
			bool l_outFromObstacle = true;
			for(auto it = m_obstacles.begin(); it != m_obstacles.end(); ++it)
			{
				l_outFromObstacle = !(*it)->contains(_box.corner(i));
				if(!l_outFromObstacle)
				{
					l_inside = false;
					break;
				}
			}
			if(l_inside)
				return true;
		}
	}

	l_inside = m_external->contains(_box.center());
	if(l_inside)
	{
		bool l_outFromObstacle = true;
		for(auto it = m_obstacles.begin(); it != m_obstacles.end(); ++it)
		{
			l_outFromObstacle = !(*it)->contains(_box.center());
			if(!l_outFromObstacle)
			{
				l_inside = false;
				break;
			}
		}
		if(l_inside)
			return true;
	}

	return false;
}

//////////////////////////////////////////////////////////////////////////
Box StructuredArea::getBoundingBox() const
{
	return m_external->getBoundingBox();
}

//////////////////////////////////////////////////////////////////////////
double StructuredArea::getDistance() const
{
	return m_external->getBoundingBox().minCoord.distance(m_external->getBoundingBox().maxCoord);
}
