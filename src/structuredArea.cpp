#include "structuredArea.h"
#include "discretizedArea.h"
#include "coverageUtility.h"
#include "shape2D.h"
#include "polygon.h"
#include "line2D.h"

#include "IDSReal2D.h"

#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

using namespace std;
using namespace Robotics::GameTheory;

std::set<std::shared_ptr<Shape2D>> createRandomObstacles()
{
	return std::set<std::shared_ptr<Shape2D> >();
}

//////////////////////////////////////////////////////////////////////////
StructuredArea::StructuredArea(std::vector<IDSReal2D> const& points) : m_external(nullptr)
{
	m_external = std::make_shared<Polygon>(points);
	m_obstacles = createRandomObstacles();
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
IDSReal2D StructuredArea::randomPosition() const
{
	/// Compute Random Position:
	srand ( (unsigned int) time(NULL) );

	IDSReal2D l_point;
	int k = 0;
	do
	{
		int xSecret = rand() % DISCRETIZATION_COL;
		int ySecret = rand() % DISCRETIZATION_ROW;

		IDSBox l_box = m_external->getBoundingBox();
		IDSReal2D l_bottomLeft = l_box.corner(0);
		IDSReal2D	l_bottomRight = l_box.corner(1);
		IDSReal2D	l_topLeft = l_box.corner(2);
		IDSReal2D	l_topRight = l_box.corner(3);

		double l_xdist = l_bottomLeft.distance(l_bottomRight);
		double l_ydist = l_bottomLeft.distance(l_topLeft);

		double l_xstep = l_xdist / double(DISCRETIZATION_COL);
		double l_ystep = l_ydist / double(DISCRETIZATION_ROW);

		Line2D l_xlineBottom (l_bottomLeft, l_bottomRight);
		Line2D l_xlineTop (l_topLeft, l_topRight);

		IDSReal2D l_bottom = l_xlineBottom.pointFromOrigin( l_xstep * double(xSecret) );
		IDSReal2D l_top = l_xlineTop.pointFromOrigin( l_xstep * double(xSecret) );

		Line2D l_yline (l_bottom,l_top);
		l_point = l_yline.pointFromOrigin( l_ystep * double(ySecret) );
		++k;
	} 
	while (!m_external->contains(l_point) && k < N_MAX);

	return l_point;
}

//////////////////////////////////////////////////////////////////////////
bool StructuredArea::isInside( IDSBox const& _box) const
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
IDSBox StructuredArea::getBoundingBox() const
{
	return m_external->getBoundingBox();
}

//////////////////////////////////////////////////////////////////////////
double StructuredArea::getDistance() const
{
	return m_external->getBoundingBox().minCoord.distance(m_external->getBoundingBox().maxCoord);
}
