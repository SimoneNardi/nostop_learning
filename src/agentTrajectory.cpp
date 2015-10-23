#include "agent.h"
#include "guard.h"
#include "discretizedArea.h"

#include "shape2D.h"
#include "circle.h"
#include "Real2D.h"

using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;

//////////////////////////////////////////////////////////////////////////
// MemoryGuardTrajectories
//////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////
void MemoryGuardTrajectories::add(MemoryGuardTrajectory const& _elem)
{
	m_elems.insert(m_elems.begin(), _elem);

	if(m_elems.size() > size_t(m_maxSize))
		m_elems.erase(m_elems.begin()+m_maxSize);
}

//////////////////////////////////////////////////////////////////////////
void MemoryGuardTrajectories::computeBestWorstTrajectories()
{
	double l_bestPayoff = -Math::Infinity;
	double l_worstPayoff = Math::Infinity;
	for(size_t i = 0; i < m_elems.size(); ++i)
	{
		double l_currentPayoff = m_elems[i].m_payoff;

		if(l_currentPayoff > l_bestPayoff)
		{
			l_bestPayoff = l_currentPayoff;
			m_best= i;
		}

		if(l_currentPayoff < l_worstPayoff)
		{
			l_worstPayoff = l_currentPayoff;
			m_worst = i;
		}
	}
	return;
}

//////////////////////////////////////////////////////////////////////////
//	GuardTrajectoryPosition
//////////////////////////////////////////////////////////////////////////
bool GuardTrajectoryPosition::operator==(GuardTrajectoryPosition const& other) const
{
	return m_position == other.m_position && m_index == other.m_index;
}

bool GuardTrajectoryPosition::operator!=(GuardTrajectoryPosition const& other) const
{
	return !( *this == other );
}

//////////////////////////////////////////////////////////////////////////
//	GuardTrajectory
//////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////
bool GuardTrajectory::contains(AgentPosition _nextPos)
{
	for(size_t i = 0; i < m_elems.size(); ++i)
	{
		if( m_elems[i].m_position.getPoint2D() == _nextPos.getPoint2D() )
			return true;
	}
	return false;
}

//////////////////////////////////////////////////////////////////////////
bool GuardTrajectory::operator==(GuardTrajectory const& other) const
{
	if(m_elems.size() != other.m_elems.size())
		return false;

	for(size_t i = 0; i < m_elems.size(); ++i)
	{
		if(!(m_elems[i] == other.m_elems[i]) )
			return false;
	}

	return true;
}

//////////////////////////////////////////////////////////////////////////
bool GuardTrajectory::operator!=(GuardTrajectory const& other) const
{
	return !(*this == other);
}

//////////////////////////////////////////////////////////////////////////
//	AgentPosition
//////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////
bool AgentPosition::operator==(AgentPosition const& other) const
{
	return m_point == other.m_point && m_camera == other.m_camera;
}

//////////////////////////////////////////////////////////////////////////
bool AgentPosition::operator!=(AgentPosition const& other) const
{
	return !(*this == other);
}

//////////////////////////////////////////////////////////////////////////
void AgentPosition::updateCounter(std::shared_ptr<DiscretizedArea> area)
{
	AreaCoordinate l_coord = area->getCoordinate(m_point);

	area->getSquare( l_coord.row, l_coord.col )->increaseCounter();

	// Calcolo la copertura in base alla camera:
	// per ora conta solo il raggio massimo e non cala con la distanza!
	std::vector<AreaCoordinate> l_cover = m_camera.getCoverage(l_coord, area);

	for(size_t i = 0; i < l_cover.size(); ++i)
	{
		std::shared_ptr<Square> l_square = area->getSquare( l_cover[i].row, l_cover[i].col );
		if(l_square)
			l_square->increaseCounter();
	}
}

//////////////////////////////////////////////////////////////////////////
bool AgentPosition::communicable(std::shared_ptr<Agent> _otherAgent) const
{
	return m_point.distance( _otherAgent->getCurrentPosition().getPoint2D() ) < 2 * m_camera.getFarRadius() + Math::TOLERANCE;
}

//////////////////////////////////////////////////////////////////////////
bool AgentPosition::visible(std::shared_ptr<Square> _area) const
{
	std::shared_ptr<Shape2D> sh = m_camera.getVisibleArea(m_point);
	if(sh->isValid())
		return sh->contains( _area->getBoundingBox().center() );
	else
		return false;
}

//////////////////////////////////////////////////////////////////////////
double AgentPosition::computeCosts() const
{
	return m_camera.computeCosts();
}

//////////////////////////////////////////////////////////////////////////
std::vector<AreaCoordinate> AgentPosition::getCoverage(std::shared_ptr<DiscretizedArea> _space ) const
{
	AreaCoordinate l_center = _space->getCoordinate(m_point);
	return m_camera.getCoverage(l_center, _space);
}

//////////////////////////////////////////////////////////////////////////
std::shared_ptr<Shape2D> AgentPosition::getVisibleArea() const
{
	return m_camera.getVisibleArea(m_point);
}

//////////////////////////////////////////////////////////////////////////
//	CameraPosition
//////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////
bool CameraPosition::operator==(CameraPosition const& other) const
{
	if(fabs(m_farRadius-other.m_farRadius) > Math::TOLERANCE)
		return false;
	else if(fabs(m_nearRadius-other.m_nearRadius) > Math::TOLERANCE)
		return false;
	else if(fabs(m_orientation-other.m_orientation) > Math::TOLERANCE)
		return false;
	else if(fabs(m_angle-other.m_angle) > Math::TOLERANCE)
		return false;
	return true;
}

//////////////////////////////////////////////////////////////////////////
bool CameraPosition::operator!=(CameraPosition const& other) const
{
	return !(*this==other);
}

//////////////////////////////////////////////////////////////////////////
std::vector<AreaCoordinate> CameraPosition::getCoverage(AreaCoordinate _center, std::shared_ptr<DiscretizedArea> _area) const
{
	Real2D l_pt = _area->getPosition(_center);
	std::shared_ptr<Shape2D> l_sensorArea = this->getVisibleArea(l_pt);
	if(!l_sensorArea || !l_sensorArea->isValid())
		return std::vector<AreaCoordinate>();

	int l_rowDelta = int(floor(m_farRadius / _area->getYStep())) + 1;
	int l_colDelta = int(floor(m_farRadius / _area->getXStep())) + 1;

	std::vector<AreaCoordinate> result;
	AreaCoordinate l_elem;
	for(int i = - l_rowDelta; i <= l_rowDelta; ++i)
	{
		int row = _center.row + i;
		if( row < 0 || row > _area->getNumRow() )
			continue;
		for(int j = - l_colDelta; j <= l_colDelta; ++j)
		{
			int col = _center.col + j;
			if( col < 0 || col > _area->getNumRow() )
				continue;

			SquarePtr l_square = _area->getSquare(row, col);
			if(!l_square)
				continue;

			if( l_sensorArea->contains(l_square->getBoundingBox().center()) )
			{
				l_elem.row = row;
				l_elem.col = col;
				result.push_back(l_elem);
			}
		}
	}
	return result;
}

//////////////////////////////////////////////////////////////////////////
std::shared_ptr<Shape2D> CameraPosition::getVisibleArea(Real2D const& point) const
	try
{
	if(fabs(m_farRadius) < Math::TOLERANCE)
		return nullptr;

	return std::make_shared<Circle>( point, m_farRadius );
}
catch (...)
{
	return nullptr;
}

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////
/*						MemoryGuardTrajectory							*/
//////////////////////////////////////////////////////////////////////////

bool MemoryGuardTrajectory::equals(MemoryGuardTrajectory const& _other) const
{
	if(this->m_memTrajectory != _other.m_memTrajectory)
		return false;

	if(fabs( this->m_payoff - _other.m_payoff ) > Math::TOLERANCE )
		return false;

	return true;
}

bool MemoryGuardTrajectory::equals(GuardTrajectory const& _trajectory, double _payoff) const
{
	if(this->m_memTrajectory != _trajectory)
		return false;

	if(fabs( this->m_payoff - _payoff ) > Math::TOLERANCE )
		return false;

	return true;
}


//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
