#include "agent.h"
#include "area.h"
#include "discretizedArea.h"
#include "guard.h"
#include "thief.h"
#include "sink.h"
#include "probability.h"

#include <memory>

#include "shape2D.h"

#include "Threads.h"
 
using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;

const int g_period = 5;

//////////////////////////////////////////////////////////////////////////
bool Agent::getRandomFeasibleAction(std::vector<AgentPosition> const& _feasible, AgentPosition & r_pos) const
{
	int l_tot = _feasible.size();
	if(l_tot = 0)
		return false;

	int l_rand = getRandomValue(l_tot);
	r_pos = _feasible[l_rand];

	return true;
}

//////////////////////////////////////////////////////////////////////////
void Agent::setCurrentPosition(AgentPosition const& pos)
{
  Lock l_lock(m_mutex);
	m_currentPosition = pos;
}

//////////////////////////////////////////////////////////////////////////
void Agent::setNextPosition(AgentPosition const& pos)
{
  Lock l_lock(m_mutex);
	m_nextPosition = pos;
}

//////////////////////////////////////////////////////////////////////////
AgentPosition Agent::getCurrentPosition() const 
{
  Lock l_lock(m_mutex);
  return m_currentPosition;
}

//////////////////////////////////////////////////////////////////////////
int Agent::getID() const 
{
  Lock l_lock(m_mutex);
  return m_id;
}
			
//////////////////////////////////////////////////////////////////////////
std::shared_ptr<Shape2D> Agent::getVisibleArea() const 
{
  			  Lock l_lock(m_mutex);
  return m_currentPosition.getVisibleArea();
}

//////////////////////////////////////////////////////////////////////////
bool Agent::isActive()
{
	Lock l_lock(m_mutex);
	bool wakeUp = m_status == Agent::WAKEUP;
	if(wakeUp)
		setStatus(Agent::ACTIVE);

	return m_status == Agent::ACTIVE;
}

//////////////////////////////////////////////////////////////////////////
bool Agent::isOutOfInterest( std::shared_ptr<DiscretizedArea> space) const
{
	Lock l_lock(m_mutex);
	return space->isOut(m_currentPosition);
}

//////////////////////////////////////////////////////////////////////////
void Agent::sleep()
{
  Lock l_lock(m_mutex);
	return setStatus(Agent::STANDBY);
}

//////////////////////////////////////////////////////////////////////////
void Agent::wakeUp()
{
  Lock l_lock(m_mutex);
	return setStatus(Agent::WAKEUP);
}

//////////////////////////////////////////////////////////////////////////
Agent::Status Agent::getStatus() const
{
  Lock l_lock(m_mutex);
	return m_status;
}

//////////////////////////////////////////////////////////////////////////PRIVATE
void Agent::setStatus(Status stat)
{
  Lock l_lock(m_mutex);
	m_status = stat;
}

//////////////////////////////////////////////////////////////////////////
std::vector<AgentPosition> Agent::getFeasibleActions( std::shared_ptr<DiscretizedArea> _space ) const
{
  Lock l_lock(m_mutex);
	AreaCoordinate l_currCoord = _space->getCoordinate( m_currentPosition.getPoint2D() );

	std::vector<AreaCoordinate> l_squares = _space->getStandardApproachableValidSquares(l_currCoord);

	std::vector<AgentPosition> l_result;
	for( size_t i = 0; i < l_squares.size(); ++i )
	{
		l_result.push_back( AgentPosition(_space->getPosition(l_squares[i]), m_currentPosition.m_camera) );
	}

	return l_result;
}

//////////////////////////////////////////////////////////////////////////
AgentPosition Agent::selectRandomFeasibleAction(std::shared_ptr<DiscretizedArea> _space)
{
  Lock l_lock(m_mutex);
	std::vector<AgentPosition> l_feasible = this->getFeasibleActions(_space);
	if(l_feasible.empty())
		return m_currentPosition;
	else
	{
		//this->removeBestTrajectoryFromFeasible(l_feasible);

		int l_value = getRandomValue( int( l_feasible.size() ) );
		return l_feasible[l_value];
	}
}

//////////////////////////////////////////////////////////////////////////
bool Agent::equals(std::shared_ptr<Agent> _other) const
{
  Lock l_lock(m_mutex);
	return _other->m_id == m_id;
}

//////////////////////////////////////////////////////////////////////////
std::shared_ptr<Thief> Agent::toThief()
{
  Lock l_lock(m_mutex);
	return std::dynamic_pointer_cast<Thief>(this->shared_from_this());
}

//////////////////////////////////////////////////////////////////////////
std::shared_ptr<Guard> Agent::toGuard()
{
  Lock l_lock(m_mutex);
	return dynamic_pointer_cast<Guard>(this->shared_from_this());
}

//////////////////////////////////////////////////////////////////////////
std::shared_ptr<Sink> Agent::toSink()
{
  Lock l_lock(m_mutex);
	return dynamic_pointer_cast<Sink>(this->shared_from_this());
}

//////////////////////////////////////////////////////////////////////////
void Agent::moveToNextPosition()
{
  Lock l_lock(m_mutex);
	m_currentPosition = m_nextPosition;
}

//////////////////////////////////////////////////////////////////////////
void Agent::updateFootprint()
{
Lock l_lock(m_mutex);
}
//////////////////////////////////////////////////////////////////////////			
void Agent::updatePosition()
{
Lock l_lock(m_mutex);
}

//////////////////////////////////////////////////////////////////////////			
double Agent::getCurrentRotation() const
{
	Lock l_lock(m_mutex);
	Real2D l_delta = m_nextPosition.getPoint2D() - m_currentPosition.getPoint2D();
	
	return Math::polarPhi2D(l_delta);
}

//////////////////////////////////////////////////////////////////////////			
double Agent::getCurrentSpeed() const
{
	Lock l_lock(m_mutex);
	return 10; // unit measure?
}
