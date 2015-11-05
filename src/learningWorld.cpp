#include "learningWorld.h"
#include "agent.h"
#include "discretizedArea.h"
#include "world.h"

#include "DISLAlgorithm.h"
#include "PIPIPAlgorithm.h"
#include "ParetoEfficientAlgorithm.h"
#include "CoarseCorrelatedAlgorithm.h"

#include "thief.h"
#include "guard.h"

#include <memory>

using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;

//////////////////////////////////////////////////////////////////////////
LearningWorld::LearningWorld(
	const std::set< std::shared_ptr<Agent> >& agent_, 
	std::shared_ptr<DiscretizedArea> space_, 
	LEARNING type_)
	: m_world(nullptr)
	, m_learning(nullptr)
	, m_count(0)
{
	m_world = std::make_shared<World>(agent_, space_);

	std::shared_ptr<LearningAlgorithm> l_learning = nullptr;
	if(type_ == DISL)
		l_learning = std::make_shared<DISLAlgorithm>(m_world->getSpace());
	else if(type_ == PIPIP)
		l_learning = std::make_shared<PIPIPAlgorithm>(m_world->getSpace());
	else if(type_ == PARETO)
		l_learning = std::make_shared<ParetoEfficientAlgorithm>(m_world->getSpace());
	else if(type_ == CORRELATED)
		l_learning = std::make_shared<CoarseCorrelatedAlgorithm>(m_world->getSpace());
	else 
		throw std::exception();

	l_learning->setExperimentalRate(0.5);

	this->setLearningAlgorithm(l_learning);
}

//////////////////////////////////////////////////////////////////////////
LearningWorld::LearningWorld(
	const std::shared_ptr<Agent>& agent_, 
	std::shared_ptr<DiscretizedArea> space_, 
	LEARNING type_)
	: m_world(nullptr)
	, m_learning(nullptr)
	, m_count(0)
{
	m_world = std::make_shared<World>(agent_, space_);

	std::shared_ptr<LearningAlgorithm> l_learning = nullptr;
	if(type_ == DISL)
		l_learning = std::make_shared<DISLAlgorithm>(m_world->getSpace());
	else if(type_ == PIPIP)
		l_learning = std::make_shared<PIPIPAlgorithm>(m_world->getSpace());
	else if(type_ == PARETO)
		l_learning = std::make_shared<ParetoEfficientAlgorithm>(m_world->getSpace());
	else if(type_ == CORRELATED)
		l_learning = std::make_shared<CoarseCorrelatedAlgorithm>(m_world->getSpace());
	else 
		throw std::exception();

	l_learning->setExperimentalRate(0.5);

	this->setLearningAlgorithm(l_learning);
}

//////////////////////////////////////////////////////////////////////////
void LearningWorld::setLearningAlgorithm(std::shared_ptr<LearningAlgorithm> _learning)
{
	m_learning = _learning;
	m_learning->setGuards(m_world->getGuards());
}

//////////////////////////////////////////////////////////////////////////
void LearningWorld::start()
{
	m_learning->resetValue();  
	m_learning->monitoringThieves(m_world->getThieves());
	m_learning->monitoringSinks(m_world->getSinks());
}

//////////////////////////////////////////////////////////////////////////
bool LearningWorld::agentUpdate()
{
	this->updateGuardsPosition();

	this->updateGuardsFootprint();

	this->updateThievesPosition();

	return true;
}

//////////////////////////////////////////////////////////////////////////
bool LearningWorld::evaluateScreenshot()
{
	++m_count;
	m_learning->resetCounter();
	
	m_learning->updateCounterOfVisibleSquare();
	
	return true;
}

//////////////////////////////////////////////////////////////////////////
bool LearningWorld::worldUpdate()
{
	m_learning->resetValue();

	m_learning->monitoringThieves(m_world->getThieves());

	m_learning->monitoringSinks(m_world->getSinks());

	return true;
}

//////////////////////////////////////////////////////////////////////////
bool LearningWorld::visualUpdate()
{
	return true;
}

//////////////////////////////////////////////////////////////////////////
void LearningWorld::updateGuardsPosition()
{
	std::set< GuardPtr > l_agents =  m_world->getGuards();
	for(auto it = l_agents.begin(); it != l_agents.end(); ++it)
	{
		GuardPtr l_agent = *it;
		l_agent->updatePosition();
	}
}

//////////////////////////////////////////////////////////////////////////
void LearningWorld::updateGuardsFootprint()
{
	std::set< GuardPtr > l_agents =  m_world->getGuards();
	for(auto it = l_agents.begin(); it != l_agents.end(); ++it)
	{
		GuardPtr l_agent = *it;
		l_agent->updateFootprint();
	}
}

//////////////////////////////////////////////////////////////////////////
void LearningWorld::updateThievesPosition()
{
	std::set< ThiefPtr > l_agents =  m_world->getThieves();
	for(auto it = l_agents.begin(); it != l_agents.end(); ++it)
	{
		ThiefPtr l_agent = *it;
		l_agent->updatePosition();
	}
}

//////////////////////////////////////////////////////////////////////////
std::shared_ptr<LearningAlgorithm> LearningWorld::getLearning() const
{
	return m_learning;
}

//////////////////////////////////////////////////////////////////////////
std::shared_ptr<World> LearningWorld::getWorld() const
{
	return m_world;
}

///
void LearningWorld::getEnergyScreenShot(std::vector<double> & sinkData_)
{
    auto l_lattice = m_world->getSpace()->getLattice();
  sinkData_.resize(l_lattice.size());
  for(auto i = 0; i < sinkData_.size(); ++i)
  {
    sinkData_[i] = int8_t(l_lattice[i]->getEnergyValue());
  }
}

///
void LearningWorld::updateEnergy(std::vector<int8_t> const& sinkData_)
{
  auto l_lattice = m_world->getSpace()->getLattice();
  for(auto i = 0; i < l_lattice.size(); ++i)
  {
    l_lattice[i]->setEnergyValue(sinkData_[i]);
  }
}

///
void LearningWorld::getMonitorScreenShot(std::vector<double> & monitorData_)
{
    auto l_lattice = m_world->getSpace()->getLattice();
  monitorData_.resize(l_lattice.size());
  for(auto i = 0; i < monitorData_.size(); ++i)
  {
    monitorData_[i] = l_lattice[i]->getThiefValue();
  }
}

///
void LearningWorld::updateMonitor(std::vector<int8_t> const& monitorData_)
{
  auto l_lattice = m_world->getSpace()->getLattice();
  for(auto i = 0; i < l_lattice.size(); ++i)
  {
    l_lattice[i]->setThiefValue(monitorData_[i]);
  }
}

///
void LearningWorld::getNeighboursScreeShot(std::vector<double> & neighboursData_)
{
  auto l_lattice = m_world->getSpace()->getLattice();
  neighboursData_.resize(l_lattice.size());
  for(auto i = 0; i < l_lattice.size(); ++i)
  {
    neighboursData_[i] = l_lattice[i]->getTheNumberOfAgent();
  }
}

///
void LearningWorld::updateNeighbours(std::vector<int8_t> const& neighboursData_)
{
  auto l_lattice = m_world->getSpace()->getLattice();
  for(auto i = 0; i < l_lattice.size(); ++i)
  {
    l_lattice[i]->setTheNumberOfAgent(neighboursData_[i]);
  }
}

///
bool LearningWorld::forwardOneStep()
{
	// Monitor has been already updated and Neighbours are already computed:
	// so everything that we need to do is compute benefit of guards and select next action.
	double l_rate = m_learning->computeExplorationRate();
	if(l_rate < 1.e-5)
	  // In this case, the algorithm is considered terminated.
	  return false;

	//	COMPUTE: Compute Benefit
	auto l_guards = m_world->getGuards();
	for(auto it = l_guards.begin(); it!= l_guards.end(); ++it)
	{
		//	ogni agente guardia calcola la prima utilità:
		m_learning->compute(*it);
	}
	
	//	UPDATE: Save current action and Select next position:
	for(auto it = l_guards.begin(); it!= l_guards.end(); ++it)
	{
		m_learning->updateWithoutMoving(*it);
	}

	return true;
}

///
void LearningWorld::addGuard(std::shared_ptr<Agent> agent_)
{
  // TODO
  return;
}
			
///
void LearningWorld::addThief(std::shared_ptr<Agent> agent_)
{
  //TODO
  return;
}
			
///
void LearningWorld::addSink(std::shared_ptr<Agent> agent_)
{
  //TODO
  return;
}

///
void LearningWorld::getPlayersPosition(std::set<Real2D>& r_agents_pos) const
{
  auto l_guards = m_world->getGuards();
  for(auto it = l_guards.begin(); it != l_guards.end(); ++it)
  {
    GuardPtr l_guard = *it;
    AgentPosition l_pos = l_guard->getCurrentPosition();
    r_agents_pos.insert( l_pos.getPoint2D() );
  }
  return;
}
			
///
void LearningWorld::getThievesPosition(std::set<Real2D>& r_agents_pos) const
{
  auto l_thieves = m_world->getThieves();
  for(auto it = l_thieves.begin(); it != l_thieves.end(); ++it)
  {
    ThiefPtr l_thief = *it;
    AgentPosition l_pos = l_thief->getCurrentPosition();
    r_agents_pos.insert( l_pos.getPoint2D() );
  }
  
  return;
}

///
int LearningWorld::getNumberOfAgents() const
{
  auto l_agents = m_world->getAgents();
  return l_agents.size();
}