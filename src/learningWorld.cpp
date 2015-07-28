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

	l_learning->setExperimentalRate(0.1);

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

	l_learning->setExperimentalRate(0.1);

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
void LearningWorld::updateMonitor(std::vector<int8_t> const& monitorData_)
{
  // TODO
}

///
void LearningWorld::updateNeighbours(std::vector<int8_t> const& neighboursData_)
{
  // TODO
}

///
void LearningWorld::forwardOneStep()
{
  // TODO
}
