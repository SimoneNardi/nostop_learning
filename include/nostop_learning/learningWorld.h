////////////////////////////////////////////////////
//	LearningWorld.h
//	Created on:	12-jun-2015
//	Original author: Simone Nardi
////////////////////////////////////////////////////
#ifndef LEARNING_WORLD_H
#define LEARNING_WORLD_H
#pragma once

#include <memory>
#include <set>
#include <vector>
#include <thread>
#include <mutex>

//struct A;
class Real2D;
namespace Robotics 
{
	namespace GameTheory
	{
		enum LEARNING
		{
			DISL = 0,
			PIPIP = 1,
			PARETO = 2,
			CORRELATED = 3
		};

		class World;
		class LearningAlgorithm;
		class Agent;
		class DiscretizedArea;

		class LearningWorld
		{
		protected:
			std::recursive_mutex m_mutex;

			std::shared_ptr<World> m_world;

			/// The learning algorithm used by agents to change their status.
			std::shared_ptr<LearningAlgorithm> m_learning;

			int m_count;

		public:
			LearningWorld(
				const std::set< std::shared_ptr<Agent> >& agent_, 
				std::shared_ptr<DiscretizedArea> space_, 
				LEARNING type_);
			
			LearningWorld(
				const std::shared_ptr<Agent>& agent_, 
				std::shared_ptr<DiscretizedArea> space_, 
				LEARNING type_);

			void start();

			/// Update algorithm of one step
			bool evaluateScreenshot();

			/// Update agent position and control configuration
			bool agentUpdate();

			/// Update visualization paramenters
			bool visualUpdate();
			
			/// Update world paramenters
			bool worldUpdate();

			/// Get Learning algorithm reference
			std::shared_ptr<LearningAlgorithm> getLearning() const;

			/// Get World Reference
			std::shared_ptr<World> getWorld() const;
			
			///
			void getMonitorScreenShot(std::vector<double> & monitorData_);
			
			///
			void updateMonitor(std::vector<int8_t> const& monitorData_);
			
			///
			void getNeighboursScreeShot(std::vector<double> & neighboursData_);
			
			///
			void updateNeighbours(std::vector<int8_t> const& neighboursData_);
			
			///
			void getEnergyScreenShot(std::vector<double> & energyData_);
			
			///
			void updateEnergy(std::vector<int8_t> const& energyData_);
			
			
			///
			bool forwardOneStep();
			
			///
			void addGuard(std::shared_ptr<Agent> );
			
			///
			void addThief(std::shared_ptr<Agent>);
			
			///
			void addSink(std::shared_ptr<Agent>);
			
			///
			void getPlayersPosition(std::set<Real2D>&) const;
			
			///
			void getThievesPosition(std::set<Real2D>&) const;
			
			///
			int getNumberOfAgents() const;

		protected:
			/// Update guard position
			void updateGuardsPosition();

			/// Update guard control configuration
			void updateGuardsFootprint();

			/// Update thief position
			void updateThievesPosition();

			void setLearningAlgorithm(std::shared_ptr<LearningAlgorithm> _learning);
		};

		typedef std::shared_ptr<LearningWorld> LearningWorldPtr;

	}

}

#endif