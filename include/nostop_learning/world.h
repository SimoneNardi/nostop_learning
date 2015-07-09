///////////////////////////////////////////////////////////
//  World.h
//  Created on:      05-may-2014 10.14.52
//  Original author: s.nardi
///////////////////////////////////////////////////////////
#ifndef WORLD_H
#define WORLD_H
#pragma once

#include <memory>
#include <vector>
#include <set>

class IDSReal2D;

namespace Robotics
{
	namespace GameTheory 
	{
		class DiscretizedArea;
		class Agent;
		class Guard;
		class Thief;
		class Sink;
		class LearningAlgorithm;
		class Area;
		class AgentPosition;
		class Square;

		class World
		{
		protected:
			/// The set of all the agents in the area.
			std::set< std::shared_ptr<Agent> > m_agent;

			/// The space where agents can move.
			std::shared_ptr<DiscretizedArea> m_space;

		public:
			World(std::set< std::shared_ptr<Agent> > _agent, std::shared_ptr<DiscretizedArea> _space);
			World(std::shared_ptr<Agent> _agent, std::shared_ptr<DiscretizedArea> _space);
			World(std::set< std::shared_ptr<Agent> > _agent, std::shared_ptr<Area> _space);

			/// All agents
			std::set< std::shared_ptr<Agent> > getAgents() const {return m_agent;}
			
			/// Guards
			std::set< std::shared_ptr<Guard> > getGuards() const;

			/// Thieves
			std::set< std::shared_ptr<Thief> > getThieves() const;

			/// Neutrals
			std::set< std::shared_ptr<Agent> > getNeutrals() const;

			/// Sink
			std::set< std::shared_ptr<Sink> > getSinks() const;

			// Move Thieves in the world
			void moveThieves(int _thiefJump);

			inline std::shared_ptr<DiscretizedArea> getSpace() const {return m_space;}

			void wakeUpAgentIfSecurityIsLow();

			void randomInitializeGuards();

			void randomInitializeNeutrals();

			void randomInitializeThief();

			int getNumberOfAgent();

			void addThief(std::shared_ptr<Thief> _thief);

			void addSink(std::shared_ptr<Sink> _sink);

			double getMaximumValue();

			void removeAllThieves();

			void removeAllSinks();

			void saveConfiguration(std::ofstream & _stream);

			void getSinksPosition(std::vector<AgentPosition> & _pos);

			void getSinksSquare(std::vector<std::pair<std::shared_ptr<Square>,int>> & _pos);

			void getSinksCoverage( std::vector< std::vector<IDSReal2D> > & _areas);
		};

		typedef std::shared_ptr<World> WorldPtr;
	}
}
#endif