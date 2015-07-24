////////////////////////////////////////////////////
//	agent.h
//	Created on:	12-jun-2015
//	Original author: Simone Nardi
////////////////////////////////////////////////////
#ifndef AGENT_H
#define AGENT_H
#pragma once

#include <vector>
#include <memory>

#include "shape2D.h"

#include "IDSReal2D.h"
#include "Threads.h"

namespace Robotics 
{
	namespace GameTheory
	{
		class DiscretizedArea;
		struct AreaCoordinate;
		class Area;
		class Agent;
		class Sink;
		class Thief;
		class Guard;
		class Square;

		//////////////////////////////////////////////////////////////////////////
		class CameraPosition
		{
		protected:
			/// Major radius.
			double m_farRadius;
			/// Minor Radius.
			double m_nearRadius;
			/// Orientation.
			double m_orientation; // [0, 2*Pi)
			/// Angle of view.
			double m_angle;

		public:
			CameraPosition(double _farRadius = 0., double _nearRadius = 0., double _orientation = 0., double _angle = IDSMath::TwoPi) 
				: m_farRadius(_farRadius), m_nearRadius(_nearRadius), m_orientation(_orientation), m_angle(_angle) {}

			std::vector<AreaCoordinate> getCoverage(AreaCoordinate _center, std::shared_ptr<DiscretizedArea> _area) const;

			double getFarRadius() const {return m_farRadius;}
			double getNearRadius() const {return m_nearRadius;}
			double getOrientation() const {return m_orientation;}
			double getAngleOfView() const {return m_angle;}

			std::shared_ptr<Shape2D> getVisibleArea(IDSReal2D const& point) const;

			double computeCosts() const {return 0.;}

			bool operator==(CameraPosition const& other) const;
			bool operator!=(CameraPosition const& other) const;
		};

		//////////////////////////////////////////////////////////////////////////
		class AgentPosition
		{
		protected:
			/// The position of the agent.
			IDSReal2D m_point;

			/// The camera orientation.
			CameraPosition m_camera;

		public:
			AgentPosition() {};

			AgentPosition(IDSReal2D const& point) : m_point(point), m_camera() {}

			AgentPosition(IDSReal2D const& point, CameraPosition _camera) : m_point(point), m_camera(_camera) {}

			/// Update the counter of the lattice visible from that position
			void updateCounter(std::shared_ptr<DiscretizedArea> area);

			/// Get Point2D
			IDSReal2D getPoint2D() const {return m_point;}
			
			CameraPosition getCameraControl() const {return m_camera;}

			/// True if other is in communication with this position
			bool communicable(std::shared_ptr<Agent> _other) const;

			/// is the center of the square visible in that position and that camera?
			bool visible(std::shared_ptr<Square> _area) const;

			/// Compute Camera Costs
			double computeCosts() const;

			std::vector<AreaCoordinate> getCoverage(std::shared_ptr<DiscretizedArea> _space ) const;

			std::shared_ptr<Shape2D> getVisibleArea() const;

			bool operator==(AgentPosition const& other) const;
			bool operator!=(AgentPosition const& other) const;

			friend class Agent;
			friend class Guard;
			friend class Thief;
			friend class Sink;
		};

		//////////////////////////////////////////////////////////////////////////
		class Agent : public std::enable_shared_from_this<Agent>
		{
		public: 
			enum Status 
			{
				ACTIVE = 0,
				DISABLE = 1,
				STANDBY = 2,
				WAKEUP = 3
			}; 

		protected:
			/// Agent Identifier
			int m_id;

			AgentPosition m_currentPosition;

			AgentPosition m_nextPosition;

			mutable Status m_status;
			
			mutable Mutex m_mutex;

		public:

			Agent(int _id, AgentPosition _position)
				: m_id(_id)
				, m_currentPosition(_position)
				, m_status(ACTIVE)
			{}

			~Agent() {}

			int getID() const;

			/// Get the position of the agent.
			AgentPosition getCurrentPosition() const;
			
			/// Set Current Position
			void setCurrentPosition(AgentPosition const& _pos);

			/// Set Current Position
			void setNextPosition(AgentPosition const& _pos);

			/// True if the Agent is active, false otherwise.
			bool isActive();

			/// True if the agent is next to be put on Stand-By, false otherwise.
			bool isOutOfInterest( std::shared_ptr<DiscretizedArea> space) const;

			/// Set the agent on standBy status. 
			void sleep();

			/// Set the agent on wakeUp status. 
			void wakeUp();

			virtual bool isThief() const {return false;}

			virtual bool isGuard() const {return false;}

			virtual bool isSink() const {return false;}

			virtual bool isNeutral() const {return false;}

			std::shared_ptr<Thief> toThief();

			std::shared_ptr<Guard> toGuard();

			std::shared_ptr<Sink> toSink();

			virtual void moveToNextPosition();

			virtual std::vector<AgentPosition> getFeasibleActions( std::shared_ptr<DiscretizedArea> _space ) const;

			bool getRandomFeasibleAction(std::vector<AgentPosition> const& _feasible, AgentPosition & _pos) const;

			AgentPosition selectRandomFeasibleAction(std::shared_ptr<DiscretizedArea> _space);

			bool equals(std::shared_ptr<Agent>) const;

			std::shared_ptr<Shape2D> getVisibleArea() const;

#pragma region ROS
			// Send broadcast messages.
			virtual void updateFootprint();

			virtual void updatePosition();

#pragma endregion
			Status getStatus() const;

			void setStatus(Status stat);
			
		
#pragma region ROS
			double getCurrentRotation() const;
			double getCurrentSpeed() const;
#pragma endregion
		};

		typedef std::shared_ptr<Agent> AgentPtr;

		class AgentActionIndex
		{
		public:
			int m_elem;
			int m_total;

			AgentActionIndex( int _elem, int _total ) : m_elem(_elem), m_total(_total) {}
		};
	}
}

#endif // AGENT_H