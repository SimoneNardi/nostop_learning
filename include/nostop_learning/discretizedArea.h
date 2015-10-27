///////////////////////////////////////////////////////////
//  DiscretizedArea.h
//  Created on:      13-may-2013 10.14.52
//  Original author: s.nardi
///////////////////////////////////////////////////////////
#ifndef DISCRETIZED_AREA_H
#define DISCRETIZED_AREA_H
#pragma once

#include "area.h"

#include "Box.h"

#include <lemon/list_graph.h>
#include <lemon/bfs.h>

#include <set>

namespace 
{
	namespace BaseGeometry
	{
		class Shape2D;
		class Real2D;
	}
}

namespace Robotics 
{
	namespace GameTheory 
	{
		const double g_maxValue = 100.;
		const double g_maxValuePossible = (g_maxValue/8. + g_maxValue/7. * 2. +  g_maxValue/6. * 3. +  g_maxValue/5. * 4. + g_maxValue/4. * 3. + g_maxValue/3. * 2. + g_maxValue/2. ) * 4 + g_maxValue*5. + g_maxValue/2.*4 + g_maxValue/3.*4 + g_maxValue/4.*4;

		class Graph;
		class Agent;
		class StructuredArea;
		class UnStructuredArea;

		struct AreaCoordinate
		{
			int row;
			int col;

			AreaCoordinate(int _col = -1, int _row = -1) : row(_row), col(_col) {}
		};

		inline bool operator< (AreaCoordinate const& a, AreaCoordinate const& b)
		{
			if(a.row < b.row)
				return true;
			else if(a.row > b.row)
				return false;
			else if(a.col < b.col)
				return true;
			else if(a.col > b.col)
				return false;
			else
				return false;
		}

		class  Square
		{
		protected:
			lemon::ListGraph::Node m_node; 

			/// True if an agent can stay in this square
			bool m_valid;

			/// number of agent which can examine this square
			int m_counter;

			/// The value associated to the square by the monitor
			std::vector<double> m_values;

			Box m_box;

			std::vector<double> m_old_values;
		public:

			/// Set the box of the Square
			void setBoundingBox(Box const& _box);
			/// Get the Bounding Box of the square
			Box getBoundingBox() {return m_box;}

			/// Set counter to zero.
			inline void resetCounter() {m_counter = 0;};

			/// Add one to counter.
			inline void increaseCounter() {++m_counter;};

			inline bool isValid() const {return m_valid;}
			inline void setValid(bool valid);

			inline int getTheNumberOfAgent() const {return m_counter;}
			inline int setTheNumberOfAgent(int num_) {m_counter=num_;}

			inline double getThiefValue() const {return m_values.size() > 0 ? m_values.at(0) : 0;}
			void setThiefValue(double _value);

			inline double getEnergyValue() const {return m_values.size() > 1 ? m_values.at(1) : 0;}
			void setEnergyValue(double _value);

			//Square() : m_valid(true), m_counter(0), m_value(0.), m_old_value(0.) {}
			Square(std::shared_ptr<lemon::ListGraph> _graph);
			lemon::ListGraph::Node getNode() const {return m_node;}

			Real2D vertex(int i) const;
			Real2D agentVertex(int i) const;

			bool isChanged() const;
			/// Set the value to zero.
			void resetThiefValue();
			void resetEnergyValue();

			//bool equals(std::shared_ptr<Square> _other) const;

			friend class  DiscretizedArea; 
		};

		typedef std::shared_ptr<Square> SquarePtr;

		class  DiscretizedArea : public std::enable_shared_from_this<DiscretizedArea>
		{
			std::shared_ptr<lemon::Bfs<lemon::ListGraph>> m_graph;
			std::shared_ptr<lemon::ListGraph> m_listGraph;

			/// Lattice from bottom left to upper right, per rows.
			std::vector< SquarePtr > m_lattice;
			int m_numRow;
			int m_numCol;

			/// x step to speed up searching
			double m_xStep;
			/// y step to speed up searching
			double m_yStep;

			int m_numberOfValidSquare;

			AreaCoordinate m_sinkCoordinate;

		public:
			DiscretizedArea(Shape2D const& _external, std::set< Shape2D > const& _obstacles);

			DiscretizedArea(std::shared_ptr<StructuredArea> _area);
			DiscretizedArea(std::shared_ptr<UnStructuredArea> _area);
			DiscretizedArea(std::string const& _filename);

			/// Compute Graph of the discretized area.
			Graph getGraph() const;

			/// Update index of lattice covered by the given agent.
			void updateSquaresCounter(std::shared_ptr<Agent> agent);

			/// Set all the lattice indexes to zero.
			void resetCounter();
			void resetValue();

			/// Get the origin of the lattice.
			Real2D getOrigin() const;

			/// Compute index of row and index of column of the given point.
			AreaCoordinate getCoordinate( Real2D const& point ) const;

			SquarePtr getSquare(int row, int col) const;
			SquarePtr getSquare(AreaCoordinate _coord) const;
			SquarePtr getSquare(Real2D const& V) const;
			Real2D getPosition(AreaCoordinate const& _coord) const;

			int getDistance(
				SquarePtr source, 
				SquarePtr target);

			int getDistance(
				AreaCoordinate _source, 
				AreaCoordinate _target);

			int getDistanceFromNearestSink(Real2D const& _agentPosition);

			std::vector<SquarePtr> getSquares() const;

			bool getRandomPosition(Real2D & _point) const;
			void setThiefPosition(AgentPosition const& _pos);
			void setSinkPosition(AgentPosition const& _pos);
			double getThiefMaxValue(AgentPosition const& _pos);

			bool isOut(AgentPosition const& pos) const;

			/// 
			void setRandomSquareValue();

			std::vector<AreaCoordinate> getStandardApproachableValidSquares(AreaCoordinate const& _current) const;
			void addSpecialApproachableValidSquares(AreaCoordinate const& _current, std::vector<AreaCoordinate> & _loci) const;

			std::set<std::shared_ptr<Square> > getVisibleSquares(AgentPosition const& _pos);

			double getXStep() const {return m_xStep;}
			double getYStep() const {return m_yStep;}
			int getNumRow() const {return m_numRow;}
			int getNumCol() const {return m_numCol;}
			std::vector< SquarePtr > getLattice() const {return m_lattice;}

			int numberOfSquaresCoveredByGuards() const;

			bool isSecurityLow() const { return true;}

			int getNumberOfValidSquare();
			void computeNumberOfValidSquare();

			void printOnFile(std::ofstream & _stream);

			bool isThereASink() const {return !(m_sinkCoordinate.col < 0);}
			std::vector<int> distanceFromNearestSink(std::vector<AgentPosition> const & _positions);
			std::vector<int> distanceFromNearestSink(std::vector< std::pair<AgentPosition, int> > const & _positions);
			
			Box getBoundingBox() const;
    
			double getDistance() const;

		protected:
			void addEdges();

			friend class  LearningAlgorithm;
			friend class  DISLAlgorithm;
			friend class  PIPIPAlgorithm;
			friend class  ParetoEfficientAlgorithm;
			friend class  CoarseCorrelatedAlgorithm;
		};

		typedef std::shared_ptr<DiscretizedArea> DiscretizedAreaPtr;
	}
}
#endif