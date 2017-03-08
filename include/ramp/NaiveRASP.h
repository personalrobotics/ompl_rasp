#ifndef NAIVE_RASP_H_
#define NAIVE_RASP_H_

#include <vector>

#include <boost/bimap.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/properties.hpp>
#include <boost/graph/exterior_property.hpp>
#include <boost/graph/floyd_warshall_shortest.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>

#include <ompl/base/Planner.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/PathGeometric.h>

#include "ExperimentInformation.h"
#include "StateTypes.h"
#include "StateWrapper.h"
#include "RoadmapFromFile.h"

namespace RAMP
{

class NaiveRASP: public ompl::base::Planner
{

public:
    struct VProp
    {
        StateWrapperPtr v_state;
        StateType stateType;
    };

    struct EProp
    {
        double length;
    };

    // Graph definitions
    typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS,
                                  VProp, EProp> Graph;
    typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
    typedef boost::graph_traits<Graph>::edge_descriptor Edge;
    typedef boost::graph_traits<Graph>::vertex_iterator VertexIter;
    typedef boost::graph_traits<Graph>::edge_iterator EdgeIter;
    typedef boost::graph_traits<Graph>::adjacency_iterator NeighborIter;

    // Property maps
    typedef boost::property_map<Graph, boost::vertex_index_t VProp::*>::type VertexIndexMap;
    typedef boost::property_map<Graph, StateWrapperPtr VProp::*>::type VPStateMap;
    typedef boost::property_map<Graph, StateType VProp::*>::type VPStateTypeMap;
    typedef boost::property_map<Graph, boost::edge_index_t EProp::*>::type EdgeIndexMap;
    typedef boost::property_map<Graph, double EProp::*>::type EPLengthMap;

    // Somewhat custom subgraph things
    template <typename T>
    using Bimap = boost::bimap<T, T>;

    // OMPL methods
    NaiveRASP(const ompl::base::SpaceInformationPtr &si);
    ~NaiveRASP(void);

    void setProblemDefinition(const ompl::base::ProblemDefinitionPtr &pdef);
    void setup();
    ompl::base::PlannerStatus solve(const ompl::base::PlannerTerminationCondition &ptc);
    ompl::base::PlannerStatus solve(double solveTime);
    ompl::base::PathPtr constructSolution(const Vertex &start, const Vertex &goal,
                                          std::map<Vertex, std::map<Vertex, double>> &riskDists,
                                          std::map<Vertex, std::map<Vertex, Vertex>> &riskPreds,
                                          std::map<Vertex, Vertex> &safePreds);

    // Member variables
    Graph g_;
    Vertex v_start_;
    Vertex v_goal_;
    ExperimentInformation<Vertex, Edge> info;

    // Naive fields and methods
    Graph gRisk_, gSafe_;
    Bimap<Vertex> gAndgRiskVertex_, gAndgSafeVertex_;
    // Bimap<Edge> gAndgRiskEdge_, gAndgSafeEdge_;
    Bimap<Vertex> riskAndSafeBorderVertex_;
    std::map<Edge, Vertex> borderEdgeToRiskVertex_, borderEdgeToSafeVertex_;
    std::map<Edge, StateWrapperPtr> borderEdgeToState_;
    void constructRiskAndSafeGraphs();
    void addBorderVertexAndEdge(Edge e, Vertex u, Vertex v, StateType uState, StateType vState);

    void setIgnoreRisk(bool ignoreRisk) {};
    bool getIgnoreRisk() {return false;};

    // Roadmap things
    boost::shared_ptr<RoadmapFromFile<Graph, VPStateMap, VPStateTypeMap, StateWrapper, EPLengthMap>> fileRoadmapPtr_;
    void setRoadmap(std::string graphFile);
    std::string getRoadmap();

protected:
    StateWrapperPtr computeBoundaryState(const Vertex &u, const Vertex &v);
    std::pair<double, double> computeBoundary(const Vertex &u, const Vertex &v);
    std::pair<double, double> computeBoundary(
        const Vertex &u, const Vertex &v, const StateWrapperPtr &boundaryPoint);
};

} // namespace RAMP

#endif // NAIVE_RASP_H_
