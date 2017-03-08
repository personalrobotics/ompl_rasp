#ifndef RASP_H_
#define RASP_H_

#include <vector>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/properties.hpp>

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

template <class ParentType>
class RASPEntry
{
    double cost_;
    double time_;
    double risk_time_;
    ParentType parent_;

public:
    RASPEntry(double c, double t, double lambda, ParentType p):
        cost_{c},
        time_{t},
        risk_time_{lambda},
        parent_{p}
        {}
    RASPEntry(double c, double t, double lambda):
        cost_{c},
        time_{t},
        risk_time_{lambda}
        {}
    RASPEntry()
        {}

    double cost() const {return cost_;}
    double time() const {return time_;}
    double risk_time() const {return risk_time_;}
    ParentType parent() const {return parent_;}

    bool operator>(RASPEntry r) const {return cost_ > r.cost_;}
    bool operator<(RASPEntry r) const {return cost_ < r.cost_;}
};

template<typename P>
using RASPList = std::vector<RASPEntry<P>>;

class RASP: public ompl::base::Planner
{

public:
    struct VProp
    {
        StateWrapperPtr v_state;
        StateType stateType;
        bool visited;

        // HACK: relies on Vertex being an unsigned long int
        // The right way to do this might be to use exterior property maps
        // to add the raspList property after Vertex is defined
        RASPList<unsigned long int> raspList;
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
    typedef boost::property_map<Graph, bool VProp::*>::type VPVisitedMap;
    typedef boost::property_map<Graph, RASPList<Vertex> VProp::*>::type VPRASPListMap;
    typedef boost::property_map<Graph, boost::edge_index_t EProp::*>::type EdgeIndexMap;
    typedef boost::property_map<Graph, double EProp::*>::type EPLengthMap;

    // OMPL methods
    RASP(const ompl::base::SpaceInformationPtr &si);
    ~RASP(void);

    void setProblemDefinition(const ompl::base::ProblemDefinitionPtr &pdef);
    void setup();
    ompl::base::PlannerStatus solve(const ompl::base::PlannerTerminationCondition &ptc);
    ompl::base::PlannerStatus solve(double solveTime);

    // RASP methods
    void validate(RASPList<Vertex> &raspList);
    ompl::base::PathPtr constructSolution(const Vertex &start, const Vertex &goal);
    void setHeuristic(std::function<double(Vertex)> h);
    void setIgnoreRisk(bool ignoreRisk) { ignoreRisk_ = ignoreRisk; };
    bool getIgnoreRisk() { return ignoreRisk_; };

    // Member variables
    Graph g_;
    std::vector<RASPList<Vertex>> raspLists_;
    std::function<double(Vertex)> heuristic_;
    bool ignoreRisk_;
    Vertex v_start_;
    Vertex v_goal_;
    ExperimentInformation<Vertex, Edge> info;

    // Roadmap things
    boost::shared_ptr<RoadmapFromFile<Graph, VPStateMap, VPStateTypeMap, StateWrapper, EPLengthMap>> fileRoadmapPtr_;
    void setRoadmap(std::string graphFile);
    std::string getRoadmap();

protected:
    double estimateCost(const Vertex &v);
    StateWrapperPtr computeBoundaryState(const Vertex &u, const Vertex &v);
    std::pair<double, double> computeBoundary(const Vertex &u, const Vertex &v);
    std::pair<double, double> computeBoundary(
        const Vertex &u, const Vertex &v, const StateWrapperPtr &boundaryPoint);
};

} // namespace RAMP

#endif // RASP_H_
