#include <algorithm>  // std::reverse
#include <cmath>      // pow, sqrt
#include <iostream>   // std::cerr
#include <queue>      // std::priority_queue
#include <set>        // std::set

#include "ramp/RASP.h"

namespace RAMP
{

RASP::RASP(const ompl::base::SpaceInformationPtr &si):
    ompl::base::Planner(si, "RASP"),
    heuristic_([](Vertex _){ return 0.0; }),
    ignoreRisk_(false)
{
    Planner::declareParam<std::string>("roadmap_file", this,
                                       &RASP::setRoadmap, &RASP::getRoadmap);
    Planner::declareParam<bool>("ignore_risk", this,
                                &RASP::setIgnoreRisk, &RASP::getIgnoreRisk);
}

RASP::~RASP()
{
}

/*! \brief Set start and goal state. Initialize v_start_ and v_goal_.
 *
 *  This snaps the start and goal states to the nearest vertex that
 *  already exists in the graph.
 */
void RASP::setProblemDefinition(const ompl::base::ProblemDefinitionPtr &pdef)
{
    ompl::base::Planner::setProblemDefinition(pdef);

    ompl::base::StateSpacePtr space(si_->getStateSpace());

    // Assume one start and one goal
    StateWrapperPtr start_state(new StateWrapper(space));
    space->copyState(start_state->state, pdef_->getStartState(0));

    StateWrapperPtr goal_state(new StateWrapper(space));
    space->copyState(goal_state->state, pdef_->getGoal()->as<ompl::base::GoalState>()->getState());

    // Snap start/goal states to nearest vertex
    double best_start_dist = std::numeric_limits<double>::max();
    double best_goal_dist = std::numeric_limits<double>::max();
    Vertex start_vertex;
    Vertex goal_vertex;

    VertexIter vi, vi_end;
    for (boost::tie(vi, vi_end) = vertices(g_); vi != vi_end; ++vi)
    {
        double startDist = space->distance(g_[*vi].v_state->state,
                                           start_state->state);
        double goalDist  = space->distance(g_[*vi].v_state->state,
                                           goal_state->state);

        if (startDist < best_start_dist)
        {
            best_start_dist = startDist;
            start_vertex = *vi;
        }
        if (goalDist < best_goal_dist)
        {
            best_goal_dist = goalDist;
            goal_vertex = *vi;
        }
    }

    v_start_ = start_vertex;
    v_goal_  = goal_vertex;
}

/*! \brief Initialize remaining state properties: visited, raspList.
 *
 */
void RASP::setup()
{
    ompl::base::Planner::setup();

    VPVisitedMap visitedMap = get(&VProp::visited, g_);
    VPRASPListMap raspListMap = get(&VProp::raspList, g_);
    double inf = std::numeric_limits<double>::infinity();

    VertexIter vi, vi_end;
    for (boost::tie(vi, vi_end) = vertices(g_); vi != vi_end; ++vi)
    {
        // Set visited to false
        visitedMap[*vi] = false;

        // Set raspList to new RASP list
        RASPList<Vertex> *raspList = new RASPList<Vertex>();
        if (*vi == v_start_)
            raspList->emplace_back(0, 0, 0, 0);
        else
            raspList->emplace_back(inf, inf, 0, 0);
        raspListMap[*vi] = *raspList;
    }
}

/*! \brief Return A* cost (cost to come + estimated cost to go).
 *
 */
double RASP::estimateCost(const Vertex &v)
{
    return g_[v].raspList[0].cost() + heuristic_(v);
}

/*! \brief Return the boundary point between u and v.
 *
 *  Returns the midpoint between u and v.
 */
StateWrapperPtr RASP::computeBoundaryState(const Vertex &u, const Vertex &v)
{
    unsigned int dim = si_->getStateDimension();

    double *uVals = g_[u].v_state->state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
    double *vVals = g_[v].v_state->state->as<ompl::base::RealVectorStateSpace::StateType>()->values;

    StateWrapperPtr boundaryPoint(new StateWrapper(si_->getStateSpace()));
    double *bVals = boundaryPoint->state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
    for (unsigned int i = 0; i < dim; i++)
    {
        bVals[i] = 0.5 * (uVals[i] + vVals[i]);
    }
    return boundaryPoint;
}

/*! \brief Return the distances to the safe-risk boundary between u and v.
 *
 */
std::pair<double, double> RASP::computeBoundary(const Vertex &u, const Vertex &v)
{
    StateWrapperPtr boundaryPoint = computeBoundaryState(u, v);
    return computeBoundary(u, v, boundaryPoint);
}

std::pair<double, double> RASP::computeBoundary(
    const Vertex &u, const Vertex &v, const StateWrapperPtr &boundaryPoint)
{
    Edge uv;
    bool edge_exists;
    boost::tie(uv, edge_exists) = edge(u, v, g_);

    double uDist = si_->distance(g_[u].v_state->state, boundaryPoint->state);
    double vDist = si_->distance(g_[v].v_state->state, boundaryPoint->state);

    return std::pair<double, double>(uDist, vDist);
}

/*! \brief Implement the RASP planner.
 *
 *  Currently ignores the solveTime argument.
 */
ompl::base::PlannerStatus RASP::solve(double solveTime)
{
    size_t solverTime = info.start();
    auto cmp = [&](Vertex left, Vertex right)
    {
        return estimateCost(left) < estimateCost(right);
    };

    std::multiset<Vertex, decltype(cmp)> queue(cmp);
    queue.insert(v_start_);

    while (!queue.empty())
    {
        Vertex u = *queue.begin();
        queue.erase(queue.begin());

        if (g_[u].stateType == StateType::FORB || g_[u].visited)
            continue;


        if (u == v_goal_)
            break;

        // v will need to be updated later, so remove it now before mutating
        // anything that could change its priority
        NeighborIter ni, ni_end;
        for (boost::tie(ni, ni_end) = adjacent_vertices(u, g_); ni != ni_end; ++ni)
        {
            Vertex v = *ni;
            auto its = queue.equal_range(v);
            for (auto it = its.first; it != its.second; ++it)
            {
                if (*it == v)
                {
                    queue.erase(it);
                    break;
                }
            }
        }

        for (boost::tie(ni, ni_end) = adjacent_vertices(u, g_); ni != ni_end; ++ni)
        {
            Vertex v = *ni;

            if (g_[v].stateType == StateType::FORB)
                continue;
            if (g_[v].visited)
                continue;

            Edge uv;
            bool edge_exists;
            boost::tie(uv, edge_exists) = edge(u, v, g_);
            double edge_length = g_[uv].length;

            if (ignoreRisk_ ||
                (g_[u].stateType == StateType::SAFE &&
                 g_[v].stateType == StateType::SAFE))
            {
                RASPEntry<Vertex> u_entry = g_[u].raspList[0];
                RASPEntry<Vertex> v_entry = g_[v].raspList[0];

                if (u_entry.cost() + edge_length < v_entry.cost())
                {
                    g_[v].raspList.erase(g_[v].raspList.begin());
                    g_[v].raspList.emplace_back(
                        u_entry.cost() + edge_length,
                        u_entry.time() + edge_length,
                        0, u);
                }
            }
            else if (g_[u].stateType == StateType::SAFE &&
                     g_[v].stateType == StateType::RISK)
            {
                std::pair<double, double> dists = computeBoundary(u, v); // safe dist, risk dist

                RASPList<Vertex> entries_from_u;
                for (RASPEntry<Vertex> u_entry : g_[u].raspList)
                {
                    entries_from_u.emplace_back(
                        u_entry.cost() + dists.first + exp(dists.second) - 1,
                        u_entry.time() + edge_length,
                        dists.second, u);
                }

                RASPList<Vertex> merged(g_[v].raspList.size() + entries_from_u.size());
                std::merge(entries_from_u.begin(), entries_from_u.end(),
                           g_[v].raspList.begin(), g_[v].raspList.end(),
                           merged.begin());
                g_[v].raspList = merged;
            }
            else if (g_[u].stateType == StateType::RISK &&
                     g_[v].stateType == StateType::RISK)
            {
                RASPList<Vertex> entries_from_u;
                for (RASPEntry<Vertex> u_entry : g_[u].raspList)
                {
                    entries_from_u.emplace_back(
                        u_entry.cost() + exp(u_entry.risk_time()) * (exp(edge_length) - 1),
                        u_entry.time() + edge_length,
                        u_entry.risk_time() + edge_length, u);
                }
                std::sort(entries_from_u.begin(), entries_from_u.end());

                RASPList<Vertex> merged(g_[v].raspList.size() + entries_from_u.size());
                std::merge(entries_from_u.begin(), entries_from_u.end(),
                           g_[v].raspList.begin(), g_[v].raspList.end(),
                           merged.begin());
                g_[v].raspList = merged;
            }
            else
            {
                std::pair<double, double> dists = computeBoundary(u, v); // risk dist, safe dist

                RASPList<Vertex> entries_from_u;
                for (RASPEntry<Vertex> u_entry : g_[u].raspList)
                {
                    entries_from_u.emplace_back(
                        u_entry.cost() + exp(u_entry.risk_time()) * (exp(dists.first) - 1) + dists.second,
                        u_entry.time() + edge_length,
                        0, u);
                }
                std::sort(entries_from_u.begin(), entries_from_u.end());

                RASPList<Vertex> merged(g_[v].raspList.size() + entries_from_u.size());
                std::merge(entries_from_u.begin(), entries_from_u.end(),
                           g_[v].raspList.begin(), g_[v].raspList.end(),
                           merged.begin());
                merged.erase(merged.begin() + 1, merged.end());
                g_[v].raspList = merged;
            }

            validate(g_[v].raspList);
            queue.insert(v);
        }

        g_[u].visited = true;
    }

    pdef_->addSolutionPath(constructSolution(v_start_, v_goal_));
    info.stop(solverTime);

    OMPL_INFORM("Runtime: %g seconds", info.runtime(solverTime));

    return ompl::base::PlannerStatus::EXACT_SOLUTION;
}

ompl::base::PlannerStatus RASP::solve(const ompl::base::PlannerTerminationCondition &ptc)
{
    return RASP::solve(1.0);
}

/*! \brief Validate the given RASP list by removing entries that won't be used.
 *
 */
void RASP::validate(RASPList<Vertex> &raspList)
{
    int length = raspList.size();
    int i = 0;

    while (i < length - 1)
    {
        if (raspList[i].risk_time() <= raspList[i+1].risk_time())
        {
            raspList.erase(raspList.begin() + i+1);
            --length;
        }
        else
        {
            ++i;
        }
    }
}

/*! \brief Return the solution path by starting at the goal and following
 *  pointers back to the start.
 */
ompl::base::PathPtr RASP::constructSolution(const Vertex &start, const Vertex &goal)
{
    std::set<Vertex> seen;

    ompl::geometric::PathGeometric *path =
        new ompl::geometric::PathGeometric(si_);
    RASPEntry<Vertex> entry;
    Vertex v = goal;

    while (v != start)
    {
        if (seen.find(v) != seen.end())
        {
            OMPL_ERROR("infinite loop");
            break;
        }

        seen.insert(v);

        path->append(g_[v].v_state->state);
        entry = g_[v].raspList[0];
        v = entry.parent();
    }
    if (v == start)
    {
        path->append(g_[start].v_state->state);
    }
    path->reverse();

    // Goal information
    RASPEntry<Vertex> goal_entry = g_[goal].raspList[0];
    OMPL_INFORM("Total cost: %f", goal_entry.cost());
    OMPL_INFORM("Total length: %f", goal_entry.time());

    return ompl::base::PathPtr(path);
}

void RASP::setHeuristic(std::function<double(Vertex)> h)
{
    heuristic_ = h;
}

void RASP::setRoadmap(std::string graphFile)
{
    fileRoadmapPtr_ = boost::shared_ptr<
        RoadmapFromFile<Graph, VPStateMap, VPStateTypeMap, StateWrapper, EPLengthMap>>
        (new RoadmapFromFile<Graph, VPStateMap, VPStateTypeMap, StateWrapper, EPLengthMap>
         (si_->getStateSpace(), graphFile));
    fileRoadmapPtr_->generate(g_,
                              get(&VProp::v_state, g_),
                              get(&VProp::stateType, g_),
                              get(&EProp::length, g_));

    OMPL_INFORM("Graph has %d vertices", num_vertices(g_));
}

std::string RASP::getRoadmap()
{
    return fileRoadmapPtr_->filename;
}

} // namespace RAMP
