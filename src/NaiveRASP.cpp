#include <algorithm>  // std::reverse
#include <cmath>      // pow, sqrt
#include <iostream>   // std::cerr
#include <queue>      // std::priority_queue
#include <set>        // std::set

#include "ramp/RASP.h"
#include "ramp/NaiveRASP.h"
#include "GraphUtils.h"

namespace RAMP
{

NaiveRASP::NaiveRASP(const ompl::base::SpaceInformationPtr &si):
    ompl::base::Planner(si, "RASP")
{
    Planner::declareParam<bool>("ignore_risk", this,
                                &NaiveRASP::setIgnoreRisk, &NaiveRASP::getIgnoreRisk);
    Planner::declareParam<std::string>("roadmap_file", this,
                                       &NaiveRASP::setRoadmap, &NaiveRASP::getRoadmap);
}

NaiveRASP::~NaiveRASP()
{
}

/*! \brief Set start and goal state. Initialize v_start_ and v_goal_.
 *
 *  This currently snaps the start and goal states to the nearest vertex that
 *  already exists in the graph.
 *
 *  This is copied directly from RASP::setProblemDefinition. Changes should
 *  be made in both places for now.
 */
void NaiveRASP::setProblemDefinition(const ompl::base::ProblemDefinitionPtr &pdef)
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

/*! \brief Nothing for now.
 *
 */
void NaiveRASP::setup()
{
    ompl::base::Planner::setup();
}

/*! \brief Return the riskDists to the safe-risk boundary between u and v.
 *
 *  Returns the midpoint between u and v.
 *
 *  This is copied directly from RASP::computeBoundaryState. Changes should be
 *  made in both places for now.
 */
StateWrapperPtr NaiveRASP::computeBoundaryState(const Vertex &u, const Vertex &v)
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
 *  This is copied directly from RASP::computeBoundary. Changes should be made
 *  in both places for now.
 */
std::pair<double, double> NaiveRASP::computeBoundary(const Vertex &u, const Vertex &v)
{
    StateWrapperPtr boundaryPoint = computeBoundaryState(u, v);
    return computeBoundary(u, v, boundaryPoint);
}

std::pair<double, double> NaiveRASP::computeBoundary(
    const Vertex &u, const Vertex &v, const StateWrapperPtr &boundaryPoint)
{
    Edge uv;
    bool edge_exists;
    boost::tie(uv, edge_exists) = edge(u, v, g_);

    double uDist = si_->distance(g_[u].v_state->state, boundaryPoint->state);
    double vDist = si_->distance(g_[v].v_state->state, boundaryPoint->state);

    return std::pair<double, double>(uDist, vDist);
}

void NaiveRASP::constructRiskAndSafeGraphs()
{
    VertexIter vi, vi_end;
    for (boost::tie(vi, vi_end) = vertices(g_); vi != vi_end; ++vi)
    {
        if (g_[*vi].stateType == StateType::RISK)
        {

            Vertex vRisk = add_vertex(gRisk_);
            gRisk_[vRisk].v_state = g_[*vi].v_state;
            gRisk_[vRisk].stateType = StateType::RISK;

            gAndgRiskVertex_.left.insert(std::make_pair(*vi, vRisk));
            gAndgRiskVertex_.right.insert(std::make_pair(vRisk, *vi));
        }
        if (g_[*vi].stateType == StateType::SAFE)
        {

            Vertex vSafe = add_vertex(gSafe_);
            gSafe_[vSafe].v_state = g_[*vi].v_state;
            gSafe_[vSafe].stateType = StateType::SAFE;

            gAndgSafeVertex_.left.insert(std::make_pair(*vi, vSafe));
            gAndgSafeVertex_.right.insert(std::make_pair(vSafe, *vi));
        }
    }

    EdgeIter ei, ei_end;
    for (boost::tie(ei, ei_end) = edges(g_); ei != ei_end; ++ei)
    {
        Vertex u = source(*ei, g_);
        Vertex v = target(*ei, g_);
        StateType uState = g_[u].stateType;
        StateType vState = g_[v].stateType;

        if (uState == StateType::FORB || vState == StateType::FORB)
            continue;
        if (uState != vState)
        {
            addBorderVertexAndEdge(*ei, u, v, uState, vState);
        }
        else if (uState == StateType::RISK && vState == StateType::RISK)
        {
            Edge eRisk;
            bool added;
            boost::tie(eRisk, added) = add_edge(
                gAndgRiskVertex_.left.at(u), gAndgRiskVertex_.left.at(v), gRisk_);
            gRisk_[eRisk].length = g_[*ei].length;
        }
        else if (uState == StateType::SAFE && vState == StateType::SAFE)
        {
            Edge eSafe;
            bool added;
            boost::tie(eSafe, added) = add_edge(
                gAndgSafeVertex_.left.at(u), gAndgSafeVertex_.left.at(v), gSafe_);
            gSafe_[eSafe].length = g_[*ei].length;
        }
    }

    OMPL_INFORM("Orig: %d vertices, %d edges", num_vertices(g_), num_edges(g_));
    OMPL_INFORM("Risk: %d vertices, %d edges", num_vertices(gRisk_), num_edges(gRisk_));
    OMPL_INFORM("Safe: %d vertices, %d edges", num_vertices(gSafe_), num_edges(gSafe_));
}

void NaiveRASP::addBorderVertexAndEdge(Edge e, Vertex u, Vertex v,
                                       StateType uState, StateType vState)
{
    StateWrapperPtr bStatePtr = computeBoundaryState(u, v);
    std::pair<double, double> dists = computeBoundary(u, v, bStatePtr);

    Vertex bRisk = add_vertex(gRisk_);
    borderEdgeToRiskVertex_.insert(std::make_pair(e, bRisk));
    gRisk_[bRisk].v_state = bStatePtr;
    gRisk_[bRisk].stateType = StateType::RISK;

    Vertex bSafe = add_vertex(gSafe_);
    borderEdgeToSafeVertex_.insert(std::make_pair(e, bSafe));
    gSafe_[bSafe].v_state = bStatePtr;
    gSafe_[bSafe].stateType = StateType::SAFE;

    borderEdgeToState_[e] = bStatePtr;
    borderEdgeToRiskVertex_[e] = bRisk;
    borderEdgeToSafeVertex_[e] = bSafe;
    riskAndSafeBorderVertex_.left.insert(std::make_pair(bRisk, bSafe));
    riskAndSafeBorderVertex_.right.insert(std::make_pair(bSafe, bRisk));

    Edge eRisk, eSafe;
    bool added;
    if (uState == StateType::RISK && vState == StateType::SAFE)
    {
        boost::tie(eRisk, added) = add_edge(
            gAndgRiskVertex_.left.at(u), bRisk, gRisk_);
        gRisk_[eRisk].length = dists.first;

        boost::tie(eSafe, added) = add_edge(
            gAndgSafeVertex_.left.at(v), bSafe, gSafe_);
        gSafe_[eSafe].length = dists.second;
    }
    if (uState == StateType::SAFE && vState == StateType::RISK)
    {
        boost::tie(eRisk, added) = add_edge(
            gAndgRiskVertex_.left.at(v), bRisk, gRisk_);
        gRisk_[eRisk].length = dists.second;

        boost::tie(eSafe, added) = add_edge(
            gAndgSafeVertex_.left.at(u), bSafe, gSafe_);
        gSafe_[eSafe].length = dists.first;
    }
}

/*! \brief Implement the naive RASP planner.
 *
 *  Currently ignores the solveTime argument.
 *  
 *  1. Compute risk-only and safe-only graphs
 *  2. Compute all-pairs shortest path on gRisk
 *  3. Add exponentiated weights to gSafe
 *  4. Compute shortest path between start and goal on gSafe
 *  5. Recover shortest path between start and goal
 */
ompl::base::PlannerStatus NaiveRASP::solve(double solveTime)
{
    size_t solverTime = info.start();
    constructRiskAndSafeGraphs();

    std::set<Vertex> riskBorderVertices;
    for (Bimap<Vertex>::left_map::const_iterator it = riskAndSafeBorderVertex_.left.begin();
         it != riskAndSafeBorderVertex_.left.end(); ++it)
    {
        riskBorderVertices.insert(it->first);
    }
    OMPL_INFORM("%d border vertices", riskBorderVertices.size());

    // Took 340s for vikings and ~8 GB
    OMPL_INFORM("Computing Dijkstra's for all border points");
    size_t dijkstraTime = info.start();
    std::map<Vertex, std::map<Vertex, Vertex>> riskPreds;
    std::map<Vertex, std::map<Vertex, double>> riskDists;
    size_t count = 0;
    for (std::set<Vertex>::iterator it = riskBorderVertices.begin();
         it != riskBorderVertices.end(); ++it)
    {
        count++;
        if (count % 100 == 0)
            OMPL_INFORM("Running Dijkstra's for %dth border point", count);
        Vertex riskBorderVertex = *it;
        std::map<Vertex, Vertex> pred;
        std::map<Vertex, double> dist;
        try
        {
            dijkstra_shortest_paths(
                gRisk_, riskBorderVertex,
                boost::make_assoc_property_map(pred),
                boost::make_assoc_property_map(dist),
                get(&EProp::length, gRisk_), get(boost::vertex_index, gRisk_),
                std::less<double>(), // compare
                boost::closed_plus<double>(std::numeric_limits<double>::max()), // combine
                std::numeric_limits<double>::max(),
                double(),
                throw_when_all_visited<Graph, Vertex, Edge>(riskBorderVertices));
        } catch (const throw_visited_exception &ex) {}
        riskPreds[riskBorderVertex] = pred;

        // Only keep distances to border vertices (otherwise takes too much space)
        riskDists[riskBorderVertex] = std::map<Vertex, double>();
        for (std::set<Vertex>::iterator itt = riskBorderVertices.begin();
             itt != riskBorderVertices.end(); ++itt)
        {
            riskDists[riskBorderVertex][*itt] = dist[*itt];
        }
    }
    info.stop(dijkstraTime);
    OMPL_INFORM("Repeated Dijkstra's runtime: %g seconds", info.runtime(dijkstraTime));

    OMPL_INFORM("Adding border vertex edges to safe graph");
    for (auto const &uBorderState : borderEdgeToState_)
    {
        StateWrapperPtr uState = uBorderState.second;
        Vertex uRisk = borderEdgeToRiskVertex_[uBorderState.first];
        Vertex uSafe = borderEdgeToSafeVertex_[uBorderState.first];
        for (auto const &vBorderState : borderEdgeToState_)
        {
            StateWrapperPtr vState = vBorderState.second;
            Vertex vRisk = borderEdgeToRiskVertex_[vBorderState.first];
            Vertex vSafe = borderEdgeToSafeVertex_[vBorderState.first];

            double riskDist = riskDists[uRisk][vRisk];
            if (riskDist == std::numeric_limits<double>::max())
                continue;

            Edge eSafe;
            bool added;
            boost::tie(eSafe, added) = add_edge(uSafe, vSafe, gSafe_);
            gSafe_[eSafe].length = exp(riskDist) - 1;
        }
    }

    std::map<Vertex, Vertex> safePreds;
    std::map<Vertex, double> safeDists;
    Vertex vSafeStart = gAndgSafeVertex_.left.at(v_start_);
    Vertex vSafeGoal = gAndgSafeVertex_.left.at(v_goal_);
    try
    {
        dijkstra_shortest_paths(
            gSafe_, vSafeStart,
            boost::make_assoc_property_map(safePreds),
            boost::make_assoc_property_map(safeDists),
            get(&EProp::length, gSafe_), get(boost::vertex_index, gSafe_),
            std::less<double>(), // compare
            boost::closed_plus<double>(std::numeric_limits<double>::max()), // combine
            std::numeric_limits<double>::max(),
            double(),
            throw_when_visited<Graph, Vertex, Edge>(vSafeGoal));
    } catch (const throw_visited_exception &ex) {}

    ompl::base::PathPtr path = constructSolution(
        v_start_, v_goal_, riskDists, riskPreds, safePreds);
    OMPL_INFORM("Total cost: %f", safeDists[vSafeGoal]);

    pdef_->addSolutionPath(path);
    info.stop(solverTime);

    OMPL_INFORM("Runtime: %g seconds", info.runtime(solverTime));
    return ompl::base::PlannerStatus::EXACT_SOLUTION;
}

ompl::base::PlannerStatus NaiveRASP::solve(const ompl::base::PlannerTerminationCondition &ptc)
{
    return NaiveRASP::solve(1.0);
}

/*! \brief Return the solution path by starting at the goal and following
 *  pointers back to the start.
 */
ompl::base::PathPtr NaiveRASP::constructSolution(
    const Vertex &start, const Vertex &goal,
    std::map<Vertex, std::map<Vertex, double>> &riskDists,
    std::map<Vertex, std::map<Vertex, Vertex>> &riskPreds,
    std::map<Vertex, Vertex> &safePreds)
{
    Vertex vSafeStart = gAndgSafeVertex_.left.at(start);
    Vertex vSafeGoal = gAndgSafeVertex_.left.at(goal);

    std::set<Vertex> safeSeen;
    ompl::geometric::PathGeometric *path =
        new ompl::geometric::PathGeometric(si_);

    Vertex vSafe = vSafeGoal;
    while (vSafe != vSafeStart)
    {
        if (safeSeen.find(vSafe) != safeSeen.end())
        {
            OMPL_ERROR("infinite loop in safe loop");
            break;
        }

        safeSeen.insert(vSafe);

        if (gAndgSafeVertex_.right.find(vSafe) == gAndgSafeVertex_.right.end())
        {
            // vSafe/vRisk is a border point, look through riskDists until vRiskStart
            Vertex vRiskGoal = riskAndSafeBorderVertex_.right.at(vSafe);
            Vertex vRiskStart = riskAndSafeBorderVertex_.right.at(safePreds[vSafe]);
            std::set<Vertex> riskSeen;

            // pred to vRisk minimizes cost(vRisk, neigh) + dist(neigh, vRiskGoal)
            Vertex vRisk = vRiskGoal;
            while (vRisk != vRiskStart)
            {
                if (riskSeen.find(vRisk) != riskSeen.end())
                {
                    OMPL_ERROR("infinite loop in risk loop");
                    break;
                }

                riskSeen.insert(vRisk);

                Vertex vRiskPrev = riskPreds[vRiskStart][vRisk];

                if (vRiskPrev != vRiskStart && vRiskPrev != vRiskGoal)
                {
                    // vRiskPrev is a risk point in the original graph
                    Vertex v = gAndgRiskVertex_.right.at(vRiskPrev);
                    path->append(g_[v].v_state->state);
                }
                vRisk = vRiskPrev;
            }

            // vSafe is border point on one side of the risk region
            // pred(vSafe) is border point on other side of risk region
            // pred(pred(vSafe)) was safe point in original graph
            vSafe = safePreds[safePreds[vSafe]];
            if (vSafe == vSafeStart)
                break;
        }

        // vSafe is a safe point in the original graph
        Vertex v = gAndgSafeVertex_.right.at(vSafe);
        path->append(g_[v].v_state->state);

        vSafe = safePreds[vSafe];
    }
    if (vSafe == vSafeStart)
        path->append(g_[start].v_state->state);
    path->reverse();

    return ompl::base::PathPtr(path);
}

void NaiveRASP::setRoadmap(std::string graphFile)
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

std::string NaiveRASP::getRoadmap()
{
    return fileRoadmapPtr_->filename;
}

} // namespace RAMP
