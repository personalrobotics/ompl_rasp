#ifndef RASP_ROADMAPFROMFILE_H_
#define RASP_ROADMAPFROMFILE_H_

#include <iostream>
#include <string>
#include <sstream>
#include <fstream>

#include <boost/property_map/dynamic_property_map.hpp>
#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/graph/graphml.hpp>
#include <boost/graph/adjacency_list.hpp>

#include <ompl/base/State.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include "ramp/StateTypes.h"

namespace RAMP {

/* RoadmapFromFilePutStateMap */

template <class PropMap, class StateWrapper>
class RoadmapFromFilePutStateMap
{
public:
    typedef boost::writable_property_map_tag category;
    typedef typename boost::property_traits<PropMap>::key_type key_type;
    typedef std::string value_type;
    typedef std::string reference;
    const PropMap prop_map;
    ompl::base::StateSpacePtr space;
    const size_t dim;

    RoadmapFromFilePutStateMap(PropMap prop_map, ompl::base::StateSpacePtr _space, size_t dim):
        prop_map(prop_map), space(_space), dim(dim)
    {
    }

};

template <class PropMap, class StateWrapper>
inline std::string
get(const RoadmapFromFilePutStateMap<PropMap,StateWrapper> &map,
    const typename RoadmapFromFilePutStateMap<PropMap,StateWrapper>::key_type &k)
{
    abort();
}


template <class PropMap, class StateWrapper>
inline void
put(const RoadmapFromFilePutStateMap<PropMap,StateWrapper> &map,
    const typename RoadmapFromFilePutStateMap<PropMap,StateWrapper>::key_type &k,
    const std::string repr)
{
    get(map.prop_map, k).reset(new StateWrapper(map.space));
    ompl::base::State *ver_state = get(map.prop_map, k)->state;
    double *values = ver_state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
    std::stringstream ss(repr);
    for (size_t ui = 0; ui < map.dim; ui++)
        ss >> values[ui];
}

/* RoadmapFromFilePutStateTypeMap */

template <class PropMap, class StateWrapper>
class RoadmapFromFilePutStateTypeMap
{
public:
    typedef boost::writable_property_map_tag category;
    typedef typename boost::property_traits<PropMap>::key_type key_type;
    typedef std::string value_type;
    typedef std::string reference;
    const PropMap prop_map;

    RoadmapFromFilePutStateTypeMap(PropMap prop_map):
        prop_map(prop_map)
    {
    }

};

template <class PropMap, class StateWrapper>
inline std::string
get(const RoadmapFromFilePutStateTypeMap<PropMap,StateWrapper> &map,
    const typename RoadmapFromFilePutStateTypeMap<PropMap,StateWrapper>::key_type &k)
{
    abort();
}


template <class PropMap, class StateWrapper>
inline void
put(const RoadmapFromFilePutStateTypeMap<PropMap,StateWrapper> &map,
    const typename RoadmapFromFilePutStateTypeMap<PropMap,StateWrapper>::key_type &k,
    const std::string repr)
{
    if (repr == "0")
        put(map.prop_map, k, StateType::FREE);
    if (repr == "1")
        put(map.prop_map, k, StateType::FORB);
    if (repr == "2")
        put(map.prop_map, k, StateType::RISK);
    if (repr == "3")
        put(map.prop_map, k, StateType::SAFE);
}

/* RoadmapFromFilePutEdgeLengthMap */

template <class PropMap>
class RoadmapFromFilePutEdgeLengthMap
{
public:
    typedef boost::writable_property_map_tag category;
    typedef typename boost::property_traits<PropMap>::key_type key_type;
    typedef std::string value_type;
    typedef std::string reference;
    const PropMap prop_map;

    RoadmapFromFilePutEdgeLengthMap(PropMap prop_map):
        prop_map(prop_map)
    {
    }

};

template <class PropMap>
inline std::string
get(const RoadmapFromFilePutEdgeLengthMap<PropMap> &map,
    const typename RoadmapFromFilePutEdgeLengthMap<PropMap>::key_type &k)
{
    abort();
}


template <class PropMap>
inline void
put(const RoadmapFromFilePutEdgeLengthMap<PropMap> &map,
    const typename RoadmapFromFilePutEdgeLengthMap<PropMap>::key_type &k,
    const std::string repr)
{
    put(map.prop_map, k, stod(repr));
}


/* RoadmapFromFile */

template <class Graph, class VStateMap, class VStateTypeMap, class StateWrapper, class EDistance>
class RoadmapFromFile
{
    typedef boost::graph_traits<Graph> GraphTypes;
    typedef typename GraphTypes::vertex_descriptor Vertex;
    typedef typename GraphTypes::vertex_iterator VertexIter;
    typedef typename GraphTypes::edge_descriptor Edge;
    typedef typename GraphTypes::edge_iterator EdgeIter;

public:

    const std::string filename;

    RoadmapFromFile(const ompl::base::StateSpacePtr _space, std::string _filename):
        space(_space), filename(_filename), bounds(0)
    {
        if (space->getType() != ompl::base::STATE_SPACE_REAL_VECTOR)
            throw std::runtime_error("This only supports real vector state spaces!");
        dim = space->getDimension();
        bounds = space->as<ompl::base::RealVectorStateSpace>()->getBounds();
    }

    ~RoadmapFromFile() {}

    void generate(Graph &g, VStateMap state_map, VStateTypeMap state_type_map, EDistance distance_map)
    {
        boost::dynamic_properties props;
        props.property("state",
                       RoadmapFromFilePutStateMap<VStateMap, StateWrapper>(state_map, space, dim));
        props.property("stateType",
                       RoadmapFromFilePutStateTypeMap<VStateTypeMap, StateWrapper>(state_type_map));
        props.property("length",
                       RoadmapFromFilePutEdgeLengthMap<EDistance>(distance_map));

        std::ifstream fp;
        fp.open(filename.c_str());
        boost::read_graphml(fp, g, props);
        fp.close();

        EdgeIter ei, ei_end;
        for (boost::tie(ei, ei_end) = edges(g); ei != ei_end; ++ei)
        {
            ompl::base::State *state1 = get(state_map, source(*ei, g))->state;
            ompl::base::State *state2 = get(state_map, target(*ei, g))->state;
            if (get(distance_map, *ei) == 0)
                put(distance_map, *ei, space->distance(state1, state2));
        }
    }

private:
    size_t dim;
    ompl::base::RealVectorBounds bounds;
    const ompl::base::StateSpacePtr space;
};

} // namespace RAMP

#endif // RASP_ROADMAPFROMFILE_H_
