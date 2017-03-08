class throw_visited_exception: public std::exception {};

template <typename Graph, typename Vertex, typename Edge>
class throw_when_visited
{
public:
    throw_when_visited(Vertex v_throw): v_throw(v_throw) {}
    inline void initialize_vertex(Vertex u, const Graph & g) {}
    inline void discover_vertex(Vertex u, const Graph & g) {}
    inline void examine_vertex(Vertex u, const Graph & g)
    {
        if (u == v_throw)
            throw throw_visited_exception();
    }
    inline void examine_edge(Edge e, const Graph & g) {}
    inline void edge_relaxed(Edge e, const Graph & g) {}
    inline void edge_not_relaxed(Edge e, const Graph & g) {}
    inline void black_target(Edge e, const Graph & g) {}
    inline void finish_vertex(Vertex u, const Graph & g) {}

protected:
    Vertex v_throw;

};

template <typename Graph, typename Vertex, typename Edge>
class throw_when_all_visited
{
public:
    throw_when_all_visited(std::set<Vertex> vs): vs(vs) {}
    inline void initialize_vertex(Vertex u, const Graph & g) {}
    inline void discover_vertex(Vertex u, const Graph & g) {}
    inline void examine_vertex(Vertex u, const Graph & g)
    {
        vs.erase(u);
        if (vs.empty())
            throw throw_visited_exception();
    }
    inline void examine_edge(Edge e, const Graph & g) {}
    inline void edge_relaxed(Edge e, const Graph & g) {}
    inline void edge_not_relaxed(Edge e, const Graph & g) {}
    inline void black_target(Edge e, const Graph & g) {}
    inline void finish_vertex(Vertex u, const Graph & g) {}

protected:
    std::set<Vertex> vs;

};
