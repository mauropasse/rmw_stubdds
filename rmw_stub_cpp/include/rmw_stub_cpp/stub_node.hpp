#ifndef STUB_NODE_HPP_
#define STUB_NODE_HPP_

class StubNode
{
public:
  StubNode()
  {
    graph_guard_condition = new rmw_guard_condition_t;
  }

  ~StubNode()
  {
    delete graph_guard_condition;
  }

  rmw_guard_condition_t * get_node_graph_guard_condition()
  {
    return graph_guard_condition;
  }

private:
    rmw_guard_condition_t * graph_guard_condition{nullptr};
};

#endif  // STUB_NODE_HPP_
