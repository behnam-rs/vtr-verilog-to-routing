#pragma once

#include "connection_router.h"
#include "router_stats.h"

#include <fstream>
#include <thread>

#ifdef VPR_USE_TBB
#    include <tbb/concurrent_vector.h>
#endif

/** Print output from PartitionTreeDebug? */
#define DEBUG_PARTITION_TREE

/** Routing iteration results per thread. (for a subset of the input netlist) */
struct RouteIterResults {
    /** Are there any connections impossible to route due to a disconnected rr_graph? */
    bool is_routable = true;
    /** Net IDs for which timing_driven_route_net() actually got called */
    std::vector<ParentNetId> rerouted_nets;
    /** RouterStats collected from my subset of nets */
    RouterStats stats;
};

/** Spatial partition tree for routing.
 *
 * This divides the netlist into a tree of regions, so that nets with non-overlapping
 * bounding boxes can be routed in parallel.
 *
 * Branch nodes represent a cutline and their nets vector includes only the nets intersected
 * by the cutline. Leaf nodes represent a final set of nets reached by partitioning.
 *
 * To route this in parallel, we first route the nets in the root node, then add
 * its left and right to a task queue, and repeat this for the whole tree.
 * 
 * The tree stores some routing results to be later combined, such as is_routable and
 * rerouted_nets. (TODO: do this per thread instead of per node) */
class PartitionTreeNode {
  public:
    /** Nets claimed by this node (intersected by cutline if branch, nets in final region if leaf) */
    std::vector<ParentNetId> nets;
    /** Left subtree. */
    std::unique_ptr<PartitionTreeNode> left = nullptr;
    /** Right subtree. */
    std::unique_ptr<PartitionTreeNode> right = nullptr;
    /** Are there any connections impossible to route due to a disconnected rr_graph? */
    bool is_routable = false;
    /** Net IDs for which timing_driven_route_net() actually got called */
    std::vector<ParentNetId> rerouted_nets;

    /* debug stuff */
    typedef enum { X,
                   Y } Axis;
    Axis cutline_axis = X;
    int cutline_pos = -1;
};

/** Holds the root PartitionTreeNode and exposes top level operations. */
class PartitionTree {
  public:
    PartitionTree() = delete;
    PartitionTree(const PartitionTree&) = delete;
    PartitionTree(PartitionTree&&) = default;
    PartitionTree& operator=(const PartitionTree&) = delete;
    PartitionTree& operator=(PartitionTree&&) = default;

    /** Can only be built from a netlist */
    PartitionTree(const Netlist<>& netlist);

    /** Access root. Shouldn't cause a segfault, because PartitionTree constructor always makes a _root */
    inline PartitionTreeNode& root(void) { return *_root; }

  private:
    std::unique_ptr<PartitionTreeNode> _root;
    std::unique_ptr<PartitionTreeNode> build_helper(const Netlist<>& netlist, const std::vector<ParentNetId>& nets, int x1, int y1, int x2, int y2);
};

#ifdef DEBUG_PARTITION_TREE
/** Log PartitionTree-related messages. Can handle multiple threads. */
class PartitionTreeDebug {
  public:
#    ifdef VPR_USE_TBB
    static inline tbb::concurrent_vector<std::string> lines;
#    else
    static inline std::vector<std::string> lines;
#    endif
    /** Add msg to the log buffer (with a thread ID header) */
    static inline void log(std::string msg) {
        auto thread_id = std::hash<std::thread::id>()(std::this_thread::get_id());
        lines.push_back("[thread " + std::to_string(thread_id) + "] " + msg);
    }
    /** Write out the log buffer into a file */
    static inline void write(std::string filename) {
        std::ofstream f(filename);
        for (auto& line : lines) {
            f << line << std::endl;
        }
        f.close();
    }
};
#else
class PartitionTreeDebug {
  public:
    static inline void log(std::string /* msg */) {}
    static inline void write(std::string /* filename */) {}
};
#endif
