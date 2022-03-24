#include "partition_tree.h"
#include <memory>

PartitionTree::PartitionTree(const Netlist<>& netlist) {
    const auto& device_ctx = g_vpr_ctx.device();

    auto all_nets = std::vector<ParentNetId>(netlist.nets().begin(), netlist.nets().end());
    _root = build_helper(netlist, all_nets, 0, 0, device_ctx.grid.width(), device_ctx.grid.height());
}

std::unique_ptr<PartitionTreeNode> PartitionTree::build_helper(const Netlist<>& netlist, const std::vector<ParentNetId>& nets, int x1, int y1, int x2, int y2) {
    if (nets.empty())
        return nullptr;

    const auto& route_ctx = g_vpr_ctx.routing();
    auto out = std::make_unique<PartitionTreeNode>();

    /* Build ParaDRo-ish prefix sum lookup for each bin (coordinate) in the device.
     * Do this for every step with only given nets, because each cutline takes some nets out
     * of the game, so if we just built a global lookup it wouldn't yield accurate results.
     *
     * VPR's bounding boxes include the borders (see ConnectionRouter::timing_driven_expand_neighbour())
     * so try to include x=bb.xmax, y=bb.ymax etc. when calculating things. */
    int W = x2 - x1;
    int H = y2 - y1;
    VTR_ASSERT(W > 0 && H > 0);
    std::vector<int> x_total_before(W, 0), x_total_after(W, 0);
    std::vector<int> y_total_before(H, 0), y_total_after(H, 0);
    for (auto net_id : nets) {
        t_bb bb = route_ctx.route_bb[net_id];
        size_t fanouts = netlist.net_sinks(net_id).size();

        /* Start and end coords relative to x1. Clamp to [x1, x2]. */
        int x_start = std::max(x1, bb.xmin) - x1;
        int x_end = std::min(bb.xmax+1, x2) - x1;
        for(int x = x_start; x < W; x++){
            x_total_before[x] += fanouts;
        }
        for(int x = 0; x < x_end; x++){
            x_total_after[x] += fanouts;
        }
        int y_start = std::max(y1, bb.ymin) - y1;
        int y_end = std::min(bb.ymax+1, y2) - y1;
        for(int y = y_start; y < H; y++){
            y_total_before[y] += fanouts;
        }
        for(int y = 0; y < y_end; y++){
            y_total_after[y] += fanouts;
        }
    }

    int best_score = std::numeric_limits<int>::max();
    int best_pos = -1;
    PartitionTreeNode::Axis best_axis = PartitionTreeNode::X;

    int max_x_before = x_total_before[W-1];
    int max_x_after = x_total_after[0];
    for(int x=0; x<W; x++){
        int before = x_total_before[x];
        int after = x_total_after[x];
        if(before == max_x_before || after == max_x_after)  /* Cutting here would leave no nets to the left or right */
            continue;
        int score = abs(x_total_before[x] - x_total_after[x]);
        if (score < best_score) {
            best_score = score;
            best_pos = x1 + x; /* Lookups are relative to (x1, y1) */
            best_axis = PartitionTreeNode::X;
        }
    }

    int max_y_before = y_total_before[H-1];
    int max_y_after = y_total_after[0];
    for(int y=0; y<H; y++){
        int before = y_total_before[y];
        int after = y_total_after[y];
        if(before == max_y_before || after == max_y_after)  /* Cutting here would leave no nets to the left or right (sideways) */
            continue;
        int score = abs(y_total_before[y] - y_total_after[y]);
        if (score < best_score) {
            best_score = score;
            best_pos = y1 + y; /* Lookups are relative to (x1, y1) */
            best_axis = PartitionTreeNode::Y;
        }
    }

    /* Couldn't find a cutline: all cutlines result in a one-way cut */
    if (best_pos == -1) {
        out->nets = nets;  /* We hope copy elision is smart enough to optimize this stuff out */
        return out;
    }

    /* Populate net IDs on each side and call next level of build_x */
    std::vector<ParentNetId> left_nets, right_nets, my_nets;

    if (best_axis == PartitionTreeNode::X) {
        for (auto net_id : nets) {
            t_bb bb = route_ctx.route_bb[net_id];
            if (bb.xmin < best_pos && bb.xmax < best_pos) {
                left_nets.push_back(net_id);
            } else if (bb.xmin > best_pos && bb.xmax > best_pos) {
                right_nets.push_back(net_id);
            } else if (bb.xmin <= best_pos && bb.xmax >= best_pos) {
                my_nets.push_back(net_id);
            } else {
                VTR_ASSERT(false); /* unreachable */
            }
        }

        out->left = build_helper(netlist, left_nets, x1, y1, best_pos, y2);
        out->right = build_helper(netlist, right_nets, best_pos, y2, x2, y2);
    } else {
        VTR_ASSERT(best_axis == PartitionTreeNode::Y);
        for (auto net_id : nets) {
            t_bb bb = route_ctx.route_bb[net_id];
            if (bb.ymin < best_pos && bb.ymax < best_pos) {
                left_nets.push_back(net_id);
            } else if (bb.ymin > best_pos && bb.ymax > best_pos) {
                right_nets.push_back(net_id);
            } else if (bb.ymin <= best_pos && bb.ymax >= best_pos) {
                my_nets.push_back(net_id);
            } else {
                VTR_ASSERT(false); /* unreachable */
            }
        }

        out->left = build_helper(netlist, left_nets, x1, best_pos, x2, y2);
        out->right = build_helper(netlist, right_nets, x1, y1, x2, best_pos);
    }

    out->nets = my_nets;
    out->cutline_axis = best_axis;
    out->cutline_pos = best_pos;
    return out;
}
