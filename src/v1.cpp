#include "v1.h"

namespace planner { namespace v1 {

// =====================================================================================================================
// region State class
// =====================================================================================================================

State::State() = default;
State::State(const Location& loc_) : loc(loc_) {}

// endregion
// =====================================================================================================================
// region Trajectory class
// =====================================================================================================================

Trajectory::Trajectory() {
    pState = nullptr;
    pSubTraj = nullptr;
    val = -INF;
    las = 0.0f;
}

Trajectory::Trajectory(State* pState_, const Eigen::MatrixXf& cmap) : pState(pState_) {
    // Sub-trajectory.
    pSubTraj = nullptr;

    // Cost.
    const Location& loc = pState->loc;
    if ((loc.ki != -1) && (loc.kj != -1))
        val = cmap(loc.ki, loc.kj);
    else
        val = -INF;

    // Laser penalty.
    las = 0.0f;
}

Trajectory::Trajectory(State* pState_, Trajectory* pSubTraj_, const Eigen::MatrixXf& cmap) : Trajectory(pState_, cmap) {
    // Start Node : delegated.

    // Sub-trajectory.
    pSubTraj = pSubTraj_;

    // Uncertainty.
    // Initialized from delegation.
    val += pSubTraj->val;

    // Laser angle penalty : sum of squares of laser angle changes.
    const Location& loc_curr = pState->loc;
    const Location& loc_next = pSubTraj->pState->loc;
    float d_theta = loc_next.theta - loc_curr.theta;
    las = d_theta * d_theta + pSubTraj->las;
}

// Traj1 < Traj2 means that Traj1 is WORSE that Traj2
bool Trajectory::operator<(const Trajectory& t) {
    if (val == t.val)
        return las > t.las;
    else
        return val < t.val;
}

// Traj1 > Traj2 means that Traj1 is BETTER that Traj2
bool Trajectory::operator>(const Trajectory& t) {
    if (val == t.val)
        return las < t.las;
    else
        return val > t.val;
}

Trajectory::~Trajectory() = default;

// endregion
// =====================================================================================================================
// region PlannerV2 class
// =====================================================================================================================

PlannerV1::PlannerV1(const CameraParameters& cparams,
                     const LaserParameters& lparams,
                     const std::vector<float>& ranges,
                     const Interpolator& interpolator,
                     bool debug) : Planner(cparams, lparams, ranges, interpolator, debug) {
    constructGraph();
}

void PlannerV1::constructGraph() {
    // Add states in the graph.
    for (int ray_i = 0; ray_i < num_camera_rays_; ray_i++)
        for (int range_i = 0; range_i < num_ranges_per_ray_; range_i++) {
            const Location& loc = layout_[ray_i][range_i];
            graph_[ray_i][range_i].loc = loc;
        }

    // Add edges in the graph.
    for (int ray_i = 0; ray_i < num_camera_rays_ - 1; ray_i++) {
        State* ray_curr = graph_[ray_i];
        State* ray_next = graph_[ray_i + 1];

        for (int curr_i = 0; curr_i < num_ranges_per_ray_; curr_i++) {
            State &state_curr = ray_curr[curr_i];
            const float& theta_curr = state_curr.loc.theta;

            for (int next_i = 0; next_i < num_ranges_per_ray_; next_i++) {
                State &state_next = ray_next[next_i];
                const float& theta_next = state_next.loc.theta;

                bool is_neighbor = abs(theta_next - theta_curr) < max_delta_theta_;
                if (is_neighbor)
                    state_curr.edges.emplace_back(ray_i + 1, next_i);
            }
        }
    }
}

PlannerV1::~PlannerV1() = default;

std::vector<std::pair<float, float>> PlannerV1::optGlobalCostDiscrete(Eigen::MatrixXf cmap) {
    // Check if cmap shape is as expected.
    if (!interpolator_.isCmapShapeValid(cmap.rows(), cmap.cols()))
        throw std::invalid_argument(std::string("PLANNER: Unexpected cmap shape (")
                                    + std::to_string(cmap.size())
                                    + std::string(")."));

    // Backward pass.
    for (int ray_i = num_camera_rays_ - 1; ray_i >= 0; ray_i--) {
        for (int range_i = 0; range_i < num_ranges_per_ray_; range_i++) {
            State* pState = &(graph_[ray_i][range_i]);

            if (ray_i == num_camera_rays_ - 1) {
                // For last ray, the trajectory starts and ends at the same node.
                dp_[ray_i][range_i] = Trajectory(pState, cmap);
            } else {
                // For non-last ray, iterate over all its valid neighbors to select best sub-trajectory.
                for (int edge_i = 0; edge_i < pState->edges.size(); edge_i++) {
                    std::pair<int, int>& edge = pState->edges[edge_i];
                    Trajectory* pSubTraj = &(dp_[edge.first][edge.second]);
                    Trajectory traj(pState, pSubTraj, cmap);
                    if (edge_i == 0 || traj > dp_[ray_i][range_i])
                        dp_[ray_i][range_i] = traj;
                }
            }
        }
    }

    // Select overall best trajectory.
    Trajectory best_traj = dp_[0][0];
    for (int range_i = 1; range_i < num_ranges_per_ray_; range_i++)
        if (dp_[0][range_i] > best_traj)
            best_traj = dp_[0][range_i];

    if (debug_) {
        std::cout << std::fixed << std::setprecision(3)
                  << "PLANNER: Optimal cost         : " << best_traj.val << std::endl
                  << "         Optimal laser penalty: " << best_traj.las << std::endl
                ;
    }

    // Forward pass.
    std::vector<std::pair<float, float>> design_pts;
    while (true) {
        // Current design point.
        design_pts.emplace_back(best_traj.pState->loc.x, best_traj.pState->loc.z);

        if (!best_traj.pSubTraj)  // trajectory ends here
            break;

        best_traj = *(best_traj.pSubTraj);
    }

    return design_pts;
}

// TODO: this function is untested
std::vector<std::pair<float, float>> PlannerV1::optGreedyL1Continuous(std::vector<float> target_ranges) {
    if (target_ranges.size() != num_camera_rays_)
        throw std::logic_error("PlannerV1::optGreedyL1Continuous : number of target ranges input to this function were "
                               + std::to_string(target_ranges.size()) + ", but PlannerV1 has been initialized with "
                               + std::to_string(num_camera_rays_) + " camera rays. Both should be equal.");

    // "c" stands for curr timestep,
    // "n" stands for next timestep,
    // "t" stands for targets in the next timestep

    Location cLoc;

    std::vector<std::pair<float, float>> design_pts;
    for (int ray_i = 0; ray_i < num_camera_rays_; ray_i++) {
        const float& tRange = target_ranges[ray_i];
        const Location tLoc = createPartialLocFromRange(ray_i, tRange);

        // quantities for curr timestep
        const float& cTheta = cLoc.theta;

        // target quantities for next timestep
        const float& tTheta = tLoc.theta;

        // quantities for next timestep
        float nTheta;

        if (ray_i == 0) {
            // No constraints, design point coincides with target on the first camera ray
            nTheta = tTheta;
        } else {
            // Velocity constraints only.
            float nThetaMax = cTheta + max_delta_theta_;
            float nThetaMin = cTheta - max_delta_theta_;
            nTheta = clamp(tTheta, nThetaMin, nThetaMax);
        }

        Location nLoc = createPartialLocFromTheta(ray_i, nTheta);
        design_pts.emplace_back(nLoc.x, nLoc.z);

        // increment timestep: update cLoc <-- nLoc
        cLoc = nLoc;
    }

    return design_pts;
}

float _cdf(float x, const std::string& r_sampling) {
    // CDF function for inputs, that is expected to be the scaled range.
    x = std::max(std::min(x, 1.0f), 0.0f);  // CDF for all inputs < 0 is the same as 0 and >1 is the same as 1
    if (r_sampling == "uniform")
        return x;
    else if (r_sampling == "linear")
        return x * x;
    else
        throw std::invalid_argument("r_sampling has to be one of <equal> or <uniform> or <linear>");
}

std::vector<float> PlannerV1::_edge_probabilities(const std::vector<std::pair<int, int>>& edges,
                                                  const std::string& r_sampling) {
    // the ranges of edges are assumed to be in ascending order
    std::vector<float> probs (edges.size());

    // Equal probabilities for all edges. Not based on the location of states at all.
    if (r_sampling == "equal") {
        std::fill(probs.begin(), probs.end(), 1.0f / edges.size());
        return probs;
    }

    float MAX_RANGE = ranges_[ranges_.size() - 1];

    float last_x = -INF;  // previous scaled range that has already been accounted for

    for (int i = 0; i < edges.size(); i++) {
        float next_x;
        if (i < edges.size() - 1) {
            // compute midpoint between current state's location the next state's location

            // current state
            const std::pair<int, int>& curr_edge = edges[i];
            const State& curr_state = graph_[curr_edge.first][curr_edge.second];
            const float& curr_state_r = curr_state.loc.r;

            // next state
            const std::pair<int, int>& next_edge = edges[i+1];
            const State& next_state = graph_[next_edge.first][next_edge.second];
            const float& next_state_r = next_state.loc.r;
            assert (next_state_r >= curr_state_r);

            // this state accounts for the interval from prev_r to the midpoint
            float next_r = (curr_state_r + next_state_r) / 2.0f;
            next_x = next_r / MAX_RANGE;
        } else {
            next_x = INF;
        }

        // the probability for this node
        probs[i] = _cdf(next_x, r_sampling) - _cdf(last_x, r_sampling);
        last_x = next_x;
    }

    // assert that the sum of probs is 1
    float sum = std::accumulate(probs.begin(), probs.end(), 0.0f);
    assert (abs(sum - 1.0f) < 1.0e-6f);

    return probs;
}

std::vector<std::array<float, 3>> PlannerV1::randomCurtainDiscrete(const std::string& r_sampling) {
    // init edges: edge to every state on the first camera ray
    std::vector<std::pair<int, int>> init_edges (num_ranges_per_ray_);
    for (int range_i = 0; range_i < num_ranges_per_ray_; range_i++)
        init_edges[range_i] = std::pair<int, int>(0, range_i);

    std::vector<std::pair<int, int>>* edges_ptr = &init_edges;

    std::vector<std::array<float, 3>> curtain;

    while (!edges_ptr->empty()) {
        // sample an edge
        const std::vector<float>& probs = _edge_probabilities(*edges_ptr, r_sampling);
        std::discrete_distribution<int> distribution (probs.begin(), probs.end());
        int sampled_edge_index = distribution(gen_);
        const std::pair<int, int>& edge = (*edges_ptr)[sampled_edge_index];
        State& state = graph_[edge.first][edge.second];

        // get location and intensity
        const Location& loc = state.loc;
        float intensity = layout_intensities_[loc.ray_i][loc.range_i];
        curtain.push_back(std::array<float, 3> {loc.x, loc.z, intensity});

        // select next set of edges
        edges_ptr = &(state.edges);
    }

    return curtain;
}

float PlannerV1::randomCurtainHitProb(float threshold, std::string r_sampling) {
    float dp[2][MAX_RANGES_PER_RAY];  // arrays that execute dp
    // one of the two arrays will be used for dp in the current iteration, the other stores dp value from the previous
    // iteration
    int curr_dp_index = 0, prev_dp_index = 1;
    // initial dp values in the previous iteration: previous probabilities are all 0
    std::fill(dp[prev_dp_index], dp[prev_dp_index] + num_camera_rays_, 0.0f);

    for (int ray_i = num_camera_rays_ - 1; ray_i >= 0; ray_i--) {
        for (int range_i = 0; range_i < num_ranges_per_ray_; range_i++) {
            const State& state = graph_[ray_i][range_i];
            const Location& loc = state.loc;
            bool hit = layout_intensities_[loc.ray_i][loc.range_i] > threshold;

            float hit_prob = 0.0f;
            if (hit)
                hit_prob = 1.0f;  // if the current state/location receives a hit, hit probability is automatically 1
            else {
                const std::vector<std::pair<int, int>>& edges = state.edges;
                if (!edges.empty()) { // if no edges, hit_prob is automatically 0
                    const std::vector<float>& edge_transition_probs = _edge_probabilities(edges, r_sampling);
                    for (int edge_i = 0; edge_i < edges.size(); edge_i++) {
                        const std::pair<int, int>& edge = edges[edge_i];
                        assert (edge.first == ray_i + 1);
                        float edge_transition_prob = edge_transition_probs[edge_i];
                        float edge_hit_prob = dp[prev_dp_index][edge.second];
                        hit_prob += edge_transition_prob * edge_hit_prob;
                    }
                }
            }
            dp[curr_dp_index][range_i] = hit_prob;
        }

        // interchange curr_dp_index and prev_dp_index
        curr_dp_index = (curr_dp_index + 1) % 2;
        prev_dp_index = (prev_dp_index + 1) % 2;
    }

    float hit_prob = 0.0f;
    // dp has been run upto the first ray. use dummy init edges for sampling of nodes from first ray.
    std::vector<std::pair<int, int>> init_edges (num_ranges_per_ray_);
    for (int range_i = 0; range_i < num_ranges_per_ray_; range_i++)
        init_edges[range_i] = std::pair<int, int>(0, range_i);
    std::vector<float> init_edge_transition_probs = _edge_probabilities(init_edges, r_sampling);
    for (int range_i = 0; range_i < num_ranges_per_ray_; range_i++) {
        float init_edge_transition_prob = init_edge_transition_probs[range_i];
        float init_edge_hit_prob = dp[prev_dp_index][range_i];
        hit_prob += init_edge_transition_prob * init_edge_hit_prob;
    }

    return hit_prob;
}

// endregion
// =====================================================================================================================

} }
