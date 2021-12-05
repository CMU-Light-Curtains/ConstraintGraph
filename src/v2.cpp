#include "v2.h"

namespace planner { namespace v2 {

// =====================================================================================================================
// region State class
// =====================================================================================================================

State::State(const Location& loc_prev_, const Location& loc_curr_, int state_i_)  // note the order!
        : loc_prev(loc_prev_), loc_curr(loc_curr_), state_i(state_i_) {}

// endregion
// =====================================================================================================================
// region Trajectory class
// =====================================================================================================================

Trajectory::Trajectory() {
    pState = nullptr;
    pSubTraj = nullptr;
    val = -INF;
}

Trajectory::Trajectory(State* pState_, const Eigen::MatrixXf& cmap) : pState(pState_) {
    // Sub-trajectory.
    pSubTraj = nullptr;

    // Cost.
    const Location& loc = pState->loc_curr;
    if ((loc.ki != -1) && (loc.kj != -1))
        val = cmap(loc.ki, loc.kj);
    else
        val = -INF;
}

Trajectory::Trajectory(State* pState_, Trajectory* pSubTraj_, const Eigen::MatrixXf& cmap) : Trajectory(pState_, cmap) {
    // Start Node : delegated.

    // Sub-trajectory.
    pSubTraj = pSubTraj_;

    // Uncertainty.
    // Initialized from delegation.
    val += pSubTraj->val;
}

// Traj1 < Traj2 means that Traj1 is WORSE that Traj2
bool Trajectory::operator<(const Trajectory& t) {
    return val < t.val;
}

// Traj1 > Traj2 means that Traj1 is BETTER that Traj2
bool Trajectory::operator>(const Trajectory& t) {
    return val > t.val;
}

Trajectory::~Trajectory() = default;

// endregion
// =====================================================================================================================
// region PlannerV2 class
// =====================================================================================================================

PlannerV2::PlannerV2(const CameraParameters& cparams,
                     const LaserParameters& lparams,
                     const std::vector<float>& ranges,
                     const Interpolator& interpolator,
                     bool debug) : Planner(cparams, lparams, ranges, interpolator, debug) {

    start_loc_.fill(-1, -1, NAN, NAN, NAN, NAN, NAN, -1, -1);

    constructGraph();
}

void PlannerV2::_add_ray_states(const Location& loc_prev,
                                const std::vector<State>& next_ray_states,
                                std::vector<State>& curr_ray_states) {
    const float& theta_prev = loc_prev.theta;

    // ordered by (next_state.loc_prev.range_i, next_state.loc_curr.range_i)
    for (const State& next_state : next_ray_states) {
        const Location& loc_curr = next_state.loc_prev;  // the current state according to the next state
        const Location& loc_next = next_state.loc_curr;  // the next state according to the next state

        const float& theta_curr = loc_curr.theta;
        const float& theta_next = loc_next.theta;

        // =============================================================================================================
        // Constraint checking
        // =============================================================================================================

        // we already know that theta_curr -> theta_next satisfies the velocity constraint

        bool is_valid_edge = false;
        if (std::isnan(theta_prev)) {
            // theta_prev == NAN means that this could be any theta (i.e. for the first camera ray).
            // so always add edge
            is_valid_edge = true;
        }
        // velocity constraint
        else if (abs(theta_curr - theta_prev) < max_delta_theta_) {
            // acceleration constraint
            float omega2 = (theta_next - theta_curr) / cam_timestep_;
            float omega1 = (theta_curr - theta_prev) / cam_timestep_;
            is_valid_edge = abs(omega2 - omega1) < max_delta_omega_;
        }
        else {
            is_valid_edge = false;
        }
        // =============================================================================================================

        // add edge
        if (is_valid_edge) {
            // Check whether a state with (loc_prev, loc_curr) exists.
            // Since next_ray_states are sorted according to (next_state.loc_prev.range_i, next_state.loc_curr.range_i),
            // this would be the most recently added state.
            bool state_exists = (!curr_ray_states.empty()) &&
                                (&(curr_ray_states.back().loc_prev) == &loc_prev) &&
                                (&(curr_ray_states.back().loc_curr) == &loc_curr);

            // create state for (loc_prev, loc_curr) if one doesn't already exist
            if (!state_exists)
                curr_ray_states.emplace_back(loc_prev, loc_curr, curr_ray_states.size());

            curr_ray_states.back().edges.push_back(&next_state);
        }
    }
}

void PlannerV2::_add_last_ray_states(const Location& loc_prev,
                                     std::vector<State>& last_ray_states) {
    const float& theta_prev = loc_prev.theta;

    for (int range_i = 0; range_i < num_ranges_per_ray_; range_i++) {
        const Location& loc_curr = layout_[num_camera_rays_ - 1][range_i];
        const float& theta_curr = loc_curr.theta;

        // We only need to check velocity constraints between theta_prev and theta_curr.
        // No acceleration constraints for states on the last ray.
        if (abs(theta_curr - theta_prev) < max_delta_theta_) {
            // there are no edges for states on the last ray
            last_ray_states.emplace_back(loc_prev, loc_curr, last_ray_states.size());
        }
    }
}

void PlannerV2::_num_ray_states_msg(int ray_i) {
    const int& size = graph_[ray_i].size();

#ifndef NDEBUG
    const int& max_states = num_ranges_per_ray_ * num_ranges_per_ray_;
    const float& pcent = float(size) * 100.0 / max_states;
    DEBUG_MSG("States in ray " << ray_i << ": " << size << " / " << max_states << "( " <<
              std::setprecision(4) << pcent << "% )");
#endif

    if (size == 0)
        throw std::runtime_error("Number of states in the constraint graph at ray " + std::to_string(ray_i) +
                                 " become zero. Either (1) relax the galvo max_omega and/or max_alpha constraints, " +
                                 " or (2) increase the number of ranges per camera ray.");
}

void PlannerV2::constructGraph() {
    // NOTE: states in each ray sorted according to (state.loc_prev.range_i, state.loc_curr.range_i)

    if (num_camera_rays_ < 3)
        throw std::out_of_range("Number of camera rays must be at least three.");

    // =================================================================================================================
    // Last ray.
    // =================================================================================================================

    std::vector<State>& last_ray_states = graph_[num_camera_rays_ - 1];
    assert (last_ray_states.empty());
    // last_ray_states.reserve(num_ranges_per_ray_ * num_ranges_per_ray_);  // preallocate to make push_back fast
    for (int range_prev_i = 0; range_prev_i < num_ranges_per_ray_; range_prev_i++) {
        const Location& loc_prev = layout_[num_camera_rays_ - 2][range_prev_i];
        _add_last_ray_states(loc_prev, last_ray_states);
    }

    _num_ray_states_msg(num_camera_rays_ - 1);

    // =================================================================================================================
    // Middle rays.
    // =================================================================================================================

    for (int ray_i = num_camera_rays_ - 2; ray_i >= 1; ray_i--) {
        const std::vector<State>& next_ray_states = graph_[ray_i + 1];
        std::vector<State>& curr_ray_states = graph_[ray_i];
        assert (curr_ray_states.empty());
        // curr_ray_states.reserve(num_ranges_per_ray_ * next_ray_states.size());  // preallocate to make push_back fast
        for (int range_prev_i = 0; range_prev_i < num_ranges_per_ray_; range_prev_i++) {
            const Location& loc_prev = layout_[ray_i - 1][range_prev_i];
            _add_ray_states(loc_prev, next_ray_states, curr_ray_states);
        }

        _num_ray_states_msg(ray_i);
    }

    // =================================================================================================================
    // First ray.
    // =================================================================================================================

    const std::vector<State>& second_ray_states = graph_[1];
    std::vector<State>& first_ray_states = graph_[0];
    assert (first_ray_states.empty());
    first_ray_states.reserve(second_ray_states.size());  // preallocate to make push_back fast
    _add_ray_states(start_loc_, second_ray_states, first_ray_states);

    _num_ray_states_msg(0);
}

PlannerV2::~PlannerV2() = default;

std::vector<std::pair<float, float>> PlannerV2::optGlobalCostDiscrete(Eigen::MatrixXf cmap) {
    // Check if cmap shape is as expected.
    if (!interpolator_.isCmapShapeValid(cmap.rows(), cmap.cols()))
        throw std::invalid_argument(std::string("PLANNER: Unexpected cmap shape (")
                                    + std::to_string(cmap.size())
                                    + std::string(")."));

    // Backward pass.
    for (int ray_i = num_camera_rays_ - 1; ray_i >= 0; ray_i--) {
        std::vector<State>& ray_states = graph_[ray_i];
        std::vector<Trajectory>& ray_trajs = dp_[ray_i];

        if (!ray_trajs.empty())
            ray_trajs.clear();

        ray_trajs.reserve(ray_states.size());  // size of dp_ is the same as the size of graph_

        for (State& state : ray_states) {
            // Trajectory starting and ending at current node.
            Trajectory best_traj(&state, cmap);

            if (ray_i < num_camera_rays_ - 1) {
                // For non-last ray, iterate over all its valid neighbors to select best sub-trajectory.
                for (int edge_i = 0; edge_i < state.edges.size(); edge_i++) {
                    const State& next_state = *(state.edges[edge_i]);
                    Trajectory *pSubTraj = &(dp_[ray_i + 1][next_state.state_i]);
                    Trajectory traj(&state, pSubTraj, cmap);
                    if (edge_i == 0 || traj.val > best_traj.val)
                        best_traj = traj;
                }
            }

            ray_trajs.push_back(best_traj);
        }
    }

    // Select overall best trajectory.
    Trajectory best_traj = dp_[0][0];
    for (Trajectory& traj : dp_[0])
        if (traj.val > best_traj.val)
            best_traj = traj;

    if (debug_) {
        std::cout << std::fixed << std::setprecision(3)
                  << "PLANNER: Optimal cost         : " << best_traj.val << std::endl
                  ;
    }

    // Forward pass.
    std::vector<std::pair<float, float>> design_pts;
    while (true) {
        // Current design point.
        const Location& loc = best_traj.pState->loc_curr;
        design_pts.emplace_back(loc.x, loc.z);

        if (!best_traj.pSubTraj)  // trajectory ends here
            break;

        best_traj = *(best_traj.pSubTraj);
    }

    assert (design_pts.size() == num_camera_rays_);
    return design_pts;
}

std::vector<std::pair<float, float>> PlannerV2::optGreedyL1Continuous(std::vector<float> target_ranges) {
    if (target_ranges.size() != num_camera_rays_)
        throw std::logic_error("PlannerV2::optGreedyL1Continuous : number of target ranges input to this function were "
                               + std::to_string(target_ranges.size()) + ", but PlannerV2 has been initialized with "
                               + std::to_string(num_camera_rays_) + " camera rays. Both should be equal.");

    Location locs[2];
    int curr_index = 0, prev_index = 1;

    // "p" stands for prev timestep,
    // "c" stands for curr timestep,
    // "n" stands for next timestep,
    // "t" stands for targets in the next timestep

    std::vector<std::pair<float, float>> design_pts;
    for (int ray_i = 0; ray_i < num_camera_rays_; ray_i++) {
        const float& tRange = target_ranges[ray_i];
        const Location tLoc = createPartialLocFromRange(ray_i, tRange);

        // quantities for prev timestep
        const float& pTheta = locs[prev_index].theta;

        // quantities for curr timestep
        const float& cTheta = locs[curr_index].theta;
        const float& cOmega = (cTheta - pTheta) / cam_timestep_;

        // target quantities for next timestep
        const float& tTheta = tLoc.theta;
        const float& tOmega = (tTheta - cTheta) / cam_timestep_;

        // quantities for next timestep
        float nTheta, nOmega;

        if (ray_i == 0) {
            // No constraints, design point coincides with target on the first camera ray
            nTheta = tTheta;
        }
        else if (ray_i == 1) {
            // Velocity constraints only.
            nOmega = clamp(tOmega, -max_omega_, max_omega_);
            nTheta = cTheta + nOmega * cam_timestep_;
        } else {
            // Velocity and acceleration constraints.
            float nOmegaMax = std::min(cOmega + max_delta_omega_,  max_omega_);
            float nOmegaMin = std::max(cOmega - max_delta_omega_, -max_omega_);
            nOmega = clamp(tOmega, nOmegaMin, nOmegaMax);
            nTheta = cTheta + nOmega * cam_timestep_;
        }

        Location nLoc = createPartialLocFromTheta(ray_i, nTheta);
        design_pts.emplace_back(nLoc.x, nLoc.z);

        // increment timestep: update pLoc <-- cLoc, cLoc <-- nLoc
        curr_index = (curr_index + 1) % 2;
        prev_index = (prev_index + 1) % 2;
        locs[curr_index] = nLoc;
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

std::vector<float> PlannerV2::_edge_probabilities(const std::vector<const State*>& edges,
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
            const State& curr_state = *(edges[i]);
            const float& curr_state_r = curr_state.loc_curr.r;

            // next state
            const State& next_state = *(edges[i+1]);
            const float& next_state_r = next_state.loc_curr.r;
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

std::vector<std::array<float, 3>> PlannerV2::randomCurtainDiscrete(const std::string& r_sampling) {
    // init edges: all states on the first camera ray (they are in ascending order of state.loc_curr.range_i)
    std::vector<const State*> init_edges;
    init_edges.reserve(graph_[0].size());
    for (const State& state : graph_[0])
        init_edges.push_back(&state);

    std::vector<const State*>* edges_ptr = &init_edges;

    std::vector<std::array<float, 3>> curtain;

    while (!edges_ptr->empty()) {
        // sample an edge
        const std::vector<float>& probs = _edge_probabilities(*edges_ptr, r_sampling);
        std::discrete_distribution<int> distribution (probs.begin(), probs.end());
        int sampled_edge_index = distribution(gen_);
        const State* edge = (*edges_ptr)[sampled_edge_index];
        State& state = const_cast<State &>(*edge);

        // get location and intensity
        const Location& loc = state.loc_curr;
        float intensity = layout_intensities_[loc.ray_i][loc.range_i];
        curtain.push_back(std::array<float, 3> {loc.x, loc.z, intensity});

        // select next set of edges
        edges_ptr = &(state.edges);
    }

    assert (curtain.size() == num_camera_rays_);  // forward traversal must only end at the last ray
    return curtain;
}

float PlannerV2::randomCurtainHitProb(float threshold, std::string r_sampling) {
    std::vector<float> dp[2];  // vectors that execute dp
    // one of the two vectors will be used for dp in the current iteration, the other stores dp value from the previous
    // iteration
    int curr_dp_index = 0, prev_dp_index = 1;

    for (int ray_i = num_camera_rays_ - 1; ray_i >= 0; ray_i--) {
        const std::vector<State>& ray_states = graph_[ray_i];

        dp[curr_dp_index].clear();
        dp[curr_dp_index].reserve(ray_states.size());

        for (const State &state : ray_states) {
            const Location &loc = state.loc_curr;
            assert(loc.ray_i == ray_i);
            bool hit = layout_intensities_[loc.ray_i][loc.range_i] > threshold;
            float hit_prob = 0.0f;
            if (hit)
                hit_prob = 1.0f;  // if the current node receives a hit, hit probability is automatically 1
            else {
                const std::vector<const State *>& edges = state.edges;
                if (!edges.empty()) {  // if no edges, hit_prob is automatically 0
                    const std::vector<float>& edge_transition_probs = _edge_probabilities(edges, r_sampling);
                    for (int edge_i = 0; edge_i < edges.size(); edge_i++) {
                        const State* edge = edges[edge_i];
                        int next_state_i = edge->state_i;

                        float edge_transition_prob = edge_transition_probs[edge_i];
                        float edge_hit_prob = dp[prev_dp_index][next_state_i];
                        hit_prob += edge_transition_prob * edge_hit_prob;
                    }
                }
            }
            dp[curr_dp_index].push_back(hit_prob);
        }

        // interchange curr_dp_index and prev_dp_index
        curr_dp_index = (curr_dp_index + 1) % 2;
        prev_dp_index = (prev_dp_index + 1) % 2;
    }

    float hit_prob = 0.0f;

    // dp has been run upto the first ray. use dummy init edges for sampling of nodes from first ray.
    // all states on the first camera ray (they are in ascending order of state.loc_curr.range_i)
    std::vector<const State*> init_edges;
    const std::vector<State>& first_ray_states = graph_[0];
    init_edges.reserve(first_ray_states.size());
    for (const State& state : first_ray_states)
        init_edges.push_back(&state);

    const std::vector<float>& init_edge_transition_probs = _edge_probabilities(init_edges, r_sampling);
    const std::vector<float>& init_hit_probs = dp[prev_dp_index];
    assert (init_edge_transition_probs.size() == init_hit_probs.size());

    for (int state_i = 0; state_i < init_edge_transition_probs.size(); state_i++) {
        float init_edge_transition_prob = init_edge_transition_probs[state_i];
        float init_edge_hit_prob = init_hit_probs[state_i];
        hit_prob += init_edge_transition_prob * init_edge_hit_prob;
    }

    return hit_prob;
}

// endregion
// =====================================================================================================================

} }
