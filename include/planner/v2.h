#ifndef V2_H
#define V2_H

#include "base.h"

namespace planner { namespace v2 {

struct State {
public:
    const Location& loc_curr;  // current location
    const Location& loc_prev;  // previous location
    int state_i;  // index of the state on the ray

    std::vector<const State*> edges;  // const pointer to state on next camera ray

    State(const Location& loc_prev_, const Location& loc_curr_, int state_i_);  // note the order!
};

class Trajectory {
public:
    float val;  // sum of values to be maximized

    State* pState;  // pointer to the state the trajectory starts from
    Trajectory* pSubTraj;  // pointer to the rest of the sub-trajectory;

    Trajectory();
    Trajectory(State* pState_, const Eigen::MatrixXf& cmap);
    Trajectory(State* pState_, Trajectory* pSubTraj_, const Eigen::MatrixXf& cmap);
    ~Trajectory();

    bool operator< (const Trajectory& t);
    bool operator> (const Trajectory& t);
};

class PlannerV2 : public Planner {
private:
    Location start_loc_;  // start location with laser angle == NAN

    // states in each ray sorted according to (state.loc_prev.range_i, state.loc_curr.range_i)
    std::vector<State> graph_[MAX_RAYS];

     std::vector<Trajectory> dp_[MAX_RAYS];

    void _add_ray_states(const Location& loc_prev,
                         const std::vector<State>& next_ray_states,
                         std::vector<State>& curr_ray_states);
    void _add_last_ray_states(const Location& loc_prev,
                              std::vector<State>& last_ray_states);
    void _num_ray_states_msg(int ray_i);
    void constructGraph();

public:
    PlannerV2(const CameraParameters& cparams,
              const LaserParameters& lparams,
              const std::vector<float>& ranges,
              const Interpolator& interpolator,
              bool debug);
    ~PlannerV2();


    // -----------------------------------------------------------------------------------------------------------------
    // Optimization methods.
    // -----------------------------------------------------------------------------------------------------------------

    std::vector<std::pair<float, float>> optGlobalCostDiscrete(Eigen::MatrixXf cmap);
    std::vector<std::pair<float, float>> optGreedyL1Continuous(std::vector<float> target_ranges);

    // -----------------------------------------------------------------------------------------------------------------
    // Random curtains: sampling and DP
    // -----------------------------------------------------------------------------------------------------------------

    std::vector<float> _edge_probabilities(const std::vector<const State*>& edges, const std::string& type);
    std::vector<std::array<float, 3>> randomCurtainDiscrete(const std::string& r_sampling);

    float randomCurtainHitProb(float threshold, std::string r_sampling);
};

} }

#endif
