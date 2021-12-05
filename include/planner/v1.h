#ifndef V1_H
#define V1_H

#include "base.h"

namespace planner { namespace v1  {

struct State {
public:
    Location loc;
    std::vector<std::pair<int, int>> edges;  // each pair contains (ray_i, range_i) of the next state

    State();
    explicit State(const Location& loc_);
};

class Trajectory {
public:
    float val;  // sum of values to be maximized
    float las;  // sum of squares of laser angle deviation to be minimized

    State* pState;  // pointer to the state the trajectory starts from
    Trajectory* pSubTraj;  // pointer to the rest of the sub-trajectory;

    Trajectory();
    Trajectory(State* pState_, const Eigen::MatrixXf& cmap);
    Trajectory(State* pState_, Trajectory* pSubTraj_, const Eigen::MatrixXf& cmap);
    ~Trajectory();

    bool operator< (const Trajectory& t);
    bool operator> (const Trajectory& t);
};

class PlannerV1 : public Planner {
private:
    State graph_[MAX_RAYS][MAX_RANGES_PER_RAY];
    Trajectory dp_[MAX_RAYS][MAX_RANGES_PER_RAY];

    void constructGraph();

public:
    PlannerV1(const CameraParameters& cparams,
              const LaserParameters& lparams,
              const std::vector<float>& ranges,
              const Interpolator& interpolator,
              bool debug);
    ~PlannerV1();


    // -----------------------------------------------------------------------------------------------------------------
    // Optimization methods.
    // -----------------------------------------------------------------------------------------------------------------

    std::vector<std::pair<float, float>> optGlobalCostDiscrete(Eigen::MatrixXf cmap);
    std::vector<std::pair<float, float>> optGreedyL1Continuous(std::vector<float> target_ranges);

    // -----------------------------------------------------------------------------------------------------------------
    // Random curtains: sampling and DP
    // -----------------------------------------------------------------------------------------------------------------

    std::vector<float> _edge_probabilities(const std::vector<std::pair<int, int>>& edges, const std::string& type);
    std::vector<std::array<float, 3>> randomCurtainDiscrete(const std::string& r_sampling);

    float randomCurtainHitProb(float threshold, std::string r_sampling);
};

} }

#endif
