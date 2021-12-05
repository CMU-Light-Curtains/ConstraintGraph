#ifndef BASE_H
#define BASE_H

#ifndef NDEBUG
#define DEBUG_MSG(msg) do { std::cerr << msg << std::endl; } while( false )
#else
#define DEBUG_MSG(msg) do { } while ( false )
#endif

#include <array>
#include <string>
#include <cfloat>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <memory>
#include <random>
#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <vector>

#define MAX_RAYS 1500
#define MAX_RANGES_PER_RAY 1000

namespace planner {

#define INF std::numeric_limits<float>::infinity()
typedef Eigen::Vector2f Pointf;
typedef Eigen::Vector2d Pointd;
typedef Eigen::ParametrizedLine<float, 2> Rayf;
typedef Eigen::ParametrizedLine<double, 2> Rayd;
typedef Eigen::Hyperplane<float, 2> Line;
typedef Eigen::MatrixX4f LineSurface;  // collection of 2D line segments that forms the surface

inline float clamp(float x, float min, float max) {
    return std::max(min, std::min(x, max));
}

// =====================================================================================================================
// region Location struct
// =====================================================================================================================

    struct Location {
    public:
        // all angles are in degrees
        int ray_i, range_i;
        float x, z;
        float r;
        float theta_cam;  // camera angle (in degrees)
        float theta;  // laser angle (in degrees)
        long ki, kj;

        Location();

        void fill(int ray_i, int range_i,
                  float x_, float z_, float r_, float theta_cam_, float theta_,
                  long ki_, long kj_);
    };

// endregion

// =====================================================================================================================
// region Interpolator classes
// =====================================================================================================================

class Interpolator {
public:
    virtual void setLocationCmapIndex(Location& loc) const = 0;
    virtual bool isCmapShapeValid(int nrows, int ncols) const = 0;
    virtual ~Interpolator() = 0;
};

class CartesianNNInterpolator : public Interpolator {
private:
    int cmap_w_, cmap_h_;
    float x_min_, x_max_, z_min_, z_max_;
public:
    CartesianNNInterpolator(int cmap_w, int cmap_h, float x_min, float x_max, float z_min, float z_max);
    void setLocationCmapIndex(Location& loc) const override;
    bool isCmapShapeValid(int nrows, int ncols) const override;
};

class PolarIdentityInterpolator : public Interpolator {
private:
    int num_camera_rays_, num_ranges_;
public:
    PolarIdentityInterpolator(int num_camera_rays, int num_ranges);
    void setLocationCmapIndex(Location& loc) const override;
    bool isCmapShapeValid(int nrows, int ncols) const override;
};

// endregion
// =====================================================================================================================
// region Parameter classes
// =====================================================================================================================

class CameraParameters {
public:
    int width;
    int height;
    float fps;
    std::vector<float> cam_matrix; // unrolled intrinsics matrix of size 9
    std::vector<float> cam_to_laser; // unrolled transform matrix of size 16
};

class LaserParameters {
public:
    float fov;
    float thickness;
    float divergence;
    float max_omega;  // maximum angular velocity (in degrees/sec)
    float max_alpha;  // maximum angular acceleration (in degrees/sec^2)
};

// endregion
// =====================================================================================================================
// region Planner class
// =====================================================================================================================

class Planner {
protected:
    bool debug_;
    float max_omega_, max_delta_theta_;
    float max_alpha_, max_delta_omega_;
    float cam_timestep_;
    float las_thickness_;
    float las_divergence_;
    Eigen::Matrix4f cam_to_laser_;
    std::mt19937 gen_;  // random number generator

    Location layout_[MAX_RAYS][MAX_RANGES_PER_RAY];

    std::vector<float> ranges_;
    const Interpolator& interpolator_;
    int num_camera_rays_, num_ranges_per_ray_;

    // members related to random light curtains
    LineSurface line_surface_;
    float visible_ranges_[MAX_RAYS];
    float layout_intensities_[MAX_RAYS][MAX_RANGES_PER_RAY];

    void constructLayout();

public:
    Planner(const CameraParameters& cparams,
            const LaserParameters& lparams,
            const std::vector<float>& ranges,
            const Interpolator& interpolator,
            bool debug);
    virtual ~Planner();

    std::vector<float> camera_angles_;
    std::vector<Rayf> camera_rays_;
    Pointf cam_origin_;  // in camera frame
    Pointf las_origin_;  // in camera frame

    // Note that both these methods do not initialize all members of the Location object!
    Location createPartialLocFromRange(int ray_i, float r);
    Location createPartialLocFromTheta(int ray_i, float theta);

    std::vector<std::vector<Location>> getVectorizedLayout();
    std::vector<float> getThetas() { return camera_angles_; }

    // -----------------------------------------------------------------------------------------------------------------
    // Optimization methods
    // -----------------------------------------------------------------------------------------------------------------

    // optimization is
    // (1) global: hence requires dynamic programming,
    // (2) cost: uses a 2D cost map as input that it tries to maximize,
    // (3) discrete: design points are selected from a discrete set of locations in the constraint graph.
    virtual std::vector<std::pair<float, float>> optGlobalCostDiscrete(Eigen::MatrixXf cmap) = 0;

    // optimization is
    // (1) greedy: doesn't use DP. performs a single sequential iteration over rays from left to right, and picks a
    //             constraint-satisfying design point on the next ray that locally optimizes the objective for that ray.
    // (2) L1: inputs a 1D set of target ranges, and tries to minimize the L1 distance between design pts and the
    //         target ranges.
    // (3) continuous: selected design points can take continuous values.
    //
    //  Note: because of (1) and (3), the constraint graph is not used at all.
     virtual std::vector<std::pair<float, float>> optGreedyL1Continuous(std::vector<float> target_ranges) = 0;

    // -----------------------------------------------------------------------------------------------------------------
    // Line surfaces: computing depth and intensities
    // -----------------------------------------------------------------------------------------------------------------

    void clear();
    void loadLineSurface(LineSurface surface);
    void loadLineSurfaceJs(std::vector<std::vector<float>> surface);

    void computeVisibleRanges();

    std::vector<std::pair<float, float>> visiblePoints();
    std::vector<std::vector<float>> visiblePointsJs();

    void computeLayoutIntensities();

    std::vector<std::vector<float>> layoutIntensities();

    // -----------------------------------------------------------------------------------------------------------------
    // Random curtains: sampling and DP
    // -----------------------------------------------------------------------------------------------------------------

    virtual std::vector<std::array<float, 3>> randomCurtainDiscrete(const std::string& r_sampling) = 0;
    std::vector<std::vector<float>> randomCurtainDiscreteJs(const std::string& r_sampling);

    std::vector<std::pair<float, float>> randomCurtainContinuous(const std::string& r_sampling);

    virtual float randomCurtainHitProb(float threshold, std::string r_sampling) = 0;
};

}

#endif
