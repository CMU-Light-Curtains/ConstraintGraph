#include <emscripten/bind.h>
#include <vector>
#include "v2.h"

using namespace emscripten;
using namespace planner;

float lerp(float a, float b, float t) {
    return (1 - t) * a + t * b;
}

std::string hello() {
    return std::string("Hello world");
}

class Dummy {
public:
    const std::vector<float>& vec_;

    Dummy(const std::vector<float>& vec);
    float dot(const std::vector<float>& vec2);
};

Dummy::Dummy(const std::vector<float>& vec): vec_(vec) {};

float Dummy::dot(const std::vector<float>& vec2) {
    float ret = 0.;
    for (int i = 0; i < vec_.size(); i++) {
        ret += vec_[i] * vec2[i];
    }
    return ret;
}

EMSCRIPTEN_BINDINGS(planner_js) {
    function("lerp", &lerp);
    function("hello", &hello);

    register_vector<float>("VectorFloat");
    register_vector<std::vector<float>>("VectorVectorFloat");

    class_<Dummy>("Dummy")
            .constructor<const std::vector<float> &>()
            .function("dot", &Dummy::dot);

    // Javascript
    // var lcsim = require('./lcsim')
    // var x = new lcsim.VectorFloat()
    // x.push_back(1.0)
    // x.push_back(2.0)
    // x.push_back(3.0)
    // var dummy = new lcsim.Dummy(x)

    // Planner
    class_<CameraParameters>("CameraParameters")
            .constructor()
            .property("width", &CameraParameters::width)
            .property("height", &CameraParameters::height)
            .property("fps", &CameraParameters::fps)
            .property("cam_matrix", &CameraParameters::cam_matrix)
            .property("cam_to_laser", &CameraParameters::cam_to_laser)
            ;

    class_<LaserParameters>("LaserParameters")
            .constructor()
            .property("fov", &LaserParameters::fov)
            .property("thickness", &LaserParameters::thickness)
            .property("divergence", &LaserParameters::divergence)
            .property("max_omega", &LaserParameters::max_omega)
            .property("max_alpha", &LaserParameters::max_alpha)
            ;

    class_<Interpolator>("Interpolator");

    class_<PolarIdentityInterpolator, base<Interpolator>>("PolarIdentityInterpolator")
            .constructor<int, int>()
            .function("isCmapShapeValid", &PolarIdentityInterpolator::isCmapShapeValid)
            ;

    class_<Planner>("Planner")
            .function("clear", &Planner::clear)
            .function("randomCurtainDiscreteJs", &Planner::randomCurtainDiscreteJs)
            .function("loadLineSurfaceJs", &Planner::loadLineSurfaceJs)
            .function("computeVisibleRanges", &Planner::computeVisibleRanges)
            .function("visiblePointsJs", &Planner::visiblePointsJs)
            .function("computeLayoutIntensities", &Planner::computeLayoutIntensities)
            ;

    class_<v2::PlannerV2, base<Planner>>("PlannerV2")
            .constructor<const CameraParameters &,
                         const LaserParameters &,
                         const std::vector<float> &,
                         const Interpolator&,
                         bool>()
            .function("randomCurtainHitProb", &v2::PlannerV2::randomCurtainHitProb)
            ;
}

