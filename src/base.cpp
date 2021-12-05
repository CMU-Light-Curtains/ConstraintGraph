#include "base.h"

using namespace planner;

// =====================================================================================================================
// region UTILS
// =====================================================================================================================

float deg2rad(float deg) {
    return deg * M_PI / 180.0f;
}
float rad2deg(float rad) {
    return rad * 180.0f / M_PI;
}

// endregion
// =====================================================================================================================
// region Location struct
// =====================================================================================================================

Location::Location() = default;

void Location::fill(int ray_i_, int range_i_, float x_, float z_, float r_, float theta_cam_, float theta_,\
                    long ki_, long kj_) {
    ray_i = ray_i_;
    range_i = range_i_;
    x = x_;
    z = z_;
    r = r_;
    theta_cam = theta_cam_;
    theta = theta_;
    ki = ki_;
    kj = kj_;
}

// Note: this method does not initialize range_i, ki, kj!
Location Planner::createPartialLocFromRange(int ray_i, float r) {
    const float& theta_cam = camera_angles_[ray_i];

    float x = r * std::sin(deg2rad(theta_cam));
    float z = r * std::cos(deg2rad(theta_cam));

    // Compute laser angle.
    Eigen::Vector4f xyz1_cam(x, 0.0f, z, 1.0f);
    Eigen::Vector4f xyz1_las = cam_to_laser_ * xyz1_cam;
    float x_las = xyz1_las(0), z_las = xyz1_las(2);
    float theta = rad2deg(atan2f(x_las, z_las));

    Location loc;
    loc.fill(ray_i, -1, x, z, r, theta_cam, theta, -1, -1);
    return loc;
}

// Note: this method does not initialize range_i, ki, kj!
Location Planner::createPartialLocFromTheta(int ray_i, float theta) {
    // camera ray
    const float& theta_cam = camera_angles_[ray_i];
    const Rayf& cam_ray = camera_rays_[ray_i];

    // laser ray
    float lx = std::sin(deg2rad(theta));
    float lz = std::cos(deg2rad(theta));
    Eigen::Vector2f direction(lx, lz);
    Rayf las_ray(las_origin_, direction);

    // intersection
    Pointf point = las_ray.intersectionPoint(Line(cam_ray));

#ifndef NDEBUG
    float range = las_ray.intersectionParameter(Line(cam_ray));
    assert (range >= 0.f);
#endif

    float x = point(0);
    float z = point(1);
    float r = point.norm();

    Location loc;
    loc.fill(ray_i, -1, x, z, r, theta_cam, theta, -1, -1);
    return loc;
}

// endregion
// =====================================================================================================================
// region Interpolator classes
// =====================================================================================================================

Interpolator::~Interpolator(){}

CartesianNNInterpolator::CartesianNNInterpolator(int cmap_w, int cmap_h,
                                                 float x_min, float x_max, float z_min, float z_max) {
    cmap_w_ = cmap_w;
    cmap_h_ = cmap_h;
    x_min_ = x_min;
    x_max_ = x_max;
    z_min_ = z_min;
    z_max_ = z_max;
}

void CartesianNNInterpolator::setLocationCmapIndex(Location& loc) const {
    // This function assumes that the cost map is a 2D grid with evenly spaced xs and zs.
    // It also assumes that X and Z are increasing with a fixed increment.

    float x_incr = (x_max_ - x_min_) / float(cmap_h_ - 1);
    float z_incr = (z_max_ - z_min_) / float(cmap_w_ - 1);

    // Convert to pixel coordinates.
    long ki = std::lround((loc.x - x_min_) / x_incr);
    long kj = std::lround((loc.z - z_min_) / z_incr);

    if (ki < 0 || ki >= cmap_h_)
        ki = -1;  // means that this is outside the cmap grid

    if (kj < 0 || kj >= cmap_w_)
        kj = -1;  // means that this is outside the cmap grid

    // Set cmap index
    loc.ki = ki;
    loc.kj = kj;
}

bool CartesianNNInterpolator::isCmapShapeValid(int nrows, int ncols) const {
    return (nrows == cmap_h_) && (ncols == cmap_w_);
}

PolarIdentityInterpolator::PolarIdentityInterpolator(int num_camera_rays, int num_ranges)
        : num_camera_rays_(num_camera_rays), num_ranges_(num_ranges) {}

void PolarIdentityInterpolator::setLocationCmapIndex(Location& loc) const {
    // Set cmap index
    loc.ki = loc.range_i;
    loc.kj = loc.ray_i;
}

bool PolarIdentityInterpolator::isCmapShapeValid(int nrows, int ncols) const {
    return (nrows == num_ranges_) && (ncols == num_camera_rays_);
}

// endregion
// =====================================================================================================================
// region Planner class
// =====================================================================================================================
// ---------------------------------------------------------------------------------------------------------------------
// region Constructors for planner and layout
// ---------------------------------------------------------------------------------------------------------------------

Planner::Planner(const CameraParameters& cparams,
                 const LaserParameters& lparams,
                 const std::vector<float>& ranges,
                 const Interpolator& interpolator,
                 bool debug)
                 : ranges_(ranges), interpolator_(interpolator), debug_(debug){

    std::sort(ranges_.begin(), ranges_.end());  // store ranges in ascending order

    assert(cparams.cam_to_laser.size() == 16);
    cam_to_laser_ = Eigen::Matrix4f(cparams.cam_to_laser.data());  // column-major order
    Eigen::Matrix4f laser_to_cam_ = cam_to_laser_.inverse();

    assert(cparams.cam_matrix.size() == 9);
    Eigen::Matrix3f cam_matrix(cparams.cam_matrix.data());  // column-major order
    Eigen::Matrix3f cam_matrix_inv = cam_matrix.inverse();

    // camera origin
    cam_origin_ = Pointf {0, 0};  // camera origin is obviously (0, 0) in camera coordinates

    // laser origin
    Eigen::Vector4f las_origin_xyz1 = laser_to_cam_ * Eigen::Vector4f(0., 0., 0., 1.);
    float lo_x = las_origin_xyz1(0), lo_y = las_origin_xyz1(1), lo_z = las_origin_xyz1(2), one = las_origin_xyz1(3);
    assert(std::abs(lo_y - 0.0) < 1.0e-6f && std::abs(one - 1.0) < 1.0e-6f);  // [x, 0, z, 1]
    las_origin_ = Pointf {lo_x, lo_z};

    // compute camera angles
    for (int i = 0; i < cparams.width; i++) {
        // pixel coordinates
        float px = 1.0f + float(i);  // TODO: verify if this should be 0.5f or 1.0f
        float py = float(cparams.height) / 2.0f;
        Eigen::Vector3f p {px, py, 1.0f};

        // camera coordinates
        Eigen::Vector3f c = cam_matrix_inv * p;  // in camera coordinates, y should be 0
        float cx = c(0), cz = c(2);
        float theta = 90.0f - rad2deg(atan2f(cz, cx));
        camera_angles_.emplace_back(theta);

        // camera ray
        Eigen::Vector2f direction(cx, cz);
        direction.normalize();
        Rayf ray(cam_origin_, direction);
        camera_rays_.emplace_back(ray);
    }

    num_camera_rays_ = cparams.width;
    num_ranges_per_ray_ = ranges_.size();
    cam_timestep_ = 1.0f / (cparams.fps * float(cparams.width - 1));  // time b/w imaging two consecutive cam columns

    // max omega
    max_omega_ = lparams.max_omega;
    max_delta_theta_ = max_omega_ * cam_timestep_;

    // max alpha
    max_alpha_ = lparams.max_alpha;
    max_delta_omega_ = max_alpha_ * cam_timestep_;

    las_thickness_ = lparams.thickness;
    las_divergence_ = lparams.divergence;

    if (debug_) {
        std::cout << std::setprecision(4)
                  << "PLANNER: Max change in laser angle: " << max_omega_ * cam_timestep_ << "°" << std::endl;
    }

    constructLayout();

    clear(); // clear random curtain related quantities
}

void Planner::constructLayout() {
    // Add locations to the layout.
    for (int ray_i = 0; ray_i < num_camera_rays_; ray_i++)
        for (int range_i = 0; range_i < num_ranges_per_ray_; range_i++) {
            float r = ranges_[range_i];

            Location loc = createPartialLocFromRange(ray_i, r);

            // set remaining quantities in loc
            loc.range_i = range_i;
            interpolator_.setLocationCmapIndex(loc);

            layout_[ray_i][range_i] = loc;
        }
}

std::vector<std::vector<Location>> Planner::getVectorizedLayout() {
    std::vector<std::vector<Location>> m;

    // Copy 2D array to matrix.
    for (int ray_i = 0; ray_i < num_camera_rays_; ray_i++) {
        m.emplace_back();
        for (int range_i = 0; range_i < num_ranges_per_ray_; range_i++) {
            Location& loc = layout_[ray_i][range_i];
            m[ray_i].push_back(loc);
        }
    }
    return m;
}

// endregion
// ---------------------------------------------------------------------------------------------------------------------
// region Line surfaces: computing depth and intensities
// ---------------------------------------------------------------------------------------------------------------------

void Planner::clear() {
    line_surface_.resize(0, 4);
    std::fill(visible_ranges_, visible_ranges_ + MAX_RAYS, INF);
    std::fill(layout_intensities_[0], layout_intensities_[0] + MAX_RAYS * MAX_RANGES_PER_RAY, 0.0f);
}

void Planner::loadLineSurface(LineSurface surface) {
    line_surface_ = surface;
}

void Planner::loadLineSurfaceJs(std::vector<std::vector<float>> surface) {
    // copying data to line_surface_
    line_surface_.resize(surface.size(), 4);
    for (int i = 0; i < surface.size(); i++) {
        for (int j = 0; j < 4; j++)
            line_surface_(i, j) = surface[i][j];
    }
}

bool ray_segment_intersection(const Rayf& ray, const Pointf& p1, const Pointf& p2, Pointf& pi, float& range) {
    Line p1p2 = Line::Through(p1, p2);
    pi = ray.intersectionPoint(p1p2);  // intersection point
    range = ray.intersectionParameter(p1p2);
    if (range <= 0.f)  // intersection point lies in the opposite direction of the ray
        return false;

    // determine if the intersection point lies on the segment
    Pointf p1p2_vec = p2 - p1;
    Pointf p1pi_vec = pi - p1;

    float k_p1p2 = p1p2_vec.dot(p1p2_vec);  // max distance if point lies between p1 and p2
    float k_p1pi = p1p2_vec.dot(p1pi_vec);  // actual distance of point

    if (k_p1pi < 0 || k_p1pi > k_p1p2)
        return false;
    else if (abs(k_p1pi) < FLT_EPSILON || abs(k_p1pi - k_p1p2) < FLT_EPSILON)
        return true;
    else if (k_p1pi > 0.f && k_p1pi < k_p1p2)
        return true;
    else
        return false;
}

void Planner::computeVisibleRanges() {
    for (int ray_index = 0; ray_index < camera_angles_.size(); ray_index++) {
        Rayf& cam_ray = camera_rays_[ray_index];

        float range = INF;  // depth is infinity if no object can be seen on the ray
        Pointf point;  // point corresponding to the range
        int seg_index;  // index of the segment corresponding to the range
        Line seg_line;  // line of the segment corresponding to the range
        for (int seg_i = 0; seg_i < line_surface_.rows(); seg_i++) {

            // intersection point between cam ray and surface segment
            Eigen::Vector4f segment = line_surface_.row(seg_i);
            Pointf p1(segment(0), segment(1));
            Pointf p2(segment(2), segment(3));

            Pointf point_i;  // intersection point between cam ray and current segment
            float range_i;  // intersection range between cam ray and current segment
            bool intersects = ray_segment_intersection(cam_ray, p1, p2, point_i, range_i);

            if (intersects && range_i < range) {
                range = range_i;
                point = point_i;
                seg_index = seg_i;
                seg_line = Line::Through(p1, p2);
            }
        }

        // verify that the range visible from the camera is also visible from the laser

        // first, check whether the laser is on the same side of the segment as the camera
        if (range != INF) {
            float sd_cam = seg_line.signedDistance(cam_origin_);
            float sd_las = seg_line.signedDistance(las_origin_);
            if (sd_cam * sd_las < 0)
                range = INF;  // camera and laser lie on opposite sides of the segment line
        }

        // second, check whether any other segment obstructs the laser's line of sight
        if (range != INF) {
            Rayf las_ray = Rayf::Through(las_origin_, point);
            float las_to_point_dist = (point - las_origin_).norm();

            for (int seg_i = 0; seg_i < line_surface_.rows(); seg_i++) {
                if (seg_i == seg_index) continue;

                Eigen::Vector4f segment = line_surface_.row(seg_i);
                Pointf p1(segment(0), segment(1));
                Pointf p2(segment(2), segment(3));

                Pointf point_i;  // intersection point between las ray and current segment
                float range_i;  // intersection range between las ray and current segment
                bool intersects_surface_seg = ray_segment_intersection(las_ray, p1, p2, point_i, range_i);

                if (intersects_surface_seg) {
                    // check if the intersection point lies on the path from the laser to the depth point
                    if (range_i < las_to_point_dist) {
                        range = INF;
                        break;
                    }
                }
            }
        }

        visible_ranges_[ray_index] = range;
    }
}

std::vector<std::pair<float, float>> Planner::visiblePoints() {
    std::vector<std::pair<float, float>> points;

    for (int cam_index = 0; cam_index < camera_angles_.size(); cam_index++) {
        float cam_angle = camera_angles_[cam_index];
        float range = visible_ranges_[cam_index];

        if (range != INF) {
            float x = range * std::sin(deg2rad(cam_angle));
            float z = range * std::cos(deg2rad(cam_angle));
            points.emplace_back(x, z);
        }
    }
    return points;
}

std::vector<std::vector<float>> Planner::visiblePointsJs() {
    std::vector<std::pair<float, float>> points_ = visiblePoints();
    std::vector<std::vector<float>> points;
    for (const std::pair<float, float>& point_ : points_) {
        std::vector<float> point {point_.first, point_.second};
        points.push_back(point);
    }
    return points;
}

void Planner::computeLayoutIntensities() {
    // NOTE: all points and rays in this function are in the camera coordinate frame.
    // NOTE: all operations in this function will be performed using double since double precision seems necessary.

    // light source of the laser is behind the origin by this distance
    double las_source_offset = (las_thickness_ / 2.0f) / tanf(deg2rad(las_divergence_ / 2.0f));
    Pointd las_origin = las_origin_.cast<double>();

    for (int ray_i = 0; ray_i < num_camera_rays_; ray_i++) {
        float visible_range = visible_ranges_[ray_i];

        // case where surface doesn't exist on ray (is at infinite distance)
        if (visible_range == INF) {
            std::fill(layout_intensities_[ray_i], layout_intensities_[ray_i] + num_ranges_per_ray_, 0.0f);
            continue;
        }

        // visible range to point
        float cam_angle = camera_angles_[ray_i];
        float x = visible_range * std::sin(deg2rad(cam_angle));
        float z = visible_range * std::cos(deg2rad(cam_angle));
        const Pointd& visible_pt {x, z};

        for (int range_i = 0; range_i < num_ranges_per_ray_; range_i++) {
            const Location& loc = layout_[ray_i][range_i];
            const Pointd& node_pt {loc.x, loc.z};

            // Ray : laser origin to node
            Rayd las_origin_to_node = Rayd::Through(las_origin, node_pt);

            // Compute location of laser light source
            Pointd las_source_pt = las_origin_to_node.pointAt(-las_source_offset);

            // unit direction vectors of rays
            Eigen::Vector2d dir_las_origin_to_node  = (node_pt - las_origin).normalized();
            Eigen::Vector2d dir_las_source_to_point = (visible_pt - las_source_pt).normalized();

            // angle between rays
            double dot_product = dir_las_origin_to_node.dot(dir_las_source_to_point);
            dot_product = std::max(dot_product, -1.);
            dot_product = std::min(dot_product,  1.);
            float delta_angle = rad2deg(float(acos(dot_product)));  // [0, 180]
            assert(0. <= delta_angle && delta_angle <= 180.);

            // angle to intensity
            float intensity = 1.0f - std::min(delta_angle / (las_divergence_ / 2.0f), 1.0f);
            layout_intensities_[ray_i][range_i] = intensity;
        }
    }
}

std::vector<std::vector<float>> Planner::layoutIntensities() {
    std::vector<std::vector<float>> m;
    for (int ray_i = 0; ray_i < num_camera_rays_; ray_i++) {
        m.emplace_back(num_ranges_per_ray_);
        for (int range_i = 0; range_i < num_ranges_per_ray_; range_i++)
            m[ray_i][range_i] = layout_intensities_[ray_i][range_i];
    }
    return m;
}

// endregion
// ---------------------------------------------------------------------------------------------------------------------
// region Random curtain sampling
// ---------------------------------------------------------------------------------------------------------------------

std::vector<std::vector<float>> Planner::randomCurtainDiscreteJs(const std::string& r_sampling) {
    const std::vector<std::array<float, 3>> &curtain = randomCurtainDiscrete(r_sampling);
    std::vector<std::vector<float>> curtain_2;
    for (const std::array<float, 3> &pt : curtain) {
        std::vector<float> tuple {pt[0], pt[1], pt[2]};
        curtain_2.push_back(tuple);
    }
    return curtain_2;
}

std::vector<std::pair<float, float>> Planner::randomCurtainContinuous(const std::string &r_sampling) {
    // the target range for every camera ray is a random sample in the range [0, MAX_RANGE].
    const float& MAX_RANGE = ranges_[ranges_.size() - 1];
    std::uniform_real_distribution<float> distribution(0.0f, 1.0f);

    std::vector<float> target_ranges(num_camera_rays_);
    for (int ray_i = 0; ray_i < num_camera_rays_; ray_i++) {
        float u = distribution(gen_);  // uniform random sample in [0, 1]

        float tRange;
        if (r_sampling == "uniform") {
            tRange = u * MAX_RANGE;
        }
        else if (r_sampling == "linear") {
            tRange = std::sqrt(u) * MAX_RANGE;
        }
        else {
            throw std::invalid_argument("r_sampling has to be one of <uniform> or <linear>");
        }

        target_ranges[ray_i] = tRange;
    }

    return optGreedyL1Continuous(target_ranges);
}

Planner::~Planner() = default;

// endregion
// ---------------------------------------------------------------------------------------------------------------------
// endregion
// ---------------------------------------------------------------------------------------------------------------------
