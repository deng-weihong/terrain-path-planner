#include "internal.hpp"

#include <numeric>

namespace terrain_planner {
namespace {

// Lower envelopes of squared-distance parabolas give an exact separable EDT.
std::vector<double> edt_line(const std::vector<double>& f) {
    const int n = static_cast<int>(f.size());
    std::vector<int> sites(n);
    std::vector<double> boundaries(n + 1), out(n, 1e20);
    int k = -1;
    for (int q = 0; q < n; ++q) {
        if (f[q] >= 1e19)
            continue;
        double boundary = -std::numeric_limits<double>::infinity();
        while (k >= 0) {
            const int p = sites[k];
            boundary = ((f[q] + double(q) * q) - (f[p] + double(p) * p)) / (2.0 * (q - p));
            if (boundary > boundaries[k])
                break;
            --k;
        }
        sites[++k] = q;
        boundaries[k] = k == 0 ? -std::numeric_limits<double>::infinity() : boundary;
        boundaries[k + 1] = std::numeric_limits<double>::infinity();
    }
    if (k < 0)
        return out;
    int envelope = 0;
    for (int q = 0; q < n; ++q) {
        while (boundaries[envelope + 1] < q)
            ++envelope;
        const double delta = q - sites[envelope];
        out[q] = delta * delta + f[sites[envelope]];
    }
    return out;
}

std::vector<double> edt(const std::vector<bool>& mask, int width, int height, bool feature) {
    std::vector<double> result(mask.size());
    for (int y = 0; y < height; ++y) {
        std::vector<double> row(width);
        for (int x = 0; x < width; ++x)
            row[x] = mask[y * width + x] == feature ? 0 : 1e20;
        const auto transformed = edt_line(row);
        std::copy(transformed.begin(), transformed.end(), result.begin() + y * width);
    }
    for (int x = 0; x < width; ++x) {
        std::vector<double> column(height);
        for (int y = 0; y < height; ++y)
            column[y] = result[y * width + x];
        const auto transformed = edt_line(column);
        for (int y = 0; y < height; ++y)
            result[y * width + x] = std::sqrt(transformed[y]);
    }
    return result;
}

} // namespace

GridMap::GridMap(std::vector<Key> keys, double resolution)
    : keys_(std::move(keys)), resolution_(resolution) {
    if (keys_.empty() || keys_.size() > 2000000 || !std::isfinite(resolution) || resolution <= 0)
        throw std::invalid_argument(
            "Map requires 1 to 2000000 voxels and a finite positive resolution");
    int min_x = keys_[0][0], max_x = min_x, min_y = keys_[0][1], max_y = min_y;
    for (const auto& key : keys_) {
        for (auto value : key)
            if (value < std::numeric_limits<int>::min() + 16 ||
                value > std::numeric_limits<int>::max() - 16 ||
                !std::isfinite((value + 0.5) * resolution))
                throw std::invalid_argument("Voxel coordinate is outside the supported range");
        min_x = std::min(min_x, int(key[0]));
        max_x = std::max(max_x, int(key[0]));
        min_y = std::min(min_y, int(key[1]));
        max_y = std::max(max_y, int(key[1]));
        columns_[{key[0], key[1]}].push_back((key[2] + 0.5) * resolution);
    }
    const auto width = std::int64_t(max_x) - min_x + 7, height = std::int64_t(max_y) - min_y + 7;
    if (width > 20000000 || height > 20000000 || width * height > 20000000)
        throw std::invalid_argument("XY distance-field bounds exceed 20000000 cells");
    origin_x_ = min_x - 3;
    origin_y_ = min_y - 3;
    width_ = static_cast<int>(width);
    height_ = static_cast<int>(height);
    std::vector<bool> mask(static_cast<std::size_t>(width) * height, false);
    for (const auto& key : keys_)
        mask[(key[1] - origin_y_) * width_ + key[0] - origin_x_] = true;
    for (auto& item : columns_) {
        auto& heights = item.second;
        std::sort(heights.begin(), heights.end());
        heights.erase(std::unique(heights.begin(), heights.end()), heights.end());
    }
    const auto inside = edt(mask, width_, height_, false),
               outside = edt(mask, width_, height_, true);
    distance_.resize(mask.size());
    for (std::size_t i = 0; i < mask.size(); ++i)
        distance_[i] = (mask[i] ? inside[i] - 0.5 : 0.5 - outside[i]) * resolution;
}

const std::vector<double>& GridMap::heights(int x, int y) const {
    static const std::vector<double> empty;
    const auto it = columns_.find({x, y});
    return it == columns_.end() ? empty : it->second;
}

bool GridMap::occupied(const Point& point) const {
    return !heights(detail::cell(point[0], resolution_), detail::cell(point[1], resolution_))
                .empty();
}

std::pair<double, bool> GridMap::height(const Point& point, double expected_z) const {
    const auto& choices =
        heights(detail::cell(point[0], resolution_), detail::cell(point[1], resolution_));
    if (choices.empty())
        return {std::numeric_limits<double>::quiet_NaN(), false};
    const double nearest =
        *std::min_element(choices.begin(), choices.end(), [expected_z](double a, double b) {
            return std::abs(a - expected_z) < std::abs(b - expected_z);
        });
    return {nearest, std::abs(nearest - expected_z) <= 0.080001};
}

FieldSample GridMap::field(const Point& point) const {
    if (!detail::finite(point))
        throw std::invalid_argument("Field query must be finite");
    const double cx = detail::clamp(point[0] / resolution_ - origin_x_ - 0.5, 0, width_ - 1);
    const double cy = detail::clamp(point[1] / resolution_ - origin_y_ - 0.5, 0, height_ - 1);
    const int x = std::min(static_cast<int>(std::floor(cx)), width_ - 2);
    const int y = std::min(static_cast<int>(std::floor(cy)), height_ - 2);
    const double fx = cx - x, fy = cy - y;
    const double a = distance_[y * width_ + x], b = distance_[y * width_ + x + 1];
    const double c = distance_[(y + 1) * width_ + x], d = distance_[(y + 1) * width_ + x + 1];
    return {(1 - fy) * ((1 - fx) * a + fx * b) + fy * ((1 - fx) * c + fx * d),
            {((1 - fy) * (b - a) + fy * (d - c)) / resolution_,
             ((1 - fx) * (c - a) + fx * (d - b)) / resolution_}};
}

std::vector<double> arc_length(const Path& path) {
    std::vector<double> s(path.size(), 0);
    for (std::size_t i = 1; i < path.size(); ++i)
        s[i] = s[i - 1] + detail::xy_distance(path[i - 1], path[i]);
    return s;
}

Path detail::interpolate_path(const Path& path, const std::vector<double>& source_s,
                              const std::vector<double>& query) {
    Path unique;
    std::vector<double> s;
    for (std::size_t i = 0; i < path.size(); ++i) {
        if (i == 0 || source_s[i] - source_s[i - 1] > epsilon) {
            unique.push_back(path[i]);
            s.push_back(source_s[i]);
        }
    }
    if (unique.size() < 2 || s.back() < 1e-6)
        throw std::invalid_argument("The route must contain two distinct XY positions");
    Path out(query.size());
    for (int dim = 0; dim < 3; ++dim) {
        std::vector<double> values;
        for (const auto& p : unique)
            values.push_back(p[dim]);
        for (std::size_t i = 0; i < query.size(); ++i)
            out[i][dim] = interpolate(s, values, query[i]);
    }
    return out;
}

std::pair<Path, std::vector<double>> resample(const Path& path, double spacing) {
    if (path.size() < 2 || !std::isfinite(spacing) || spacing <= 0)
        throw std::invalid_argument("Resampling requires two points and a positive spacing");
    for (const auto& p : path)
        if (!detail::finite(p))
            throw std::invalid_argument("Route coordinates must be finite");
    const auto source_s = arc_length(path);
    if (source_s.back() > 1000)
        throw std::invalid_argument("Route length must not exceed 1000 m");
    auto s = detail::linspace(source_s.back(),
                              std::max<std::size_t>(2, std::ceil(source_s.back() / spacing) + 1));
    return {detail::interpolate_path(path, source_s, s), std::move(s)};
}

} // namespace terrain_planner
