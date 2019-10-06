#ifndef PRIMITIVES_H_
#define PRIMITIVES_H_

#include <cmath>
#include <list>
#include <algorithm>

namespace racer::math
{

struct vector
{
private:
    double x_, y_;

public:
    vector() : x_{0}, y_{0} {}
    vector(double x, double y) : x_{x}, y_{y} {}
    
    vector(vector&& vec) = default;
    vector& operator =(vector&& vec) = default;
    
    vector(const vector& vec) = default;
    vector& operator =(const vector& vec) = default;
    
    constexpr const double& x() const noexcept { return x_; }
    constexpr const double& y() const noexcept { return y_; }

    double dot(const vector &other) const noexcept {
        return x_ * other.x_ + y_ * other.y_;
    }

    vector normal() const
    {
        double size = std::sqrt(x_ * x_ + y_ * y_);
        return vector(
            -y_ / size,
            x_ / size);
    }

    bool operator==(const vector &other) const
    {
        return x_ == other.x_ && y_ == other.y_;
    }

    bool operator!=(const vector &other) const
    {
        return x_ != other.x_ && y_ != other.y_;
    }

    vector rotate(double radians) const
    {
        return vector(
            x_ * std::cos(radians) - y_ * std::sin(radians),
            y_ * std::cos(radians) + x_ * std::sin(radians));
    }

    vector operator-(const vector &other) const
    {
        return vector(x_ - other.x_, y_ - other.y_);
    }

    vector operator+(const vector &other) const
    {
        return vector(x_ + other.x_, y_ + other.y_);
    }

    vector operator*(const double scale) const
    {
        return vector(scale * x_, scale * y_);
    }

    vector interpolate_with(const vector &other, const double weight, const double weight_other) const
    {
        const double wa = weight / (weight + weight_other);
        const double wb = weight_other / (weight + weight_other);
        return vector(wa * x_ + wb * other.x_, wa * y_ + wb * other.y_);
    }

    inline double length_sq() const
    {
        return dot(*this);
    }

    inline double length() const
    {
        return std::sqrt(length_sq());
    }

    inline double distance_sq(const vector &other) const
    {
        return (*this - other).length_sq();
    }

    inline double distance(const vector &other) const
    {
        return std::sqrt(distance_sq(other));
    }

    inline double angle() const
    {
        return std::atan2(y_, x_);
    }
};

typedef vector point; // alias

struct rectangle
{
    const point A, B, C, D;
    rectangle(point a, point b, point c, point d)
        : A(a), B(b), C(c), D(d)
    {
    }

    rectangle rotate(double radians) const
    {
        return rectangle(
            A.rotate(radians),
            B.rotate(radians),
            C.rotate(radians),
            D.rotate(radians));
    }

    bool intersects(const rectangle &other) const
    {
        for (const auto &edge : edges())
        {
            if (intersets_along(edge.normal(), other))
            {
                return true;
            }
        }

        for (const auto &edge : other.edges())
        {
            if (intersets_along(edge.normal(), other))
            {
                return true;
            }
        }

        return false;
    }

private:
    std::list<vector> edges() const
    {
        return {B - A, C - B, D - C, A - D};
    }

    std::pair<double, double> project_onto(const vector &axis) const
    {
        double a = axis.dot(A);
        double b = axis.dot(B);
        double c = axis.dot(C);
        double d = axis.dot(D);

        return std::pair<double, double>(
            std::min({a, b, c, d}),
            std::max({a, b, c, d}));
    }

    bool intersets_along(const vector &axis, const rectangle &other) const
    {
        auto a = project_onto(axis);
        auto b = other.project_onto(axis);

        auto interval_distance = a.first < b.first
                                     ? b.first - a.second
                                     : a.first - b.second;

        return interval_distance <= 0;
    }
};

struct angle
{
    const double radians;

    angle(double radians)
        : radians(to_small_angle(radians))
    {
    }

    operator double() const
    {
        return radians;
    }

    bool operator==(const angle &other) const
    {
        return radians == other.radians;
    }

    bool operator<(const angle &other) const
    {
        return radians < other.radians;
    }

private:
    static double to_small_angle(double angle)
    {
        while (angle < 0)
            angle += 2 * M_PI;
        while (angle >= 2 * M_PI)
            angle -= 2 * M_PI;
        return angle;
    }
};

template <typename T>
void hash_combine(size_t &seed, T const &v)
{
    seed ^= std::hash<T>()(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

struct circle
{
    racer::math::point center;
    double radius;

    circle(const point &center, double radius)
        : center(center), radius(radius)
    {
    }

    circle(const circle &other)
        : center(other.center), radius(other.radius)
    {
    }

    const std::list<racer::math::point> points_on_circumference(double starting_angle, double arc_angle, size_t number_of_points) const
    {
        std::list<racer::math::point> points;
        racer::math::angle step = arc_angle / (double)number_of_points;

        for (double angle = starting_angle; angle < starting_angle + arc_angle; angle += step)
        {
            points.emplace_back(
                center.x() + radius * std::cos(angle),
                center.y() + radius * std::sin(angle));
        }

        return points;
    }

    bool overlaps_with(const circle &other) const
    {
        return (other.center - center).length_sq() < std::pow(radius + other.radius, 2);
    }

    bool contains(const point &pt) const
    {
        return (center - pt).length_sq() < std::pow(radius, 2);
    }

    bool operator==(const circle &other) const
    {
        // for the purposes of this algorithm, we assume that two circles
        // are equal, if they mostly overlap
        double dist = (other.center - center).length_sq();
        return dist <= std::pow(radius, 2) || dist <= std::pow(other.radius, 2);
    }
};
} // namespace racer::math

#endif