//
// Created by janne on 2024-07-17.
//

#include "triangulate.h"

Eigen::Vector2d calculateRelativePoint(const Eigen::Vector3d &a, const Eigen::Vector3d &b, double distance) {
    using Vec2 = Eigen::Vector2d;
    using line = Eigen::Hyperplane<double, 2>;

    const Vec2 a_start(-distance/2, 0);
    const Vec2 b_start(distance/2,0);
    const Vec2 a2d = a.head(2);
    const Vec2 b2d = b.head(2);

    const line a_line = line::Through(a_start, a2d);
    const line b_line = line::Through(b_start, b2d);

    if(a_line.isApprox(b_line)) {
        return {0,0};
    }

    return a_line.intersection(b_line);
}