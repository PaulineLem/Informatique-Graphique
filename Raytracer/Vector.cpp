//
//  Vector.cpp
//  Raytracer
//
//  Created by Pauline Lemeille on 13/01/2020.
//  Copyright © 2020 Pauline Lemeille. All rights reserved.
//

#include "Vector.h"
#include <vector>
#include <omp.h>
#include "random.hpp"

#include <random>


Vector operator+(const Vector &v1, const Vector &v2) {
    return Vector(v1[0]+v2[0], v1[1]+v2[1], v1[2]+v2[2]);
}
Vector operator-(const Vector &v1, const Vector &v2) {
    return Vector(v1[0]-v2[0], v1[1]-v2[1], v1[2]-v2[2]);
}
Vector operator-(const Vector &v1) {
    return Vector(-v1[0], -v1[1], -v1[2]);
}
Vector operator*(double k, const Vector &v2) {
    return Vector(k*v2[0], k*v2[1], k*v2[2]);
}
Vector operator*(const Vector &v2, double k) {
    return Vector(k*v2[0], k*v2[1], k*v2[2]);
}
Vector operator*(const Vector &v1, const Vector &v2) {
    return Vector(v1[0]*v2[0], v1[1]*v2[1], v1[2]*v2[2]);
}
Vector operator/(const Vector &v1, double k) {
    return Vector(v1[0]/k, v1[1]/k, v1[2]/k);
}
double dot(const Vector &v1, const Vector &v2) {
    return v1[0]*v2[0]+v1[1]*v2[1]+v1[2]*v2[2];
}
Vector cross(const Vector& v1, const Vector &v2) {
    return Vector(v1[1]*v2[2]- v1[2]*v2[1], v1[2]*v2[0]- v1[0]*v2[2], v1[0]*v2[1]- v1[1]*v2[0]);
}


