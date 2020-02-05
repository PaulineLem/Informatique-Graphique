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

#include <random>
std::default_random_engine engine[8];
std::uniform_real_distribution<double> distrib(0,1);




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

Vector randomcos(const Vector &N) {
    Vector T1;
    if (N[0] <= N[1] && N[0] <= N[2]) {
        T1 = Vector(0, -N[2], N[1]);
    }
    else{
        if (N[1] <= N[0] && N[1] <= N[2]) {
        T1 = Vector(-N[2], 0, N[0]);
        } else {
            T1 = Vector(-N[1], N[0], 0);
        }
    }
    T1.normalize();
    Vector T2 = cross(N, T1);
    
    double r1, r2;
    r1 = distrib(engine[omp_get_thread_num()]);
    r2 = distrib(engine[omp_get_thread_num()]);

    Vector V;
    
    V[0] = cos(2 * M_PI * r1)* sqrt(1-r2);
    V[1] = sin(2 * M_PI * r1)* sqrt(1-r2);
    V[2] = sqrt(r2);
    
    return V[0]*T1 + V[1] * T2 + V[2] *N;
    }
