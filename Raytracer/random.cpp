#include "Vector.h"
#include <vector>
#include <omp.h>

#include <random>

std::default_random_engine engine[8];
std::uniform_real_distribution<double> distrib(0,1);

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

Vector randomcos_2(const Vector &N) {
    
    
    double r1, r2;
    r1 = distrib(engine[omp_get_thread_num()]);
    r2 = distrib(engine[omp_get_thread_num()]);
    
    
    Vector V;
    
    V[0] = cos(2 * M_PI * r1)* sqrt(1-r2);
    V[1] = sin(2 * M_PI * r1)* sqrt(1-r2);
    V[2] = sqrt(r2);
    
    Vector aleatoire (distrib(engine[omp_get_thread_num()]) - 0.5, distrib(engine[omp_get_thread_num()])-0.5, distrib(engine[omp_get_thread_num()])-0.5);
    
    Vector tangent1= cross(N, aleatoire);
    tangent1.normalize();
    Vector tangent2 = cross(tangent1, N);
    
    
    return V[0]*tangent1 + V[1] * tangent2+ V[2] *N;
}

Vector random_2() {
    double r1, r2;
    r1 = distrib(engine[omp_get_thread_num()]);
    r2 = distrib(engine[omp_get_thread_num()]);
    
    return Vector(r1,r2);
}

