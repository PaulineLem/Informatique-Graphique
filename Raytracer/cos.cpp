//
//  cos.cpp
//  Raytracer
//
//  Created by Pauline Lemeille on 29/01/2020.
//  Copyright © 2020 Pauline Lemeille. All rights reserved.
//

#include <thread>
#include <iostream>
#include <cmath>
#include <cstdlib>


double cos30(double x) {
    return std::pow(std::cos(x),30);
}

double p(double x) {
    double sigma = M_PI/2;
    return 1/(sigma* sqrt(2*M_PI))*std::exp(- x*x /(2*sigma*sigma));
}


int x_main() {
    double N = 100000;
    double somme = 0;
    
    for (int i = 0; i < N; i += 1){
        double x;
        x = -M_PI/2 + i* M_PI/N;
//        std::cout << x;
        
        somme+= cos30(x)/p(x);
    }
        
    double r1, r2;
    r1 = random()*1.0 / (INT_MAX);
    r2 = random()*1.0  / (INT_MAX);
    
    double x1, x2;
    x1 = std::sqrt(-2* std::log(r1)*std::cos(2 * M_PI * r2))*0.25;
    x2 = std::sqrt(-2* std::log(r1)*std::sin(2 * M_PI * r2))*0.25;
    
    std::cout << somme/N;

    return 1;
}
