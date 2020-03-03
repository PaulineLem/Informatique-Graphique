
 
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
 
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

// Raytracer.cpp : Defines the entry point for the console application.
#define _CRT_SECURE_NO_WARNINGS // for Visual Studio 2017 (maybe 2015 as well)

#include <iostream>
#include <vector>
#include "Vector.h"
#include "random.hpp"
#include "geometry2.hpp"

#include <omp.h>
#include <random>
#include <algorithm>

#include <typeinfo>


//



void save_image(const char* filename, const unsigned char* tableau, int w, int h) { // (0,0) is top-left corner
 
    FILE *f;
 
    int filesize = 54 + 3 * w*h;
 
    unsigned char bmpfileheader[14] = { 'B','M', 0,0,0,0, 0,0, 0,0, 54,0,0,0 };
    unsigned char bmpinfoheader[40] = { 40,0,0,0, 0,0,0,0, 0,0,0,0, 1,0, 24,0 };
    unsigned char bmppad[3] = { 0,0,0 };
 
    bmpfileheader[2] = (unsigned char)(filesize);
    bmpfileheader[3] = (unsigned char)(filesize >> 8);
    bmpfileheader[4] = (unsigned char)(filesize >> 16);
    bmpfileheader[5] = (unsigned char)(filesize >> 24);
 
    bmpinfoheader[4] = (unsigned char)(w);
    bmpinfoheader[5] = (unsigned char)(w >> 8);
    bmpinfoheader[6] = (unsigned char)(w >> 16);
    bmpinfoheader[7] = (unsigned char)(w >> 24);
    bmpinfoheader[8] = (unsigned char)(h);
    bmpinfoheader[9] = (unsigned char)(h >> 8);
    bmpinfoheader[10] = (unsigned char)(h >> 16);
    bmpinfoheader[11] = (unsigned char)(h >> 24);
 
    f = fopen(filename, "wb");
    fwrite(bmpfileheader, 1, 14, f);
    fwrite(bmpinfoheader, 1, 40, f);
    unsigned char *row = new unsigned char[w * 3];
    for (int i = 0; i < h; i++)
    {
        for (int j = 0; j < w; j++) {
            row[j * 3] = tableau[(w*(h - i - 1) * 3) + j * 3+2];
            row[j * 3+1] = tableau[(w*(h - i - 1) * 3) + j * 3+1];
            row[j * 3+2] = tableau[(w*(h - i - 1) * 3) + j * 3];
        }
        fwrite(row, 3, w, f);
        fwrite(bmppad, 1, (4 - (w * 3) % 4) % 4, f);
    }
    fclose(f);
    delete[] row;
}
 

Vector getColor(const Ray& r, Scene& s, int nbrefl) {
    
        double eps=0.01;
        if (nbrefl == 0) return Vector(0,0,0);
        Vector P, N, albedo;
        int sphere_id;
        double t;
        bool has_intersection = s.intersection(r, P,N, sphere_id, t, albedo);
//    bool has_intersection = s.intersection(r, P,N, sphere_id, t);

        Vector pixel_intensity(0,0,0);
        
    
        if (has_intersection) {
            if(s.objects[sphere_id]->is_mirror) {
                Vector dir_mir = r.direction - 2*dot(N, r.direction)*N;
                Ray mirrorRay(P+eps*N, dir_mir);
                pixel_intensity = getColor(mirrorRay, s, nbrefl-1);
            }
            else {
                if (s.objects[sphere_id]->is_transparent) {
                    double n1=1;
                    double n2=1.3;
                    Vector Ntransp(N);
                    if(dot(r.direction,N)>0) {
                        //out of the sphere
                        n1=1.3;
                        n2=1;
                        Ntransp = -N;
                    }
                    double radical = 1-pow((n1/n2),2)*(1-pow((dot(Ntransp , r.direction)),2));
                    if (radical > 0) {
                        Vector dir_ref = (n1/n2)*(r.direction - dot(r.direction, Ntransp)*Ntransp) - Ntransp * sqrt (radical);
                        Ray refRay(P - eps*Ntransp, dir_ref);
                        pixel_intensity = getColor(refRay, s, nbrefl-1);
                    }
                }
                    
                else {
                
                    
//                    Eclairage direct
                    Vector L = s.lumiere->O;
//                    Vector LP = L-P;
                    Vector LP = P-L;
                    LP.normalize();
                    Vector randSdir = randomcos_2(LP);
                    Vector xi = L + randSdir*dynamic_cast<Sphere*>(s.objects[0])->R;
                    Vector wi = xi-P;
                    wi.normalize();
                    double d2 = (xi-P).getNorm2();
                    Vector Np= randSdir;
                    double costheta = std::max(0., dot(N, wi));
                    double costhetaprime = dot(Np,-wi );
//                    double costhetasecond = std::max(0.,dot(randSdir,LP ));
                    double costhetasecond = dot(LP,randSdir);

                                    
                    Vector light_or = P + eps * N;
                    Ray light_ray(light_or, wi);
                    Vector P_light, N_light, albedo_light;
                    int sphere_id_light;
                    double t_light;
                    bool has_intersection_light= s.intersection(light_ray, P_light, N_light, sphere_id_light, t_light, albedo_light);
//                                        bool has_intersection_light= s.intersection(light_ray, P_light, N_light, sphere_id_light, t_light);
                    
                    
//                    pixel_intensity = (s.light_intensity/(4*M_PI*d2)*costheta*costhetaprime/costhetasecond)* s.objects[sphere_id]->albedo;

                    if ( has_intersection_light && t_light*t_light < d2*0.99 ){
                        pixel_intensity = Vector(0,0,0);

                    }

                    else {
//                        pixel_intensity = (s.light_intensity/(4*M_PI*d2)*costheta*costhetaprime/costhetasecond)* s.objects[sphere_id]->albedo;
                        pixel_intensity = (s.light_intensity/(4*M_PI*d2)*costheta*costhetaprime/costhetasecond)* albedo;
                    }

                        
//                       Eclairage indirect

                    wi= randomcos_2(N);
                    Ray indirectRay(P+eps*N, wi);
//                    pixel_intensity += getColor(indirectRay, s, nbrefl-1) * s.objects[sphere_id]->albedo ;
                    pixel_intensity += getColor(indirectRay, s, nbrefl-1) * albedo ;

//                    pixel_intensity += s.objects[sphere_id]->albedo * s.objects[sphere_id]->emissivity;
                    
//                    if (sphere_id==5){
//                    std :: cout << s.objects[sphere_id]->albedo[0];
//                    }

                

                }
            }
            
        }

    
        
        return pixel_intensity;
}
    



int main() {

    int W = 1024;
    int H = 1024;
    double fov = 60 * M_PI / 180;
    int nb_ray = 80;
    
    Scene s;
    s.light_intensity = 10000000000;
    double R = 15;
    Vector cameraPosition = (static_cast<void>(0.) ,static_cast<void>(0.) ,0.);
    double focus_distance = 35.;
    
    

    
    
    Sphere slum(Vector(15, 70, -40), R,Vector (1,1,1),false, false);
//    Sphere slum(Vector(0, 20, focus_distance), R,Vector (1,1,1));

    Sphere s1(Vector(-15,0, -focus_distance),10, Vector (1,1,1), true);
    Sphere s7(Vector(15,0, -focus_distance),10, Vector (1,1,1), false, true);
    Sphere s2(Vector(0,-2000-20, 0),2000, Vector (1,1,0)); //ground
    Sphere s3(Vector(0,200+100, 0),2000, Vector (1,0,1)); //ceiling
    Sphere s4(Vector(-2000-50,0, 0),2000, Vector (0,1,1)); // left wall
    Sphere s5(Vector(2000+50,0, 0),2000, Vector (0,0,1)); // right wall
    Sphere s6(Vector(0, 0, -2000-100),2000, Vector (0,1,0)); // back wall
    
    Triangle tri(Vector(-10, -10, -55), Vector(10, -10, -20), Vector(0, 10, -20), Vector(1, 0, 0), false, false, false);
    
    Geometry g1("Beautiful Girl.obj", 10, Vector(0,-20, -35), Vector (1,1,1));
    
    s.addSphere(slum);
    s.addGeometry(g1);

    
//    s.addTriangle(tri);
//    s.addSphere(s1);
    s.addSphere(s2);
    s.addSphere(s3);
    s.addSphere(s4);
    s.addSphere(s5);
    s.addSphere(s6);

//    s.addSphere(s7);
    s.lumiere = &slum;

    
   
    std::vector<unsigned char> image(W*H * 3);
    
    
#pragma omp parallel for

// Lance des thread pour la boucle for suivante
    for (int i = 0; i < H; i++) {

        for (int j = 0; j < W; j++) {
            
            Vector Color(0.,0.,0.);
            for (int k = 0; k < nb_ray; k++) {
                
                double r1, r2, dx, dy;
                Vector rand = random_2();
                
                r1 = rand[0];
                r2 = rand[1];


                dx = cos(2 * M_PI * r2)* sqrt(-2*log(r1));
                dy = sin(2 * M_PI * r2)* sqrt(-2*log(r1));

                
                Vector rand2 = random_2();
                double dx_apperture = (rand2[0] -0.5) *0.5 ;
                double dy_apperture = (rand2[1]-0.5) *0.5;

                
                Vector direction(j-W/2 +0.5 +dx, i-H/2+0.5+dy, -W/ (2*tan(fov/2)));
                direction.normalize();
                
                Vector destination = cameraPosition + focus_distance * direction;
                Vector new_origin = cameraPosition +Vector (dx_apperture, dy_apperture, 0);
                Ray r(new_origin, (destination-new_origin).getNormalized());
//                Ray r(Vector(0,0,0), direction);
                Color += getColor(r, s, 5) / nb_ray;
            }
//            Vector Color = getColor(r, s, 5) ;
//            std::cout << Color[0];

            image[((H-i-1)*W + j) * 3 + 0] = std::min(255., std::max(0.,pow(Color[0], 1/2.2)));
            image[((H-i-1)*W + j) * 3 + 1] = std::min(255., std::max(0.,pow(Color[1], 1/2.2)));
            image[((H-i-1)*W + j) * 3 + 2] = std::min(255., std::max(0.,pow(Color[2], 1/2.2)));
        }
    }
    save_image("seance6-test-beautiful-triangle-1-plus-de-ray.bmp",&image[0], W, H);
    
    
 
    return 0;
}
