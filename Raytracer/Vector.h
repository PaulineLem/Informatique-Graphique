#include <math.h>
#include <iostream>
#include <vector>



class Vector {
public:
    Vector (double x=0, double y= 0, double z =0) {
        coord[0]= x;
        coord[1]= y;
        coord[2]= z;
    }
    
    const double& operator[](int i) const { return coord[i]; }
    
    double& operator[](int i) { return coord[i]; }
    
    double getNorm2(){
        return coord[0]*coord[0]+coord[1]*coord[1]+coord[2]*coord[2];
    }
    
    void normalize() {
        double norm = sqrt(getNorm2());
        coord[0]/= norm;
        coord[1]/= norm;
        coord[2]/= norm;
    }
    
    Vector getNormalized(){
        Vector result(*this);
        result.normalize();
        return result;
    }
    Vector& operator+=(const Vector& v2){
        coord[0] += v2[0];
        coord[1] += v2[1];
        coord[2] += v2[2];
        return *this;
    }

    
private:
    double coord[3];
};

Vector operator+(const Vector& v1, const Vector &v2);
Vector operator-(const Vector& v1, const Vector &v2);
Vector operator-(const Vector& v1);
Vector operator*(double k, const Vector &v2);
Vector operator*(const Vector &v2, double k);
Vector operator*(const Vector& v1, const Vector &v2);
Vector operator/(const Vector &v1, double k);
double dot(const Vector& v1, const Vector &v2);
Vector cross(const Vector& v1, const Vector &v2);
Vector randomcos(const Vector &N);


class Ray {
public :
    Ray(const Vector& o, const Vector& d ) : origin(o), direction(d) {};
    Vector origin, direction;
};

class Sphere {
public :
    Sphere(const Vector &origin, double rayon, const Vector &color, bool mirror = false, bool transp = false, double emissivity =1) : O(origin), R(rayon), albedo(color), is_mirror(mirror), is_transparent(transp), emissivity(emissivity){};
    
    bool intersection (const Ray& r,  Vector& P, Vector& N, double &t) const {
        
        double a =1;
        double b = 2*dot(r.direction, r.origin - O);
        double c = (r.origin - O).getNorm2() - R*R;
        
        double delta = b*b - 4*a*c;
        
        if (delta<0) return false ;
        double t1 = (-b - sqrt(delta))/ (2*a);
        double t2 = (-b + sqrt(delta))/ (2*a);
        if (t2<0) return false;

        if (t1>0)
            t=t1;
        else
            t=t2;
        
        P = r.origin + t*r.direction;
        N = (P-O).getNormalized();
        
        return true;
    };
    
    Vector O;
    double R;
    Vector albedo;
    bool is_mirror;
    bool is_transparent;
    double intensity;
    double emissivity;
};

class Scene {
public :
    Scene() {};
    void addSphere(const Sphere& s) {spheres.push_back(s);}
    
    bool intersection (const Ray& r,  Vector& P, Vector& N, int& sphere_id, double& min_t) {
        
        bool has_intersection = false;
        min_t = 1E99;
        
        for (int i=0; i<spheres.size(); i++) {
            Vector localP, localN;
            double t;
            bool local_has_intersection = spheres[i].intersection(r, localP, localN, t);
            if (local_has_intersection){
                has_intersection = true;
                if (t<min_t){
                    min_t =t;
                    P= localP;
                    N=  localN;
                    sphere_id = i;
                }
            }
        }
        return has_intersection;
        };
    

    Sphere *lumiere;
    double light_intensity;
    std::vector<Sphere> spheres;
};
