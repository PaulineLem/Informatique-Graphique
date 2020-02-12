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

class Object {
public :
    Object(){}
    
    virtual bool intersection(const Ray&r, Vector &P, Vector &N, double &T) const = 0;
    Vector albedo;
    bool is_mirror;
    bool is_transparent;
    double emissivity;
    
    
};

class Triangle : public Object{
public :
    Triangle(const Vector& A, const Vector &B, const Vector& C, const Vector &color, bool mirror = false, bool transp = false, double emissivity =1) : A(A), B(B), C(C) {
        albedo = color;
        is_mirror = mirror;
        is_transparent = transp;
        this->emissivity =  emissivity;
    };
    
    bool intersection(const Ray& r, Vector& P, Vector &N, double &t) const {
        
        
        N = cross(B-A, C-A);
        N.normalize();

        
        double denom = dot(r.direction, N);
        if (std::abs(denom)< 1E-12) return false; // ray parallele
        t = dot(A-r.origin, N)/dot(r.direction, N);
        if (t<0) return false ; //intersect derriere
        
        P = r.origin + t*r.direction;
        double APAB = dot(P-A, B-A);
        double ACAB = dot(C-A, B-A);
        double ABAB = dot(B-A, B-A);
        double APAC = dot(P-A, C-A);
        double ACAC = dot(C-A, C-A);
        double det = ABAB*ACAC-ACAB*ACAB;
        double beta = (APAB*ACAC-APAC*ACAB)/det;
        double gamma = (ABAB*APAC -ACAB*APAB)/det;
        if (beta<0) return false;
        if (gamma<0) return false;
        if (beta+gamma>1) return false;
        return true;
            
    };
    Vector A, B, C, N;
};


class Sphere : public Object{
public :
    Sphere(const Vector &origin, double rayon, const Vector &color, bool mirror = false, bool transp = false, double emissivity =1) : O(origin), R(rayon){
        albedo = color;
        is_mirror = mirror;
        is_transparent = transp;
        this->emissivity = emissivity;
    };

    
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

};

class Scene {
public :
    Scene() {};
    void addSphere(const Sphere& s) {objects.push_back((Object*)&s);}
    void addTriangle(const Triangle& s) {objects.push_back((Object*)&s);}

    
    bool intersection (const Ray& r,  Vector& P, Vector& N, int& sphere_id, double& min_t) {
        
        bool has_intersection = false;
        min_t = 1E99;
        
        for (int i=0; i<objects.size(); i++) {
            Vector localP, localN;
            double t;
            bool local_has_intersection = objects[i]->intersection(r, localP, localN, t);
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
    std::vector<Object*> objects;


};
