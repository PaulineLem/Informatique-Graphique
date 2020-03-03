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
    
//    Vector reflect(const Vector& N) const {
//        Vector refl = *this - 2. * dot(*this, N)*N;
//        return refl;
//    }

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
