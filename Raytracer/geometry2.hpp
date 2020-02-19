#include <vector>
#include <string>
#include <map>

class TriangleIndices {
public:
    TriangleIndices(int vtxi = -1, int vtxj = -1, int vtxk = -1, int ni = -1, int nj = -1, int nk = -1, int uvi = -1, int uvj = -1, int uvk = -1) : vtxi(vtxi), vtxj(vtxj), vtxk(vtxk), uvi(uvi), uvj(uvj), uvk(uvk), ni(ni), nj(nj), nk(nk) {
    };
    int vtxi, vtxj, vtxk;
    int uvi, uvj, uvk;
    int ni, nj, nk;
    int faceGroup;
};

class Bbox{
public:
    Bbox(){};
    Bbox (const Vector& bmin, const Vector& bmax): bmin(bmin), bmax(bmax){};
    
    bool intersection(const Ray& r) const {
        double t_1_x = (bmin[0]-r.origin[0])/r.direction[0];
        double t_2_x = (bmax[0]-r.origin[0])/r.direction[0];
        double t_min_x = std::min(t_1_x, t_2_x);
        double t_max_x = std::max(t_1_x, t_2_x);
        
        double t_1_y = (bmin[1]-r.origin[1])/r.direction[1];
        double t_2_y = (bmax[1]-r.origin[1])/r.direction[1];
        double t_min_y = std::min(t_1_y, t_2_y);
        double t_max_y = std::max(t_1_y, t_2_y);
        
        double t_1_z = (bmin[2]-r.origin[2])/r.direction[2];
        double t_2_z = (bmax[2]-r.origin[2])/r.direction[2];
        double t_min_z = std::min(t_1_z, t_2_z);
        double t_max_z = std::max(t_1_z, t_2_z);
        
        if (std::min(std::min(t_max_x,t_max_y),t_max_z) - std::max(std::max(t_min_x, t_min_y), t_min_z) > 0) return true ;
        return false;
    };

    Vector bmin, bmax;
};

class Object {
public :
    Object(){}
    
    virtual bool intersection(const Ray&r, Vector &P, Vector &N, double &t) const = 0;
    Vector albedo;
    bool is_mirror;
    bool is_transparent;
    double emissivity;
    
    
    
};

class Triangle : public Object{
public :
    Triangle(const Vector& A, const Vector &B, const Vector& C, const Vector &color, bool mirror = false, bool transp = false, double emissivity =1) : A(A), B(B), C(C) {
        albedo = color;
        is_mirror =  mirror;
        is_transparent = transp;
        this->emissivity =  emissivity;
    };
    
    bool intersection(const Ray& r, Vector& P, Vector &N, double &t) const {
        
        
        N = cross(B-A, C-A);
        N.normalize();

        
        double denom = dot(r.direction, N);
        if (std::abs(denom)< 1E-12) return false; // ray parallele
        t = dot(C-r.origin, N)/dot(r.direction, N);
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
    const Vector &A, &B, &C;
};


class Geometry : public Object {
public:
    Geometry() {};
        Geometry(const char* obj, double scaling,const Vector& offset, const Vector &color, bool mirror = false, bool transp = false, double emissivity =1) {
        
        albedo = color;
        is_mirror =  mirror;
        is_transparent = transp;
        this->emissivity =  emissivity;
            
        readOBJ(obj);
       
        };

    void readOBJ(const char* obj) {

        char matfile[255];
        char grp[255];

        FILE* f;
        f = fopen(obj, "r");

        std::map<std::string, int> groupNames;
        int curGroup = -1;
        while (!feof(f)) {
            char line[255];
            if (!fgets(line, 255, f)) break;

            std::string linetrim(line);
            linetrim.erase(linetrim.find_last_not_of(" \r\t") + 1);
            strcpy(line, linetrim.c_str());

            if (line[0] == 'u' && line[1] == 's') {
                sscanf(line, "usemtl %[^\n]\n", grp);
                if (groupNames.find(std::string(grp)) != groupNames.end()) {
                    curGroup = groupNames[std::string(grp)];
                }
                else {
                    curGroup = groupNames.size();
                    groupNames[std::string(grp)] = curGroup;
                }
            }
            if (line[0] == 'm' && line[1] == 't' && line[2] == 'l') {
                sscanf(line, "mtllib %[^\n]\n", matfile);
            }
            if (line[0] == 'v' && line[1] == ' ') {
                Vector vec;
                Vector col;
                if (sscanf(line, "v %lf %lf %lf %lf %lf %lf\n", &vec[0], &vec[2], &vec[1], &col[0], &col[1], &col[2]) == 6) {
                    vertices.push_back(vec);
                    vertexcolors.push_back(col);
                }
                else {
                    sscanf(line, "v %lf %lf %lf\n", &vec[0], &vec[2], &vec[1]);  // helmet
                                                                                 //vec[2] = -vec[2]; //car2
                    vertices.push_back(vec);
                }
            }
            if (line[0] == 'v' && line[1] == 'n') {
                Vector vec;
                sscanf(line, "vn %lf %lf %lf\n", &vec[0], &vec[2], &vec[1]); //girl
                normals.push_back(vec);
            }
            if (line[0] == 'v' && line[1] == 't') {
                Vector vec;
                sscanf(line, "vt %lf %lf\n", &vec[0], &vec[1]);
                uvs.push_back(vec);
            }
            if (line[0] == 'f') {
                TriangleIndices t;
                int i0, i1, i2, i3;
                int j0, j1, j2, j3;
                int k0, k1, k2, k3;
                int nn;

                char* consumedline = line + 1;
                int offset;
                t.faceGroup = curGroup;
                nn = sscanf(consumedline, "%u/%u/%u %u/%u/%u %u/%u/%u%n", &i0, &j0, &k0, &i1, &j1, &k1, &i2, &j2, &k2, &offset);
                if (nn == 9) {
                    if (i0 < 0) t.vtxi = vertices.size() + i0; else    t.vtxi = i0 - 1;
                    if (i1 < 0) t.vtxj = vertices.size() + i1; else    t.vtxj = i1 - 1;
                    if (i2 < 0) t.vtxk = vertices.size() + i2; else    t.vtxk = i2 - 1;
                    if (j0 < 0) t.uvi = uvs.size() + j0; else    t.uvi = j0 - 1;
                    if (j1 < 0) t.uvj = uvs.size() + j1; else    t.uvj = j1 - 1;
                    if (j2 < 0) t.uvk = uvs.size() + j2; else    t.uvk = j2 - 1;
                    if (k0 < 0) t.ni = normals.size() + k0; else    t.ni = k0 - 1;
                    if (k1 < 0) t.nj = normals.size() + k1; else    t.nj = k1 - 1;
                    if (k2 < 0) t.nk = normals.size() + k2; else    t.nk = k2 - 1;

                    indices.push_back(t);
                }
                else {
                    nn = sscanf(consumedline, "%u/%u %u/%u %u/%u%n", &i0, &j0, &i1, &j1, &i2, &j2, &offset);
                    if (nn == 6) {
                        if (i0 < 0) t.vtxi = vertices.size() + i0; else    t.vtxi = i0 - 1;
                        if (i1 < 0) t.vtxj = vertices.size() + i1; else    t.vtxj = i1 - 1;
                        if (i2 < 0) t.vtxk = vertices.size() + i2; else    t.vtxk = i2 - 1;
                        if (j0 < 0) t.uvi = uvs.size() + j0; else    t.uvi = j0 - 1;
                        if (j1 < 0) t.uvj = uvs.size() + j1; else    t.uvj = j1 - 1;
                        if (j2 < 0) t.uvk = uvs.size() + j2; else    t.uvk = j2 - 1;
                        indices.push_back(t);
                    }
                    else {
                        nn = sscanf(consumedline, "%u %u %u%n", &i0, &i1, &i2, &offset);
                        if (nn == 3) {
                            if (i0 < 0) t.vtxi = vertices.size() + i0; else    t.vtxi = i0 - 1;
                            if (i1 < 0) t.vtxj = vertices.size() + i1; else    t.vtxj = i1 - 1;
                            if (i2 < 0) t.vtxk = vertices.size() + i2; else    t.vtxk = i2 - 1;
                            indices.push_back(t);
                        }
                        else {
                            nn = sscanf(consumedline, "%u//%u %u//%u %u//%u%n", &i0, &k0, &i1, &k1, &i2, &k2, &offset);
                            if (i0 < 0) t.vtxi = vertices.size() + i0; else    t.vtxi = i0 - 1;
                            if (i1 < 0) t.vtxj = vertices.size() + i1; else    t.vtxj = i1 - 1;
                            if (i2 < 0) t.vtxk = vertices.size() + i2; else    t.vtxk = i2 - 1;
                            if (k0 < 0) t.ni = normals.size() + k0; else    t.ni = k0 - 1;
                            if (k1 < 0) t.nj = normals.size() + k1; else    t.nj = k1 - 1;
                            if (k2 < 0) t.nk = normals.size() + k2; else    t.nk = k2 - 1;
                            indices.push_back(t);
                        }
                    }
                }


                consumedline = consumedline + offset;

                while (true) {
                    if (consumedline[0] == '\n') break;
                    if (consumedline[0] == '\0') break;
                    nn = sscanf(consumedline, "%u/%u/%u%n", &i3, &j3, &k3, &offset);
                    TriangleIndices t2;
                    t2.faceGroup = curGroup;
                    if (nn == 3) {
                        if (i0 < 0) t2.vtxi = vertices.size() + i0; else    t2.vtxi = i0 - 1;
                        if (i2 < 0) t2.vtxj = vertices.size() + i2; else    t2.vtxj = i2 - 1;
                        if (i3 < 0) t2.vtxk = vertices.size() + i3; else    t2.vtxk = i3 - 1;
                        if (j0 < 0) t2.uvi = uvs.size() + j0; else    t2.uvi = j0 - 1;
                        if (j2 < 0) t2.uvj = uvs.size() + j2; else    t2.uvj = j2 - 1;
                        if (j3 < 0) t2.uvk = uvs.size() + j3; else    t2.uvk = j3 - 1;
                        if (k0 < 0) t2.ni = normals.size() + k0; else    t2.ni = k0 - 1;
                        if (k2 < 0) t2.nj = normals.size() + k2; else    t2.nj = k2 - 1;
                        if (k3 < 0) t2.nk = normals.size() + k3; else    t2.nk = k3 - 1;
                        indices.push_back(t2);
                        consumedline = consumedline + offset;
                        i2 = i3;
                        j2 = j3;
                        k2 = k3;
                    }
                    else {
                        nn = sscanf(consumedline, "%u/%u%n", &i3, &j3, &offset);
                        if (nn == 2) {
                            if (i0 < 0) t2.vtxi = vertices.size() + i0; else    t2.vtxi = i0 - 1;
                            if (i2 < 0) t2.vtxj = vertices.size() + i2; else    t2.vtxj = i2 - 1;
                            if (i3 < 0) t2.vtxk = vertices.size() + i3; else    t2.vtxk = i3 - 1;
                            if (j0 < 0) t2.uvi = uvs.size() + j0; else    t2.uvi = j0 - 1;
                            if (j2 < 0) t2.uvj = uvs.size() + j2; else    t2.uvj = j2 - 1;
                            if (j3 < 0) t2.uvk = uvs.size() + j3; else    t2.uvk = j3 - 1;
                            consumedline = consumedline + offset;
                            i2 = i3;
                            j2 = j3;
                            indices.push_back(t2);
                        }
                        else {
                            nn = sscanf(consumedline, "%u//%u%n", &i3, &k3, &offset);
                            if (nn == 2) {
                                if (i0 < 0) t2.vtxi = vertices.size() + i0; else    t2.vtxi = i0 - 1;
                                if (i2 < 0) t2.vtxj = vertices.size() + i2; else    t2.vtxj = i2 - 1;
                                if (i3 < 0) t2.vtxk = vertices.size() + i3; else    t2.vtxk = i3 - 1;
                                if (k0 < 0) t2.ni = normals.size() + k0; else    t2.ni = k0 - 1;
                                if (k2 < 0) t2.nj = normals.size() + k2; else    t2.nj = k2 - 1;
                                if (k3 < 0) t2.nk = normals.size() + k3; else    t2.nk = k3 - 1;
                                consumedline = consumedline + offset;
                                i2 = i3;
                                k2 = k3;
                                indices.push_back(t2);
                            }
                            else {
                                nn = sscanf(consumedline, "%u%n", &i3, &offset);
                                if (nn == 1) {
                                    if (i0 < 0) t2.vtxi = vertices.size() + i0; else    t2.vtxi = i0 - 1;
                                    if (i2 < 0) t2.vtxj = vertices.size() + i2; else    t2.vtxj = i2 - 1;
                                    if (i3 < 0) t2.vtxk = vertices.size() + i3; else    t2.vtxk = i3 - 1;
                                    consumedline = consumedline + offset;
                                    i2 = i3;
                                    indices.push_back(t2);
                                }
                                else {
                                    consumedline = consumedline + 1;
                                }
                            }
                        }
                    }
                }

            }


        }
        fclose(f);
        
        bb.bmin =vertices[0];
        bb.bmax =vertices[0];
                   


       for (int i=1; i<vertices.size(); i++) {
           for (int j=0; j<3; j++) {

               bb.bmin[j] = std::min(bb.bmin[j],vertices[i][j]);
               bb.bmax[j] = std::max(bb.bmax[j],vertices[i][j]);

           }
       
       }

        
    }


    void add_texture(const char* filename) {

        textures.resize(textures.size() + 1);
        w.resize(w.size() + 1);
        h.resize(h.size() + 1);

        FILE* f;
        f = fopen(filename, "rb");
        unsigned char info[54];
        fread(info, sizeof(unsigned char), 54, f); // read the 54-byte header

        w[w.size() - 1] = *(int*)&info[18]; // extract image height and width from header
        h[h.size() - 1] = *(int*)&info[22];

        int size = 3 * w[w.size() - 1] * h[h.size() - 1];
        textures[textures.size() - 1].resize(size); // allocate 3 bytes per pixel
        fread(&textures[textures.size() - 1][0], sizeof(unsigned char), size, f); // read the rest of the data at once
        fclose(f);

        for (int i = 0; i < size; i += 3) {
            std::swap(textures[textures.size() - 1][i], textures[textures.size() - 1][i + 2]);
        }
    }
    bool intersection(const Ray& r, Vector& P, Vector &N, double &t) const {
        if (!bb.intersection(r)) return false;
        t=1E99;
        bool has_intersection = false;
        for (int i=0; i<indices.size(); i++ ){
            
            
            int i0 = indices[i].vtxi ;
            int i1 = indices[i].vtxj;
            int i2 = indices[i].vtxk;

            Triangle tri(vertices[i0], vertices[i1], vertices[i2], albedo, is_mirror, is_transparent, emissivity);
            Vector localP, localN;
            double localt;
            if (tri.intersection(r, localP, localN, localt)) {
                has_intersection = true;
                if(localt<t){
                    t=localt;
                    P=localP;
                    N=localN;
                }
            }
        }
        return has_intersection;
        
    };
    
//    std::vector<int> faceGroup;
//    std::vector<int> normalIds;
//    std::vector<int> uvIds;
    std::vector<TriangleIndices> indices;
    std::vector<Vector> vertices;
    std::vector<Vector> normals;
    std::vector<Vector> uvs; // Vector en 3D mais on n'utilise que 2 composantes
    std::vector<Vector> vertexcolors;

    std::vector<std::vector<unsigned char> > textures;
    std::vector<int> w, h;
    
    
    Vector albedo;
    bool is_mirror;
    bool is_transparent;
    double emissivity;
private :
    Bbox bb;
};


//class Triangle : public Object{
//public :
//    Triangle(const Vector& A, const Vector &B, const Vector& C, const Vector &color, bool mirror = false, bool transp = false, double emissivity =1) : A(A), B(B), C(C) {
//        albedo = color;
//        is_mirror =  mirror;
//        is_transparent = transp;
//        this->emissivity =  emissivity;
//    };
//
//    bool intersection(const Ray& r, Vector& P, Vector &N, double &t) const {
//
//
//        N = cross(B-A, C-A);
//        N.normalize();
//
//
//        double denom = dot(r.direction, N);
//        if (std::abs(denom)< 1E-12) return false; // ray parallele
//        t = dot(A-r.origin, N)/dot(r.direction, N);
//        if (t<0) return false ; //intersect derriere
//
//        P = r.origin + t*r.direction;
//        double APAB = dot(P-A, B-A);
//        double ACAB = dot(C-A, B-A);
//        double ABAB = dot(B-A, B-A);
//        double APAC = dot(P-A, C-A);
//        double ACAC = dot(C-A, C-A);
//        double det = ABAB*ACAC-ACAB*ACAB;
//        double beta = (APAB*ACAC-APAC*ACAB)/det;
//        double gamma = (ABAB*APAC -ACAB*APAB)/det;
//        if (beta<0) return false;
//        if (gamma<0) return false;
//        if (beta+gamma>1) return false;
//        return true;
//
//    };
//    const Vector &A, &B, &C;
//};


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
    void addGeometry(const Geometry& s) {objects.push_back((Object*)&s);}


    
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
