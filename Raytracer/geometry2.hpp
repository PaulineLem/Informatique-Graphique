#include <vector>
#include <string>
#include <map>
#include<list>

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
        
        double t_far = std::min(std::min(t_max_x,t_max_y),t_max_z) ;
        if(t_far<0) return false;
        
        if (std::min(std::min(t_max_x,t_max_y),t_max_z) - std::max(std::max(t_min_x, t_min_y), t_min_z) > 0) return true ;
        return false;
    };
    bool intersection(const Ray& r, double &t) const {
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
        
        double t_far = std::min(std::min(t_max_x,t_max_y),t_max_z) ;
        if(t_far<0) return false;
        
        double t_close =std::max(std::max(t_min_x, t_min_y), t_min_z);
        if(t_close>0) t=t_close; else t=0;
        
        if (t_far - t_close > 0) return true ;
        return false;
    };
    
    Vector bmin, bmax;
};

class BVH{
public:
    BVH *fg, *fd;
    int i0, i1;
    Bbox bbox;
    
};

class Object {
    public :
    Object(){}
    
    virtual bool intersection(const Ray&r, Vector &P, Vector &N, double &t, Vector &color) const = 0;
 
    Vector albedo;
    bool is_mirror;
    bool is_transparent;
    
};

class Triangle : public Object{
    public :
    Triangle(const Vector& A, const Vector &B, const Vector& C, const Vector &coleur, bool mirror = false, bool transp = false) : A(A), B(B), C(C) {
        albedo = coleur;
        is_mirror =  mirror;
        is_transparent = transp;
    };
    
    bool intersection(const Ray& r, Vector& P, Vector &N, double &t, Vector &color) const {
        double alpha, beta, gamma;
        color = albedo;
        return intersection(r, P, N, t, alpha, beta, gamma);
    };
    
    bool intersection(const Ray& r, Vector& P, Vector &N, double &t, double &alpha, double &beta, double &gamma) const {
        
        N = -cross(B-A, C-A);
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
        beta = (APAB*ACAC-APAC*ACAB)/det;
        gamma = (ABAB*APAC -ACAB*APAB)/det;
        alpha = 1-beta-gamma;
        if (beta<0 || beta>1) return false;
        if (gamma<0 || gamma>1) return false;
        if (alpha<0 || alpha>1) return false;
        return true;
    };
    
    const Vector &A, &B, &C;
};


class Geometry : public Object {
public:
    Geometry() {};
    Geometry(const char* obj, double scaling,const Vector& offset, const Vector &coleur, bool mirror = false, bool transp = false) {
        
        albedo = coleur;
        is_mirror =  mirror;
        is_transparent = transp;
        
        readOBJ(obj);
        

        for (int i = 0; i < vertices.size(); i++) {
            vertices[i] = scaling * vertices[i] + offset;
        }
        
        build_bvh(&bvh, 0, indices.size());
        
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
        
        f = fopen("Beautiful Girl.mtl","r");
        while (!feof(f)) {
            char line[255];
            fgets(line, 255, f);
            if (line[0]=='m' && line[4]=='K' && line[5]=='d'){
                char texturefile[255];
                sscanf(line, "map_Kd %100s\n", texturefile);
                add_texture((std::string("texturesbmp/") + std::string(texturefile)).c_str());
            }
        }
        fclose(f);
    }
    

    
    Bbox build_bb(int i0, int i1){
        Bbox bb;
        bb.bmin =vertices[indices[i0].vtxi];
        bb.bmax =vertices[indices[i0].vtxi];
        for (int i=i0; i<i1; i++) {// triangle
            for( int k=0; k<3; k++){//dimension
                bb.bmin[k] = std::min(bb.bmin[k],vertices[indices[i].vtxi][k]);
                bb.bmax[k] = std::max(bb.bmax[k],vertices[indices[i].vtxi][k]);
                
                bb.bmin[k] = std::min(bb.bmin[k],vertices[indices[i].vtxj][k]);
                bb.bmax[k] = std::max(bb.bmax[k],vertices[indices[i].vtxj][k]);
                
                bb.bmin[k] = std::min(bb.bmin[k],vertices[indices[i].vtxk][k]);
                bb.bmax[k] = std::max(bb.bmax[k],vertices[indices[i].vtxk][k]);
                
            }
        }
        
        return bb;
    };
    
    void build_bvh(BVH* node, int i0, int i1){
        node -> bbox =build_bb(i0,i1);
        node->i0 = i0;
        node->i1 = i1;
        node->fg = NULL;
        node->fd = NULL;

        Vector diag = node->bbox.bmax-node->bbox.bmin;
        int split_dim;
        if((diag[0]>diag[1]) && (diag[0]>diag[2])){
            
            split_dim = 0;
            
        }
        else {
            if ((diag[1]>diag[0]) && (diag[1]>diag[2]))
            {
                split_dim = 1;
            }
            
            else {
                split_dim =2 ;
            }
            
        }
        double split_val = node -> bbox.bmin[split_dim] + diag[split_dim]/2;
        
        int pivot = i0-1;
        
        for(int i=i0; i<i1; i++){
            double center_split_dim = (vertices[indices[i].vtxi][split_dim] + vertices[indices[i].vtxj][split_dim] + vertices[indices[i].vtxk][split_dim])/3;
            if(center_split_dim <split_val){
                pivot ++;
                std::swap(indices[i].vtxi, indices[pivot].vtxi);
                std::swap(indices[i].vtxj, indices[pivot].vtxj);
                std::swap(indices[i].vtxk, indices[pivot].vtxk);
                
                std::swap(indices[i].ni, indices[pivot].ni);
                std::swap(indices[i].nj, indices[pivot].nj);
                std::swap(indices[i].nk, indices[pivot].nk);
                
                std::swap(indices[i].uvi, indices[pivot].uvi);
                std::swap(indices[i].uvj,indices[pivot].uvj);
                std::swap(indices[i].uvk, indices[pivot].uvk);
                
                std::swap(indices[i].faceGroup, indices[pivot].faceGroup);
                
            }
        }
        
        if(pivot < i0|| pivot>=i1-1 || i1==i0+1){
            return;
        }
        node->fg= new BVH();
        build_bvh(node->fg, i0, pivot+1);
        
        node->fd= new BVH();
        build_bvh(node->fd, pivot+1, i1);
        
    };
    
    
    void add_texture(const char* filename) {
        
        textures.resize(textures.size() + 1);
        w.resize(w.size() + 1);
        h.resize(h.size() + 1);
        
        FILE* f;
        f = fopen(filename, "r");
        
        unsigned char info[54];
        
        fread(info, sizeof(unsigned char), 54, f); // read the 54-byte header

        w[w.size() - 1] = *(int*)&info[18]; // extract image height and width from header
        h[h.size() - 1] = *(int*)&info[22];
        
        int size = 3 * w[w.size() - 1] * h[h.size() - 1];
        
        
        textures[textures.size() - 1].resize(size);
        
        
        fread(&textures[textures.size() - 1][0], sizeof(unsigned char), size, f); // read the rest of the data at once
        fclose(f);
        
        
        for (int i = 0; i < size; i += 3) {
            std::swap(textures[textures.size() - 1][i], textures[textures.size() - 1][i + 2]);
        }
    };
    
    bool intersection(const Ray& r, Vector& P, Vector &N, double &t, Vector &color) const {

        t=1E99;
        bool has_intersection = false;
        if (!bvh.bbox.intersection(r)) return false;
        
//        std::list<const BVH*> l;
        const BVH* l[40];
        int idx_back = -1;
        
        
        

//        l.push_front(&bvh);
        idx_back++; l[idx_back]=&bvh;

        
        
        while(idx_back>=0){
//            const BVH* current = l.front();
            const BVH* current = l[idx_back];

//            l.pop_front();
            idx_back --;
            // font -> back, permet de faire un meilleur parcours de la liste (en profonfeur)
            double t_boxe ;
            if(current->fg && current->fg->bbox.intersection(r, t_boxe)){
                if (t_boxe<t){
                idx_back++; l[idx_back]=current->fg;

//                l.push_back(current->fg);
                }
            }
            if(current->fd && current->fd->bbox.intersection(r, t_boxe)){
//                if(current->fg && current->fg->bbox.intersection(r, t_boxe)){
                    if (t_boxe<t){
//                l.push_back(current->fd);
                        idx_back++; l[idx_back]=current->fd;

                    }
            }
            if(!current->fg){
                for (int i=current->i0; i<current->i1; i++ ){
                    
                    
                    int a = indices[i].vtxi ;
                    int b = indices[i].vtxj;
                    int c = indices[i].vtxk;
                    Triangle tri(vertices[a], vertices[b], vertices[c], albedo, is_mirror, is_transparent);
                    Vector localP, localN;
                    double alpha, beta, gamma;
                    double localt;
                    if (tri.intersection(r, localP, localN, localt,alpha, beta, gamma)) {
                        has_intersection = true;
                        if(localt<t){
                            t=localt;
                            P=localP;
                            //                                    N=localN;
                            N = normals[indices[i].ni] * alpha + normals[indices[i].nj]*beta + normals[indices[i].nk]*gamma;
                            N.normalize();
                            int textureId = indices[i].faceGroup;
                            int x = (uvs[indices[i].uvi][0] * alpha + uvs[indices[i].uvj][0]*beta + uvs[indices[i].uvk][0]*gamma) * (w[textureId]-1);
                            int y = (uvs[indices[i].uvi][1] * alpha + uvs[indices[i].uvj][1]*beta + uvs[indices[i].uvk][1]*gamma) * (h[textureId]-1);
                            
                            double cr = (textures[textureId][(y*w[textureId]+x)*3])/255.;
                            double cg = (textures[textureId][(y*w[textureId]+x)*3+1])/255.;
                            double cb = (textures[textureId][(y*w[textureId]+x)*3+2])/255.;
                            
                            color = Vector(cr,cg,cb);
                        }
                    }
                    
                }
            }
            
        }
        return has_intersection;
    };
    
    
    std::vector<TriangleIndices> indices;
    std::vector<Vector> vertices;
    std::vector<Vector> normals;
    std::vector<Vector> uvs; // Vector en 3D mais on n'utilise que 2 composantes
    std::vector<Vector> vertexcolors;
    
    
    private :
    BVH bvh;
    std::vector<std::vector<unsigned char> > textures;
    std::vector<int> w,h;
};




class Sphere : public Object{
    public :
    Sphere(const Vector &origin, double rayon, const Vector &coleur, bool mirror = false, bool transp = false) : O(origin), R(rayon){
        albedo = coleur;
        is_mirror = mirror;
        is_transparent = transp;
    };
    
    
    bool intersection (const Ray& r,  Vector& P, Vector& N, double &t, Vector &color) const {
      
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
        
        color = albedo;
        
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
    
    
    
    bool intersection (const Ray& r,  Vector& P, Vector& N, int& sphere_id, double& min_t, Vector &color) const{
        
        
        bool has_intersection = false;
        min_t = 1E99;
        
        for (int i=0; i<objects.size(); i++) {
            Vector localP, localN, localColor;
            double t;
            bool local_has_intersection = objects[i]->intersection(r, localP, localN, t, localColor);
            
            if (local_has_intersection){
                has_intersection = true;
                if (t<min_t){
                    min_t =t;
                    P= localP;
                    N=  localN;
                    sphere_id = i;
                    color = localColor;
                }
            }
        }
        return has_intersection;
    };
    
    
    Sphere *lumiere;
    double light_intensity;
    std::vector<Object*> objects;
    
    
    
};
