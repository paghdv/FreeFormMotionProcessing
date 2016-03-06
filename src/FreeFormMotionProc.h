#ifndef FREE_FORM_MOTION_PROC_H
#define FREE_FORM_MOTION_PROC_H

#include <iostream>
#include <eigen3/Eigen/Sparse>
#include <eigen3/Eigen/Dense>

//PROCESS OUTLINE
//1. Read mesh with vertices X and connectivity M
//2. create matrix G
//3. Create matrix D
//4. Blend two meshes
//5.1  Decompose every D into q and S
//5.2  Perform blend operation (Re-orienting q's coherently)
//6.1 Create system and add constraints
//6.2 Reconstruct X

class FreeFormMotionProc
{

    private:
        //GX=F^T
        //F_o=[u_o v_o]=[(xj-xi) (xk-xi)]
        //D_o=[u_o v_o n_o]
        const int MAXPOLIT=100;
        const double DISTWEIGHT=10000;
        Eigen::SparseMatrix<double, Eigen::ColMajor> G;
        std::vector<Eigen::Quaterniond> q;
        std::vector<Eigen::Matrix2d> S;
        Eigen::Matrix3Xd F;
        Eigen::MatrixX3d X; //Vertex coordinates
        Eigen::Matrix3Xd D; //Vertex coordinates
        Eigen::MatrixX3i T; //Triangles
        std::vector<Eigen::Triplet<double> > constraints[3];

        int nvertex, nfaces;

        void compute_q_S();
        void compute_D();
        void build_G();
        void polar_decomposition(const Eigen::Matrix3d &Di, Eigen::Quaterniond &qRi, Eigen::Matrix2d &Si);
        void polar_decomposition_svd(const Eigen::Matrix3d &Di, Eigen::Quaterniond &qRi, Eigen::Matrix2d &Si);
        //Compute dot prod and if <0 reorient (*-1)
        void re_orient_quaternions();
    public:
        FreeFormMotionProc();
        void get_X(const int i, float &x, float &y, float &z);
        void add_sqr_distance_constraint(const unsigned int axis, const unsigned int ind1,
                                         const unsigned int ind2, const double dist);
        void init();
        void set_faces(const Eigen::MatrixX3i T_){T=T_; nfaces=T.rows();}
        void blend(const FreeFormMotionProc &otherFFMP, const double w);
        void dumb_blend(const FreeFormMotionProc &otherFFMP, const double w,FreeFormMotionProc &otherFFMP2);
        Eigen::Quaterniond get_q(const int i)const{return q[i];}
        Eigen::Matrix2d get_S(const int i)const{return S[i];}
        bool loadOBJ(const std::string &filename, const int num_vertex, const int num_faces);
        bool saveOBJ(const std::string &filename) const;
        void compute_vertex();
        void compute_vertex(const int axis);
        //Solve (G^T)GX=(G^T)(F^T)

        ~FreeFormMotionProc(){}

};

#endif

