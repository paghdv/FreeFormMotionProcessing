#include "FreeFormMotionProc.h"
#include <stdio.h>
#include <iostream>
#include <stdio.h>
#include <fstream>

#include <eigen3/Eigen/SVD>
#include <eigen3/Eigen/LU>
#include "utils_q.h"

using namespace Eigen;
using Eigen::MatrixXd;

FreeFormMotionProc::FreeFormMotionProc()
{

}
void FreeFormMotionProc::compute_q_S()
{
    q.resize(nfaces);
    S.resize(nfaces);
    #pragma omp parallel for
    for (int i=0;i<D.cols();i+=3)
    {
        polar_decomposition_svd(D.block<3,3>(0,i),q[i/3],S[i/3]);
    }
}
void FreeFormMotionProc::compute_D()
{
    Eigen::Vector3d v_1,v_2;
    F=(G*X).transpose();
    D.resize(3,3*nfaces);
    const size_t sF=F.cols();
    int j=0;
    for (size_t i=0;i<sF;i+=2)
    {
        v_1[0] = F.coeffRef(0,i  );
        v_1[1] = F.coeffRef(1,i  );
        v_1[2] = F.coeffRef(2,i  );

        v_2[0] = F.coeffRef(0,i+1);
        v_2[1] = F.coeffRef(1,i+1);
        v_2[2] = F.coeffRef(2,i+1);

        D.coeffRef(0,j)   = v_1[0];
        D.coeffRef(1,j)   = v_1[1];
        D.coeffRef(2,j)   = v_1[2];
        j++;

        D.coeffRef(0,j) = v_2[0];
        D.coeffRef(1,j) = v_2[1];
        D.coeffRef(2,j) = v_2[2];
        j++;

        v_1=v_1.cross(v_2);
        v_1.normalize();
        D.coeffRef(0,j) = v_1[0];
        D.coeffRef(1,j) = v_1[1];
        D.coeffRef(2,j) = v_1[2];
        j++;
    }
}
void FreeFormMotionProc::build_G()
{
    std::vector<Eigen::Triplet<double> > triplets(nfaces*4);

    G.resize(2*nfaces,nvertex);
    int j=0;
    int k=0;
    for (int i=0;i<nfaces;i++)
    {
        //First vector
        triplets[k] = Eigen::Triplet<double>(j, T.coeff(i,0), 1);
        k++;
        triplets[k] = Eigen::Triplet<double>(j, T.coeff(i,1),-1);
        k++;
        j++;
        //Second vector
        triplets[k] = Eigen::Triplet<double>(j, T.coeff(i,2), 1);
        k++;
        triplets[k] = Eigen::Triplet<double>(j, T.coeff(i,1),-1);
        k++;
        j++;
    }
    G.setFromTriplets(triplets.begin() , triplets.end());
}
void FreeFormMotionProc::polar_decomposition(const Eigen::Matrix3d &Di,
                                                   Quaterniond &qRi,
                                                   Eigen::Matrix2d &Si)
{
    //D=QS
    //(Q^-1)D=S
    Eigen::Matrix3d Q0,Q1=Di,Q0inv,R,S1;
    double sn;
    int it=0;
    do
    {
        it++;
        Q0=Q1;
        Q0inv=Q0.inverse();
        Q1=0.5*(Q0 + Eigen::Transpose<Eigen::Matrix3d>(Q0inv));
        sn=(Q0-Q1).squaredNorm();
    }while( sn >0 && MAXPOLIT>it);
    R=Q1;
    S1=Q1.inverse()*Di;
    R=R*S1(2,2);
    S1=S1/S1(2,2);
    qRi=R;
    Si=S1.block<2,2>(0,0);
    qRi.normalize();
    //std::cout<<it<<std::endl;
}

void FreeFormMotionProc::polar_decomposition_svd(const Eigen::Matrix3d &Di,
                                                       Quaterniond &qRi,
                                                       Eigen::Matrix2d &Si)
{

    Eigen::Matrix3d W,V,R,VT,S1;
    W.setZero();
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(Di, Eigen::ComputeFullU | Eigen::ComputeFullV);//, Eigen::ComputeThinU | Eigen::ComputeThinV
    W(0,0)=svd.singularValues()[0];
    W(1,1)=svd.singularValues()[1];
    W(2,2)=svd.singularValues()[2];

    V = svd.matrixV();
    VT=Eigen::Transpose<Eigen::Matrix3d>(V);

    S1 = svd.matrixV()*W*VT;
    R= svd.matrixU()*VT;

    qRi=R;
    Si=S1.block<2,2>(0,0);

}

void FreeFormMotionProc::re_orient_quaternions()
{

}

void FreeFormMotionProc::init()
{
    std::cout<<"Building G"<<std::endl;
    build_G();
    std::cout<<"Building D"<<std::endl;
    compute_D();
    //compute q and S
    std::cout<<"Building q and S"<<std::endl;
    compute_q_S();

}
void FreeFormMotionProc::blend(const FreeFormMotionProc &otherFFMP,
                               const double w)
{
    Eigen::Matrix3d tempS;
    Eigen::Quaterniond qtemp;
    Eigen::Matrix2d Stemp;
    tempS.setZero();
    tempS(2,2)=1.0;
    int j=0;
    for (int i=0;i<q.size();i++)
    {
        //Check quaternions
        if (q[i].dot(otherFFMP.get_q(i))<0)
            qscale(q[i],-1);
        //Minus
        qtemp = otherFFMP.get_q(i)*(q[i].inverse());
        Stemp = otherFFMP.get_S(i)-S[i];
        //Plus
        qtemp = qpow(qtemp,w)*q[i];
        Stemp = w*Stemp+S[i];
        tempS.block<2,2>(0,0) = Stemp;
        //Recompute D (and F)
        D.block<3,3>(0,i*3) = qtemp.toRotationMatrix()*tempS;
        F.block<3,2>(0,j*2) = D.block<3,2>(0,i*3);
        j++;
    }
}

void FreeFormMotionProc::dumb_blend(const FreeFormMotionProc &otherFFMP,
                                    const double w,
                                          FreeFormMotionProc &otherFFMP2)
{
    otherFFMP2.X.resize(nvertex,3);
    otherFFMP2.T.resize(nfaces,3);
    otherFFMP2.X=otherFFMP.X*w+(X*(1.0-w));
    otherFFMP2.T=T;
    otherFFMP2.nvertex=nvertex;
    otherFFMP2.nfaces=nfaces;
}

bool FreeFormMotionProc::loadOBJ(const std::string &filename,const int num_vertex,const int num_faces)
{
    nvertex=num_vertex;
    nfaces =num_faces;

    //Open file
    std::fstream input(filename.c_str(),std::fstream::in);
    if (!input.is_open())
        return false;
    //
    std::string dummystr;
    //
    char dummy;
    X.resize(num_vertex,3);
    T.resize(num_faces,3);

    for (int i=0;i<num_vertex;i++)
    {
        input>>dummy;
        while (dummy!='v')
        {
            getline(input,dummystr);
            input>>dummy;
        }
        //Write vertex
        input>>X.coeffRef(i,0);
        input>>X.coeffRef(i,1);
        input>>X.coeffRef(i,2);
    }

    if (num_faces>0)
    {
        //Read faces
        for (int i=0;i<num_faces;i++)
        {
            input>>dummy;
            while (dummy!='f')
            {
                getline(input,dummystr);
                input>>dummy;
            }
            //Write face
            input>>T.coeffRef(i,0);
            input>>T.coeffRef(i,1);
            input>>T.coeffRef(i,2);
            //Indices are 1 based in obj
            T.coeffRef(i,0)--;
            T.coeffRef(i,1)--;
            T.coeffRef(i,2)--;
        }
    }
    input.close();
    return true;
}

void FreeFormMotionProc::get_X(const int i,float &x,float &y,float &z)
{
    x=(float)X.coeff(i,0);
    y=(float)X.coeff(i,1);
    z=(float)X.coeff(i,2);
}

bool FreeFormMotionProc::saveOBJ(const std::string &filename) const
{
    //Open file
    std::fstream output(filename.c_str(),std::fstream::out);
    if (!output.is_open())
    {
        std::cout<<"Could not write obj file to: "<<filename<<std::endl;
        return false;
    }
    //
    for (int i=0;i<nvertex;i++)
        output<<"v "<<X(i,0)<<" "<<X(i,1)<<" "<<X(i,2)<<std::endl;
    //
    for (int i=0;i<nfaces;i++)
        output<<"f "<<T(i,0)+1<<" "<<T(i,1)+1<<" "<<T(i,2)+1<<std::endl;
    //
    output.close();
    return true;
}

void FreeFormMotionProc::compute_vertex(const int axis)
{
    Eigen::SparseMatrix<double, Eigen::ColMajor> GTG,GTGT;
    Eigen::VectorXd GTFTx(nvertex+1+constraints[axis].size(),1);

    GTG = G.transpose();
    //F.transpose();
    GTFTx.block(0,0,nvertex,1) = GTG* F.block(axis,0,1,nfaces*2).transpose();
    GTG  = GTG*G;

    //Add constraint to GTG
    GTG.conservativeResize(GTG.rows()+1+constraints[axis].size(),GTG.cols());

    GTG.coeffRef(nvertex,0)=DISTWEIGHT*10;
    GTFTx(nvertex,0)=X(0,axis)*DISTWEIGHT*10;
    for (size_t i=0;i<constraints[axis].size();i++)
    {
        GTG.coeffRef(nvertex+i+1, constraints[axis][i].row()) =  DISTWEIGHT;
        GTG.coeffRef(nvertex+i+1, constraints[axis][i].col()) = -DISTWEIGHT;
        GTFTx(nvertex+i+1,0) = constraints[axis][i].value()*DISTWEIGHT;
    }

    //Make the matrix square
    GTGT = GTG.transpose();

    GTFTx = GTGT*GTFTx;
    GTG   = GTGT*GTG;

    // Solve the system
    GTG.makeCompressed();
    Eigen::SparseLU<Eigen::SparseMatrix<double, Eigen::ColMajor>, Eigen::COLAMDOrdering<int> > solver;
    solver.analyzePattern(GTG);
    solver.factorize(GTG);

    //Eigen::VectorXd x(nvertex);
    X.block(0,axis,nvertex,1)=solver.solve(GTFTx);
}

void FreeFormMotionProc::compute_vertex()
{
    Eigen::SparseMatrix<double, Eigen::ColMajor> GTG,GTGT;
    Eigen::VectorXd GTFTx(nvertex+1,1),
                    GTFTy(nvertex+1,1),
                    GTFTz(nvertex+1,1);

    GTG = G.transpose();
    //F.transpose();
    GTFTx.block(0,0,nvertex,1) = GTG* F.block(0,0,1,nfaces*2).transpose();
    GTFTy.block(0,0,nvertex,1) = GTG* F.block(1,0,1,nfaces*2).transpose();
    GTFTz.block(0,0,nvertex,1) = GTG* F.block(2,0,1,nfaces*2).transpose();
    GTG  = GTG*G;

    //Add constraint to GTG
    GTG.conservativeResize(GTG.rows()+1,GTG.cols());

    GTG.coeffRef(GTG.rows()-1,0)=1;
    GTFTx(GTFTx.rows()-1,0)=X(0,0);
    GTFTy(GTFTx.rows()-1,0)=X(0,1);
    GTFTz(GTFTx.rows()-1,0)=X(0,2);

    //Make the matrix square
    GTGT=GTG.transpose();

    GTFTx = GTGT*GTFTx;
    GTFTy = GTGT*GTFTy;
    GTFTz = GTGT*GTFTz;
    GTG   = GTGT*GTG;

    // Solve the system
    GTG.makeCompressed();
    Eigen::SparseLU<Eigen::SparseMatrix<double, Eigen::ColMajor>, Eigen::COLAMDOrdering<int> > solver;
    solver.analyzePattern(GTG);
    solver.factorize(GTG);

    //Eigen::VectorXd x(nvertex);
    X.block(0,0,nvertex,1)=solver.solve(GTFTx);
    X.block(0,1,nvertex,1)=solver.solve(GTFTy);
    X.block(0,2,nvertex,1)=solver.solve(GTFTz);
}

void FreeFormMotionProc::add_sqr_distance_constraint(const unsigned int axis, const unsigned int ind1,const unsigned int ind2,const double dist)
{
    constraints[axis].push_back(Eigen::Triplet<double>(ind1,ind2,dist));
}
