#include "FreeFormMotionProc.h"

int main(int argc, char *argv[])
{
    FreeFormMotionProc ffmp1,ffmp2,ffmp3;
    std::string res_file=argv[1];
    int nvertex=atoi(argv[2]),
        nfaces=atoi(argv[3]);

    double w1,w2,w3;
    //Example of merging 3 meshes
    w1=atof(argv[4]);
    std::cout<<"Reading file"<<std::endl;
    ffmp1.loadOBJ(argv[5],nvertex,nfaces);
    ffmp1.init();
    std::cout<<"Reading file"<<std::endl;
    w2=atof(argv[6]);
    ffmp2.loadOBJ(argv[7],nvertex,nfaces);
    ffmp2.init();
    std::cout<<"Blending"<<std::endl;
    ffmp1.blend(ffmp2,w1/(w1+w2));
    std::cout<<"Blending done"<<std::endl;
    w3=atof(argv[8]);
    ffmp3.loadOBJ(argv[9],nvertex,nfaces);
    ffmp3.init();
    ffmp1.blend(ffmp3,w1/(w1+w2+w3));

    std::cout<<"Solving system...";
    ffmp1.compute_vertex();
    std::cout<<"DONE"<<std::endl;

    std::cout<<"Writing file...";
    ffmp1.saveOBJ(res_file);
    std::cout<<"DONE"<<std::endl;
    //ffmp3.saveOBJ(std::string(argv[5])+"dumb.obj");
}
