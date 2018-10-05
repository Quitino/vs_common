#include <Eigen/Dense>
#include <iostream>
#include "vs_common.h"

void print(float *a, int n)
{
    for(int i=0;i<n;i++)
    {
        std::cout<<a[i]<<" ";
    }
    std::cout<<std::endl;
}

void test1()
{
    float e[3] = {0.1,-0.2,-0.32};
    float r[9];
    float q[4];
    float e_1[3];
    float e_2[3];
    float e_3[3];
    euler2rot(e,r);
    rot2euler(r,e_1);
    euler2quat(e,q);
    quat2euler(q,e_2);
    euler2quat(e,q);
    quat2rot(q,r);
    rot2euler(r,e_3);
    print(e_1,3);
    print(e_2,3);
    print(e_3,3);
}

void test2()
{
    float e[3] = {0.1,-0.2,0.3};
    float r[9];
    euler2rot(e,r);
    print(r,9);
    rot2euler(r,e);
    print(e,3);
}

void testAlias()
{
    std::vector<int> weights = {6,4,1,1};
    // std::vector<float> weights = {0.3,0.4,0.1,0.2};
    std::vector<float> probs;
    std::vector<int> alias;
    aliasTable(weights, probs, alias);
    for(size_t i=0; i<probs.size(); i++)
    {
        printf("%f %d\n", probs[i], alias[i]);
    }
}

int main()
{
    // test1();
    // test2();
    testAlias();
}
