#include "vs_random.h"

namespace vs
{

static std::mt19937 mt;
static unsigned int seed = 0;

int randi(int high)
{
    return 0 < high ? rand_r(&seed)%high : 0;
}

int randi(int low, int high)
{
    return low < high ? low + rand_r(&seed)%(high-low) : 0;
}

double randf(double high)
{
    std::uniform_real_distribution<float> dis(0.0, high);
    return dis(mt);
}

double randf(double low, double high)
{
    std::uniform_real_distribution<float> dis(low, high);
    return dis(mt);
}

double randn(double mu, double sigma)
{
    std::normal_distribution<double> dis(mu, sigma);
    return dis(mt);
}

std::vector<int> randIntVec(int low, int high, int n)
{
    std::vector<int> res(n);
    for(int i=0; i<n; i++)
        res[i] = low < high ? low + rand_r(&seed)%(high-low) : 0;
    return res;
}

std::vector<float> randFloatVec(float low, float high, int n)
{
    std::uniform_real_distribution<float> dis(low, high);
    std::vector<float> res(n);
    for(int i=0; i<n; i++)
        res[i] = dis(mt);
    return res;
}

std::vector<double> randDoubleVec(double low, double high, int n)
{
    std::uniform_real_distribution<double> dis(low, high);
    std::vector<double> res(n);
    for(int i=0; i<n; i++)
        res[i] = dis(mt);
    return res;
}

std::vector<double> randnVec(double mu, double sigma, int n)
{
    std::normal_distribution<double> dis(mu, sigma);
    std::vector<double> res(n);
    for(int i=0; i<n; i++)
        res[i] = dis(mt);
    return res;
}

} /* namespace vs */