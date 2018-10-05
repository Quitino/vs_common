#ifndef __VS_RANDOM_H__
#define __VS_RANDOM_H__
#include <random>
#include "vs_vecutils.h"

/** \brief generate a random integer by unform distribution [0, high)*/
int randi(int high);

inline int randInt(int high) {return randi(high);}

/** \brief generate a random integer by unform distribution [low, high)*/
int randi(int low, int high);

inline int randInt(int low, int high) {return randi(low, high);}

/** \brief generate a random float number by unform distribution [0, high)*/
double randf(double high);

inline double randDouble(double high){return randf(high);}

/** \brief generate a random float number by unform distribution [low, high)*/
double randf(double low, double high);

inline double randDouble(double low, double high){return randf(low, high);}

std::vector<int> randIntVec(int low, int high, int n);

std::vector<float> randFloatVec(float low, float high, int n);

std::vector<double> randDoubleVec(double low, double high, int n);


template <typename T>
void aliasTable(const std::vector<T>& weights, std::vector<float>& probs, std::vector<int>& alias)
{
    int N = weights.size();
    float weight_sum = vecSum(weights);
    std::vector<float> norm_probs(N);  //prob * N
    for(int i=0; i<N; i++)
        norm_probs[i] = (float)weights[i] / weight_sum * (float)N;

    std::vector<int> small_block;
    small_block.reserve(N);
    std::vector<int> large_block;
    large_block.reserve(N);
    for (int k = N - 1; k >= 0; k--)
    {
        if (norm_probs[k] < 1)
            small_block.push_back(k);
        else
            large_block.push_back(k);
    }

    alias.resize(N);
    probs.resize(N);
    while(!small_block.empty() && !large_block.empty())
    {
        int cur_small = small_block.back();
        int cur_large = large_block.back();
        small_block.pop_back();
        large_block.pop_back();
        probs[cur_small] = norm_probs[cur_small];
        alias[cur_small] = cur_large;
        norm_probs[cur_large] = norm_probs[cur_large] + norm_probs[cur_small] - 1;
        if(norm_probs[cur_large] < 1)
            small_block.push_back(cur_large);
        else
            large_block.push_back(cur_large);
    }
    while(!large_block.empty())
    {
        int cur_large = large_block.back();
        large_block.pop_back();
        probs[cur_large] = 1;
    }
    while(!small_block.empty())
    {
        int cur_small = small_block.back();
        small_block.pop_back();
        probs[cur_small] = 1;
    }
}

template <typename T>
std::vector<int> weightedSample(const std::vector<T>& weights, int sample_cnt = 1, bool unique = true)
{
    std::vector<int> ids;
    ids.reserve(sample_cnt);
    int N = weights.size();
    if(unique)
    {
        if(N <= sample_cnt)
        {
            ids.resize(N);
            for(int i=0;i<N;i++) ids[i] = i;
        }
        else
        {
            //Vose's Alias Method
            std::vector<float> probs;
            std::vector<int> alias;
            aliasTable(weights, probs, alias);
            static std::mt19937 mt;
            static std::uniform_real_distribution<float> dis(0.0, 1.0);
            std::vector<int> select(N, false);
            int select_cnt = 0;
            while(select_cnt<sample_cnt)
            {
                int k = rand()%N;
                if(dis(mt)>probs[k]) k = alias[k];
                if(!select[k])
                {
                    select[k] = true;
                    select_cnt++;
                    ids.push_back(k);
                }
            }
            std::sort(ids.begin(), ids.end());
        }
    }
    else
    {
        //Vose's Alias Method
        std::vector<float> probs;
        std::vector<int> alias;
        aliasTable(weights, probs, alias);
        static std::mt19937 mt;
        static std::uniform_real_distribution<float> dis(0.0, 1.0);
        for(int i=0; i<sample_cnt; i++)
        {
            int k = rand()%N;
            if(dis(mt)>probs[k]) k = alias[k];
            ids.push_back(k);
        }
        std::sort(ids.begin(), ids.end());
    }
    return ids;
}


#endif//__VS_RANDOM_H__