#ifndef __VS_PF_H__
#define __VS_PF_H__
#include "vs_numeric.h"
#include "vs_vecutils.h"
#include "vs_random.h"
#include <functional>

namespace vs
{

template <class TypeE, class TypeW, int D>
class ParticleFilter
{
public:
    struct Particle
    {
        TypeE v[D];
        TypeW w;
    };
    typedef std::vector<Particle> ParticleList;
    typedef std::vector<TypeW> WeightList;

    ParticleFilter(): m_min_cnt(30), m_max_cnt(10000), m_kld_err(0.02), m_kld_z(0.99)
    {
        for(int i = 0; i < D; i++) m_kld_resolution[i] = 1.0;
    }

    virtual ~ParticleFilter(){}

    /** \brief clear particles*/
    void clear() {m_particles.clear();}

    /** \brief set particles*/
    void setParticles(const ParticleList& new_particles)
    {
        m_particles = new_particles;
    }

    /** \brief get particles*/
    const ParticleList& particles() const {return m_particles;}

    /** \brief get particles' weights*/
    WeightList weights() const
    {
        WeightList res;
        res.reserve(m_particles.size());
        for(const auto& p : m_particles)
            res.push_back(p.w);
        return res;
    }

    /** \brief particle count*/
    int count() const {return m_particles.size();}

    /** \brief any particle has no-zero weight*/
    bool weightAny() const
    {
        for(const auto& p : m_particles) if(p.w > 0) return true;
        return false;
    }

    /** \brief normalize that all weights sum to 1 */
    void weightNormalize()
    {
        if(m_particles.empty()) return;
        TypeW sum = 0;
        for(const auto& p : m_particles) sum += p.w;
        if(sum <= 1e-5)
        {
            double w = 1.0 / m_particles.size();
            for(auto& p : m_particles) p.w = w;
        }
        else
        {
            for(auto& p : m_particles) p.w /= sum;
        }
    }

    /** \brief universal propagate api*/
    void propagate(std::function<void(Particle&)> foo)
    {
        for(auto& p : m_particles) foo(p);
    }

    /** \brief set min max sample count */
    void setMinMaxCnt(double min_cnt, double max_cnt)
    {
        m_min_cnt = min_cnt;
        m_max_cnt = max_cnt;
    }

    /** \brief resample fix count of particles with low-variance resampling */
    void resample(int cnt = -1)
    {
        if(m_particles.empty()) return;
        if(cnt <= 0) return resampleKLD();

        auto ids = weightedSample(weights(), cnt, 2);
        ParticleList new_particles;
        new_particles.reserve(ids.size());
        for(auto i : ids) new_particles.push_back(m_particles[i]);
        m_particles = new_particles;
    }

    /** \brief set KLD param*/
    void setKLD(double kld_err, double kld_z)
    {
        m_kld_err = kld_err;
        m_kld_z = kld_z;
    }

    /** \brief set KLD resolution*/
    void setKLDResolution(TypeE res[D])
    {
        for(int i = 0; i < D; i++) m_kld_resolution[i] = res[i];
    }

    /** \brief compute particle data range */
    void range(TypeE range_min[D], TypeE range_max[D]) const
    {
        if(m_particles.empty()) return;
        const auto& p0 = m_particles[0];
        for(int i = 0; i < D; i++)
        {
            range_min[i] = range_max[i] = p0.v[i];
        }
        for(const auto& p : m_particles)
        {
            for(int i = 0; i < D; i++)
            {
                if(p.v[i] < range_min[i]) range_min[i] = p.v[i];
                else if(p.v[i] > range_max[i]) range_max[i] = p.v[i];
            }
        }
    }

    /** \brief adaptive resample with KLD*/
    void resampleKLD()
    {
        if(m_particles.empty()) return;
        // build grid for particle range
        TypeE range_min[D];
        TypeE range_max[D];
        range(range_min, range_max);
        int size[D];
        for(int i = 0; i < D; i++)
        {
            size[i] = (range_max[i] - range_min[i]) / m_kld_resolution[i] + 1.0;
        }
        int stride[D];
        stride[D - 1] = 1;
        for(int i = D - 1; i > 0; i--)
        {
            stride[i - 1] = stride[i] * size[i];
        }
        int n = stride[0] * size[0];
        if(n > 1e8) return resample(m_max_cnt);
        std::vector<bool> bin(n, false);

        // build Vose's Alias with weights
        std::vector<float> probs;
        std::vector<int> alias;
        aliasTable(weights(), probs, alias);

        // random sample
        static std::mt19937 mt;
        static std::uniform_real_distribution<float> dis(0.0, 1.0);
        ParticleList new_particles;
        new_particles.reserve(m_particles.size());
        int bin_cnt = 0;
        while(new_particles.size() < m_max_cnt)
        {
            int k = randi(probs.size());
            if(dis(mt) > probs[k]) k = alias[k];
            auto p = m_particles[k];
            p.w = 1.0;
            new_particles.push_back(p);

            int bin_idx = 0;
            for(int i = 0; i < D; i++)
            {
                bin_idx += (int)((p.v[i] - range_min[i]) / m_kld_resolution[i])
                            * stride[i];
            }
            if(!bin[bin_idx])
            {
                bin[bin_idx] = true;
                bin_cnt++;
            }

            if(new_particles.size() > kldNum(bin_cnt)) break;
        }
        m_particles = new_particles;
    }

    void updateWeights(const WeightList& weights)
    {
        if(weights.empty()) return;
        if(weights.size() != m_particles.size())
        {
            printf("[ERROR]updateWeights: size not match(%d, %d).\n",
                    (int)weights.size(), (int)m_particles.size());
            return;
        }
        // update particle weight
        if(vecAny(weights))
        {
            for(size_t i = 0; i < weights.size(); i++)
            {
                m_particles[i].w *= weights[i];
            }
        }
    }

    /** \brief add particle with uniform or gaussian distribution
        \param[in] p1
        \param[in] p2
        \param[in] dis_uni 0: gaussian distribution: mu = p1, sigma = p2, N(mu,sigma^2)
                           1: uniform distribution: min = p1, max = p2, U[min,max]
        \param[in] add_cnt add particle count
        \param[in] weight particle weight
    */
    void addParticles(const TypeE p1[D], const TypeE p2[D], int dis_uni = 1,
                     int add_cnt = -1, double weight = 1)
    {
        if(dis_uni) addParticleUniform(p1, p2, add_cnt, weight);
        else addParticleGaussian(p1, p2, add_cnt, weight);
    }

protected:
    ParticleList m_particles;
    TypeE m_kld_resolution[D];
    int m_min_cnt, m_max_cnt;
    double m_kld_err, m_kld_z;

    int kldNum(int k)
    {
        if(k <= 1) m_min_cnt;
        double b = 2.0 / (9 * ((double) k - 1));
        double x = 1 - b + sqrt(b) * m_kld_z;
        int n = (int)ceil(((double)k - 1) / (2.0 * m_kld_err) * x * x * x);
        return clip(n, m_min_cnt, m_max_cnt);
    }

    // add_cnt default -1 means calc adaptive count from [pmin, pmax]
    void addParticleUniform(const TypeE p1[D], const TypeE p2[D],
                            int add_cnt = -1, double weight = 1)
    {
        if(add_cnt <= 0)
        {
            double prod = 100.0;
            for(int i = 0; i < D; i++)
            {
                prod *= p2[i] - p1[i];
            }
            add_cnt = refineAddCnt(fabs(prod));
        }
        std::vector<std::uniform_real_distribution<TypeE> > dis;
        dis.reserve(D);
        for(int i = 0; i < D; i++)
        {
            dis.push_back(std::uniform_real_distribution<TypeE>(p1[i], p2[i]));
        }
        m_particles.reserve(add_cnt + m_particles.size());
        static std::mt19937 mt;
        while(add_cnt-- > 0)
        {
            Particle p;
            for(int i = 0; i < D; i++)
            {
                p.v[i] = dis[i](mt);
            }
            p.w = weight;
            m_particles.push_back(p);
        }
    }

    // add_cnt default -1 means calc adaptive count from [pmin, pmax]
    void addParticleGaussian(const TypeE p1[D], const TypeE p2[D],
                             int add_cnt = -1, double weight = 1)
    {
        if(add_cnt <= 0)
        {
            double prod = 100.0;
            for(int i = 0; i < D; i++)
            {
                prod *= p2[i];
            }
            add_cnt = refineAddCnt(fabs(prod));
        }
        std::vector<std::normal_distribution<TypeE> > dis;
        for(int i = 0; i < D; i++)
        {
            dis.push_back(std::normal_distribution<TypeE>(p1[i], p2[i]));
        }
        m_particles.reserve(add_cnt + m_particles.size());
        static std::mt19937 mt;
        while(add_cnt-- > 0)
        {
            Particle p;
            for(int i = 0; i < D; i++)
            {
                p.v[i] = dis[i](mt);
            }
            p.w = weight;
            m_particles.push_back(p);
        }
    }

    int refineAddCnt(int add_cnt)
    {
        int all_cnt = clip(add_cnt + count(), m_min_cnt, m_max_cnt);
        add_cnt = all_cnt - count();
        if(add_cnt <= 0)
        {
            printf("[WARN]addParticleGaussian: not add, over max cnt.\n");
            return 0;
        }
        return add_cnt;
    }
};

} /* namespace vs */
#endif//__VS_PF_H__