#ifndef __VS_DATA_STRUCT_H__
#define __VS_DATA_STRUCT_H__
#include <vector>
#include <deque>

namespace vs
{

/** \brief Derived class of vector<vector<T>> for simple construction.
    Eg:vector<vector<int> > array2d = Array2D<int>(rows, cols);
*/
template <class T>
class Array2D : public std::vector<std::vector<T> >
{
public:
    Array2D(int rows = 0, int cols = 0, const T& val = T())
        : std::vector<std::vector<T> >(rows, std::vector<T>(cols, val)) {}
};

typedef Array2D<double> Array2Dd;

/** \brief Fixed length queue.
    Once data is out of queue size(max_len), delete the oldest data.
*/
template <class T>
class FixedQueue : public std::deque<T>
{
public:
    FixedQueue(unsigned int max_len = 1): std::deque<T>(), m_max_len(max_len) {}

    void push(const T& a)
    {
        this->push_back(a);
        if (this->size() > m_max_len) this->pop_front();
    }

private:
    unsigned int m_max_len;
};

/** \brief A median filter. Output the median value of the kth latest inputs.*/
template<class T>
class MedFilter
{
public:
    MedFilter(int k = 1): m_buffer(k) {}

    void input(const T & a) {m_buffer.push(a);}

    T output() {return findKth(m_buffer, m_buffer.size() / 2);}

private:
    FixedQueue<T> m_buffer;
};


/** \brief Circular queue*/
template<class T>
class CircularQueue
{
public:
    CircularQueue(int buf_len = 64)
    {
        setSize(buf_len);
    }

    void setSize(int buf_len)
    {
        m_len = buf_len;
        m_buffer.resize(buf_len);
    }

    int size() const {return m_len;}

    T& operator [] (int i)
    {
        return m_buffer[i % m_len];
    }

private:
    std::vector<T> m_buffer;
    int            m_len;
};

/** \brief Fast circular queue*/
template<class T>
class FastCircularQueue
{
public:
    FastCircularQueue(int buf_len = 64)
    {
        setSize(buf_len);
    }

    void setSize(int buf_len)
    {
        if(buf_len < 2 || (buf_len & (buf_len - 1)) != 0)
        {
            printf("[ERROR]CamCapture: invalid buffer size %d, use default %d"
                   "only support 2^n, eg: 4, 8, 16, 32, 64, 128...\n",
                   buf_len, m_len);
            return;
        }
        m_len = buf_len;
        m_mask = buf_len - 1;
        m_buffer.resize(buf_len);
    }

    int size() const {return m_len;}

    T& operator [] (int i)
    {
        return m_buffer[i & m_mask];
    }

private:
    std::vector<T> m_buffer;
    int            m_len;
    int            m_mask;
};

template<class T>
class MaxQueue
{
public:
    void push(const T& a)
    {
        m_buffer.push_back(a);
        while(!m_max_queue.empty() && m_max_queue.back() < a) m_max_queue.pop_back();
        m_max_queue.push_back(a);
    }

    T pop()
    {
        if(m_buffer.empty())
        {
            printf("[ERROR]MaxQueue: pop failed, empty queue.\n");
            return T(0);
        }
        T res = m_buffer.front();
        m_buffer.pop_front();
        if(res >= m_max_queue.front()) m_max_queue.pop_front();
        return res;
    }

    T max() const
    {
        if(m_buffer.empty() || m_max_queue.empty())
        {
            printf("[ERROR]MaxQueue: max failed, empty queue.\n");
            return T(0);
        }
        return m_max_queue.front();
    }

    T peek() const
    {
        if(m_buffer.empty())
        {
            printf("[ERROR]MaxQueue: pop failed, empty queue.\n");
            return T(0);
        }
        return m_buffer.front();
    }

    int size() const {return m_buffer.size();}

    const std::deque<T>& data() const {return m_buffer;}

private:
    std::deque<T> m_buffer;
    std::deque<T> m_max_queue;
};

} /* namespace vs */
#endif