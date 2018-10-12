#ifndef __VS_SINGLETON_H__
#define __VS_SINGLETON_H__

namespace vs
{

/** \brief Meyers Singleton: Release resource before exiting */
template <class T>
class Singleton
{
public:
    static T* instance()
    {
        static T instance;
        return &instance;
    }

    T operator ->() {return instance();}

    const T operator ->() const {return instance();}

private:
    Singleton() {}
    ~Singleton() {}
};

} /* namespace vs */
#endif//__VS_SINGLETON_H__