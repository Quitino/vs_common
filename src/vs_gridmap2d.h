#ifndef __VS_GRIDMAP_H__
#define __VS_GRIDMAP_H__
#include <vector>
#include <iostream>

namespace vs
{

/**
    \brief grid map buffer
    x - horizontal direction  y - verticle direction
    the coodinate of data(i, j) :  x = (i+0.5)*res+origin_x   y = (j+0.5)*res+origin_y
  */
template<typename T>
struct GridMap2D
{
    GridMap2D(): rows(0), cols(0), origin_x(0), origin_y(0), res(0) {}

    GridMap2D(int _rows, int _cols, double ox = 0, double oy = 0,
              double res = 0.05, T val = 0)
        : data(std::vector<std::vector<T> >(_rows, std::vector<T>(_cols, val)))
        , rows(_rows), cols(_cols), origin_x(ox), origin_y(oy), res(res)
    {}

    GridMap2D(double xmax, double xmin, double ymax, double ymin,
              double res = 0.05, T val = 0)
        : rows((ymax - ymin) / res + 1) , cols((xmax - xmin) / res + 1)
        , origin_x(xmin) , origin_y(ymin) , res(res)
    {
        if(rows * cols > 1e9)
            printf("[ERROR]GridMap2D: map size(%dx%d) out of memory.\n", cols, rows);
        else
            resize(rows, cols);
    }

    int ridx(double y) const {return (y - origin_y) / res;}

    int cidx(double x) const {return (x - origin_x) / res;}

    /** \brief Is a point (x, y) in this map */
    bool inMap(double x, double y) const {return inMap(ridx(y), cidx(x));}

    /** \brief Is a indexed grid (cidx, ridx) in this map */
    bool inMap(int r, int c) const {return c >= 0 && r >= 0 && c < cols && r < rows;}

    /** \brief Get the value in grid which point (x, y) located into. */
    const T& at(double x, double y) const {return data[ridx(y)][cidx(x)];}
    T& at(double x, double y) {return data[ridx(y)][cidx(x)];}

    /** \brief Get the value in grid (ridx, cidx) */
    const T& getValue(int r, int c) const {return data[r][c];}
    T& getValue(int r, int c) {return data[r][c];}

    /** \brief Convert grid idx to point cordinate.
        Use the center of the specific grid.
        x = (c+0.5)*res+origin_x   y = (r+0.5)*res+origin_y
    */
    void idx2xy(int c, int r, double& x, double &y) const
    {
        x = origin_x + (c + 0.5) * res;
        y = origin_y + (r + 0.5) * res;
    }

    /** \brief Convert grid idx which point located.
        c = (int)((x-origin_x)/res) r = (int)((y-origin_y)/res)
    */
    void xy2idx(double x, double y, int &c, int &r) const
    {
        c = cidx(x);
        r = ridx(y);
    }

    /** \brief Resize the grid map */
    void resize(int r, int c)
    {
        rows = r;
        cols = c;
        data = std::vector<std::vector<T> >(r, std::vector<T>(c, 0));
    }

    /** \brief Set all grid value to \val */
    void setTo(const T& val)
    {
        for(int i = 0; i < rows; i++)
            for(int j = 0; j < cols; j++)
                data[i][j] = val;
    }

    /** \brief Set the specific grid[r][c] value to \val */
    void setTo(int r, int c, const T& val)
    {
        if(inMap(r, c)) data[r][c] = val;
    }

    /** \brief All grid value in map add \a */
    GridMap2D operator + (const T& a)
    {
        GridMap2D m(*this);
        for(int i = 0;i < m.rows; i++)
            for(int j = 0;j < m.cols; j++)
                m.data[i][j] += a;
        return m;
    }

    /** \brief All grid value in map add \a */
    GridMap2D &operator += (const T& a)
    {
        for(int i = 0; i < rows; i++)
            for(int j = 0; j < cols; j++)
                data[i][j] += a;
        return (*this);
    }

    /** \brief All grid value in map multi \a */
    GridMap2D operator* (const T& a)
    {
        GridMap2D m(*this);
        for(int i = 0; i < m.rows; i++)
            for(int j = 0; j < m.cols; j++)
                m.data[i][j] *= a;
        return m;
    }

    /** \brief All grid value in map multi \a */
    GridMap2D &operator *= (const T& a)
    {
        for(int i = 0; i<rows; i++)
            for(int j = 0; j < cols; j++)
                data[i][j] *= a;
            return (*this);
    }

    std::vector<std::vector <T> > data; //-1 unknown; 0~1:prob of occupied
    int rows, cols;
    double origin_x, origin_y;  //the (x, y) coodinate of data(0, 0)
    double res; //the length of each grid edge
};

} /* namespace vs */
#endif//__VS_GRIDMAP_H__