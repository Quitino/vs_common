#include <vs_common.h>

cv::Mat mapCvt(const cv::Mat &map, int mid[2])
{
    cv::Mat res;
    if(mid[0] > 0)
        cv::vconcat(map.rowRange(mid[0], map.rows), map.rowRange(0, mid[0]), res);
    else
        res = map;
    cv::Mat res2;
    if(mid[1] > 0)
        cv::hconcat(res.colRange(mid[1], res.cols), res.colRange(0, mid[1]), res2);
    else
        res2 = res;
    return res2;
}

void testGridMapping2D()
{
    std::ifstream fin("../data/intelab.map");
    int E = 8;
    int M = (1 << E) - 1;
    vs::GridMapping2D gmapping(E, 0.05f, 255, 128, 20, 100);
    float pos[2];
    float pts[50000][2] = {0};
    int N;
    while(!fin.eof())
    {
        fin >> pos[0] >> pos[1] >> N;
        for(int i = 0; i < N; i++)
        {
            fin >> pts[i][0] >> pts[i][1];
        }
        gmapping.update(pos, pts, N);
        auto map = gmapping.map();
        auto origin_idx = gmapping.originIdx();

        cv::Mat raw_map(1<<E, 1<<E, CV_8UC1, map);
        int mid[2] = {(origin_idx[0] + (1<<(E-1))) & M,
                      (origin_idx[1] + (1<<(E-1))) & M};
        cv::Mat center_map = mapCvt(raw_map, mid);

        cv::cvtColor(raw_map, raw_map, cv::COLOR_GRAY2BGR);
        cv::cvtColor(center_map, center_map, cv::COLOR_GRAY2BGR);

        cv::circle(raw_map, cv::Point2i(origin_idx[1] & M, origin_idx[0] & M), 2,
                    cv::Scalar(0,0,255), 2);
        cv::line(raw_map, cv::Point2i(mid[1], 0), cv::Point2i(mid[1], raw_map.rows),
                    cv::Scalar(128,128,0), 1);
        cv::line(raw_map, cv::Point2i(0, mid[0]), cv::Point2i(raw_map.cols, mid[0]),
                    cv::Scalar(128,128,0), 1);
        cv::circle(center_map, cv::Point2i((1<<(E-1)), (1<<(E-1))), 2,
                    cv::Scalar(0,0,255), 2);
        cv::imshow("raw_map", raw_map);
        cv::imshow("center_map", center_map);
        static bool halt = true;
        char key = cv::waitKey(halt ? 0 : 50);
        if(key == 27) break;
        else if(key == 's') halt = !halt;
    }
}

void testGridMapping3D()
{
    vs::Viz3D g_viz;
    int E = 8;
    int M = (1 << E) - 1;
    vs::GridMapping3D gmapping(E, 0.5f, 128, 128, 1, 100);
    // create point cloud
    float pts[500000][3] = {0};
    int N = 0;
    auto append = [&pts, &N](float x, float y, float z)
    {
        if(N>=500000) return;
        pts[N][0] = x;
        pts[N][1] = y;
        pts[N++][2] = z;
    };
    for(float x = -3; x < 3; x+=0.5)
        for(float y = -5; y < 5; y += 0.8)
            append(x, y, 12);

    for(float x = -10, y = -5; x < -3; x += 0.1, y += 0.2)
        for(float z = 5; z < 15; z += 0.5)
            append(x, y, z);

    float origin[2] = {0,0};
    gmapping.update(origin, pts, N);
    // show cloud
    g_viz.updateWidget("cood", cv::viz::WCoordinateSystem(5));
    std::vector<cv::Point3f> cloud;
    for(int i=0; i<N; i++)
    {
        cloud.push_back(cv::Point3f(pts[i][0],pts[i][1],pts[i][2]));
    }
    if(!cloud.empty())
        g_viz.updateWidget("cloud", cv::viz::WCloud(cloud, cv::viz::Color::green()));
    
    // show grid
    int cnt = 0;
    float hr = gmapping.resolution() / 2;
    auto map = gmapping.map();
    for(int i = 0; i < (1 << (E + E + E)); i++)
    {
        if(gmapping.occupyVal(map[i]))
        {

            int idx[3] = {(i >> (E + E)) & M, (i >> E) & M, i & M};
            float pos[3];
            gmapping.idx2pos(idx, pos);
            cv::Point3f p_min(pos[0] - hr, pos[1] - hr, pos[2] - hr);
            cv::Point3f p_max(pos[0] + hr, pos[1] + hr, pos[2] + hr);
            char name[128] = {0};
            snprintf(name, 128, "cube%d", cnt++);
            g_viz.updateWidget(name, cv::viz::WCube(p_min, p_max, false));
        }
    }
    printf("cnt:%d N:%d\n", cnt, N);
    printf("按任意键退出\n");
    getchar();
}

int main(int argc, char** argv)
{
    if(argc <= 1)
    {
        testGridMapping2D();
    }
    else
    {
        int k = atoi(argv[1]);
        if(k == 3) testGridMapping3D();
        else testGridMapping2D();
    }
}