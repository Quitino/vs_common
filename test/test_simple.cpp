#include <Eigen/Dense>
#include <iostream>
#include <vs_common.h>
#include <cmath>

#define PrintHeader() do{printf("============= %s ============\n", __func__);} while(0)

using namespace vs;

void print(float *a, int n)
{
    for(int i = 0; i < n; i++)
        std::cout << a[i] << " ";
    std::cout << std::endl;
}

void test1()
{
    PrintHeader();
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
    PrintHeader();
    float e[3] = {0.1,-0.2,0.3};
    float r[9];
    euler2rot(e,r);
    print(r,9);
    rot2euler(r,e);
    print(e,3);
}

void testAlias()
{
    PrintHeader();
    std::vector<int> weights = {1, 2, 3, 4};
    // std::vector<float> weights = {0.3,0.4,0.1,0.2};
    std::vector<float> probs;
    std::vector<int> alias;
    aliasTable(weights, probs, alias);
    for(size_t i=0; i<probs.size(); i++)
    {
        printf("%f %d\n", probs[i], alias[i]);
    }
}

void testShuffle()
{
    std::vector<int> a;
    for(int i = 0; i < 100; i++) a.push_back(i);
    shuffleInPlace(a);
    for(auto i : a) printf("%d ", i);
    printf("\n");
}

void testWeightedSample()
{
    std::vector<float> weights = {1, 2, 3, 4, 5, 6};
    for(int method = 0; method < 3; method++)
    {
        for(int k = 0; k < 6; k++)
        {
            auto res = weightedSample(weights, k, method);
            printf("sample %d with method %d:", k, method);
            for(auto i : res) printf("%d ", i);
            printf("\n");
        }
    }
}

void testRaycast2D()
{
    PrintHeader();
    int start[] = {0, -3};
    int end[] = {-5, -1};

    auto foo = [](int x, int y) {printf("(%d %d)\n", x, y);return true;};
    raycast2D(start, end, foo);
}

void testRaycast3D()
{
    PrintHeader();
    int start[] = {-7, 0, -3};
    int end[] = {2, -5, -1};
    std::vector<std::vector<int> > out;
    auto foo = [](int x, int y, int z) {printf("(%d %d %d)\n", x, y, z);return true;};
    raycast3D(start, end, foo);
    printf("========================\n");
    raycast3D(end, start, foo);
}

void testStereoRectifier()
{
    PrintHeader();
    StereoRectifier re;
    if(!re.init("../data/calib_fisheye.yaml"))
    {
        printf("[ERROR]StereoRectifier:Failed init.\n");
        return;
    }
    std::cout<<"Calib raw:"<<re.getCalibRaw()<<std::endl<<std::endl;
    std::cout<<"Calib rectify:"<<re.getCalibRectify()<<std::endl<<std::endl;
}

void testDataSaver()
{
    PrintHeader();
    struct MyData
    {
        float x, y, z;
        int d;
    };

    DataSaver<MyData> saver("save.txt", [](FILE* fp, const MyData& a)
                            {fprintf(fp, "%f %.3f %.3f %d\n", a.x, a.y, a.z, a.d);});

    for(int i = 0; i < 100; i++)
    {
        MyData a;
        a.d = i;
        a.x = sin(i);
        a.y = cos(i);
        a.z = tan(i);
        saver.push(a);
    }
    usleep(5000000);
}

void testLineMatch()
{
    PrintHeader();
    LineSegment2DList lines = {LineSegment2D(0, 0, 1, 0),
                               LineSegment2D(0.5, 0.2, 0.7, 0.9),
                               LineSegment2D(1, 2, 3, 4),
                               LineSegment2D(1, 1, 1, 2),
                               LineSegment2D(0, 1, 2, 1.1)};

    cv::Point3f motion(0.2, -0.3, 0.0);
    Eigen::Matrix3d T_gt;
    T_gt << cos(motion.z), -sin(motion.z), motion.x,
            sin(motion.z), cos(motion.z), motion.y,
            0, 0, 1;
    Eigen::Matrix3d Tinv = T_gt.inverse();

    double sigma = 0.005;
    LineSegment2DList target;
    for(const auto& l : lines)
    {
        Eigen::Vector3d v1 = Tinv * Eigen::Vector3d(l.p1.x, l.p1.y, 1);
        Eigen::Vector3d v2 = Tinv * Eigen::Vector3d(l.p2.x, l.p2.y, 1);
        if(randf(1) > 0.5) std::swap(v1, v2);
        target.push_back(LineSegment2D(v1(0)+randn(0, sigma), v1(1)+randn(0, sigma),
                                       v2(0)+randn(0, sigma), v2(1)+randn(0, sigma)));
    }

    {
        cv::Point3f T;
        cv::Mat info;
        bool ok = solveTransform(lines, target, T, info);
        printf("solve: %d (%.3f %.3f %.3f)\n", ok, T.x, T.y, T.z);
        std::cout<<"info"<<info<<std::endl;
    }

    std::reverse(target.begin(), target.end());
    target.erase(target.begin());

    cv::Point3f T2;
    bool ok2 = ICL(lines, target, T2, 0);
    printf("ICL: %d (%.3f %.3f %.3f)\n", ok2, T2.x, T2.y, T2.z);

    cv::Point3f T3;
    ICLSolver solver(lines, 1);
    bool ok3 = solver.match(target, T3, 0);
    printf("ICLSolver: %d (%.3f %.3f %.3f)\n", ok3, T3.x, T3.y, T3.z);
    std::cout<<"Info:"<<solver.getInfo()<<std::endl;
}

void testLineMatch2()
{
    PrintHeader();
    // blackbox test between ICL and ICLSolver
    for(int ite = 0; ite < 50000; ite++)
    {
        auto foo_rand_line = []()
        {
            while(1)
            {
                auto r = randDoubleVec(-100, 100, 4);
                if(r[0] != r[2] || r[1] != r[3])
                    return LineSegment2D(r[0], r[1], r[2], r[3]);
            }
        };
        int N = randi(1, 10);
        LineSegment2DList lines;
        lines.reserve(N);
        for(int i = 0; i < N; i++)
        {
            lines.push_back(foo_rand_line());
        }

        cv::Point3f motion(randf(-0.5, 0.5), randf(-0.5, 0.5), randf(-0.29, 0.29));
        Eigen::Matrix3d T_gt;
        T_gt << cos(motion.z), -sin(motion.z), motion.x,
                sin(motion.z), cos(motion.z), motion.y,
                0, 0, 1;
        Eigen::Matrix3d Tinv = T_gt.inverse();

        double sigma = 0.005;
        LineSegment2DList target;
        for(const auto& l : lines)
        {
            Eigen::Vector3d v1 = Tinv * Eigen::Vector3d(l.p1.x, l.p1.y, 1);
            Eigen::Vector3d v2 = Tinv * Eigen::Vector3d(l.p2.x, l.p2.y, 1);
            if(randf(1) > 0.5) std::swap(v1, v2);
            target.push_back(LineSegment2D(v1(0)+randn(0, sigma),
                                           v1(1)+randn(0, sigma),
                                           v2(0)+randn(0, sigma),
                                           v2(1)+randn(0, sigma)));
        }

        std::reverse(target.begin(), target.end());
        target.erase(target.begin());

        cv::Point3f T2;
        bool ok2 = ICL(lines, target, T2, 0);

        cv::Point3f T3;
        ICLSolver solver(lines, 1);
        bool ok3 = solver.match(target, T3, 0);

        cv::Point3f diff = T3 - T2;
        if(cv::norm(diff) > 1e-4)
        {
            printf("ite:%d Diff %d (%.3f %.3f %.3f) : %d (%.3f %.3f %.3f)\n",
                ite, ok2?1:0, T2.x, T2.y, T2.z, ok3?1:0, T3.x, T3.y, T3.z);
            
            printf("%f %f %f %f\n", lines[0].theta, lines[0].d,
                    target[0].theta, target[0].d);
            getchar();
        }
    }
}


void testCamOdomCalib()
{
    PrintHeader();
    using namespace Eigen;
    Isometry3d T_c_o = Isometry3d::Identity();
    T_c_o.linear() = (AngleAxisd(0.05, Vector3d::UnitZ())
                     * AngleAxisd(1.57, Vector3d::UnitY())
                     * AngleAxisd(-0.02, Vector3d::UnitX())).toRotationMatrix();
    T_c_o.translation() << 0.5, -0.01, 0.25;

    Isometry3d T_m = Isometry3d::Identity();
    T_m.linear() = AngleAxisd(0.01, Vector3d::UnitZ()).toRotationMatrix();
    T_m.translation() << 0.5, 0.4, 0;

    std::vector<Eigen::Isometry3d> cam_poses, odom_poses;
    Isometry3d T_o = Isometry3d::Identity();
    for(double t = - 0.1; t <= 0.1; t += 0.02)
    {
        T_o.translation() << t, 0, 0;
        Isometry3d T_c_m = T_m.inverse() * T_o * T_c_o;
        odom_poses.push_back(T_o);
        cam_poses.push_back(T_c_m);
    }
    T_o.translation() << 0.1, 0.2, 0;
    for(double t = -0.1; t <= 0.1; t += 0.02)
    {
        T_o.linear() = AngleAxisd(t, Vector3d::UnitZ()).toRotationMatrix();
        Isometry3d T_c_m = T_m.inverse() * T_o * T_c_o;
        // std::cout<<"ypr1:"<<T_o.linear().transpose().eulerAngles(0, 1, 2).transpose()<<std::endl;
        // std::cout<<"ypr:"<<T_o.linear().eulerAngles(2, 1, 0).transpose()<<std::endl;
        // std::cout<<"T:"<<T_o.matrix()<<std::endl<<std::endl;
        odom_poses.push_back(T_o);
        cam_poses.push_back(T_c_m);
    }

    Isometry3d T_calib = Eigen::Isometry3d::Identity();
    bool ok = camOdomCalib(cam_poses, odom_poses, T_calib);
    std::cout<<"Real T_c_o:"<<std::endl<<T_c_o.matrix()<<std::endl<<std::endl;
    std::cout<<"Calib T_c_o:"<<ok<<std::endl<<T_calib.matrix()<<std::endl<<std::endl;
    std::cout<<"Diff:"<<std::endl<<(T_calib.inverse()*T_c_o).matrix()<<std::endl<<std::endl;
}

void testColor(int argc, char** argv)
{
    PrintHeader();
    if(argc < 2) return;

    cv::Mat K = (cv::Mat_<double>(3, 3) << 347.907932735926, 0, 316.6024902247042,
                                           0, 348.4394426469446, 236.8952741402495,
                                           0, 0, 1);
    cv::Mat D = (cv::Mat_<double>(1, 4) << -0.024463666926774732, -0.015081763908611457,
                                            -0.0015079423491373898, 0.002447934595710685);
#if 1
    cv::VideoCapture cap(argv[1]);
    if(!cap.isOpened()) return;
    cv::Mat img;
    int start_idx = 0;
    for(int i = 0; cap.read(img); i++)
    {
        while(i < start_idx) continue;
        // cv::Mat img2;
        // cv::fisheye::undistortImage(img, img2, K, D, K);
#else
    for(const auto& f : listdir(argv[1], 1))
    {
        cv::Mat img = cv::imread(f);
#endif
        cv::medianBlur(img, img, 5);
        cv::Mat mask;
        colorFilter(img, mask, defaultColorModel(3), 0.5);
        // std::vector<cv::Mat> bgr;
        // cv::split(img, bgr);
        // bgr[2] += mask;
        // cv::Mat img2;
        // cv::merge(bgr, img2);
        cv::imshow("img", img);
        cv::imshow("mask", mask);
        static bool halt = true;
        uchar key = cv::waitKey(halt ? 0 : 10);
        if(key == 27) break;
        else if(key == 's') halt = !halt;
    }
}

void testLaneDetect(int argc, char** argv)
{
    PrintHeader();
    if(argc < 2) return;

#if 0 //usb web camera taobao
    cv::Mat K = (cv::Mat_<double>(3, 3) << 347.907932735926, 0, 316.6024902247042,
                                           0, 348.4394426469446, 236.8952741402495,
                                           0, 0, 1);
    cv::Mat D = (cv::Mat_<double>(1, 4) << -0.024463666926774732, -0.015081763908611457,
                                         -0.0015079423491373898, 0.002447934595710685);
    cv::Mat K_new = (cv::Mat_<double>(3, 3) << 250, 0, 320,
                                           0, 250, 240,
                                           0, 0, 1);
    int roi_row = 20;
    cv::Mat K_roi = K_new.clone();
    K_roi.at<double>(1, 2) -= roi_row;
    cv::Mat T_c_b = (cv::Mat_<double>(4, 4) << -0.507232, -0.789457,  0.345649,  0.575,
                                              -0.797997 , 0.278785, -0.534303, -0.23,
                                              0.325447, -0.546842, -0.771393,  0.265,
                                               0.0, 0.0, 0.0, 1.0);
#endif
#if 1 // mindvision industrial camera
    cv::Mat K = (cv::Mat_<double>(3, 3) << 451.0356167313587, 0, 323.92094563577683,
                                           0, 451.0540055780082, 244.13028827466718,
                                           0, 0, 1);
    cv::Mat D = (cv::Mat_<double>(1, 4) << -0.3792862607865829, 0.1333768262529482, 
                                           -0.00046334666522035564, 0.0038021330351276496);
    double f = K.at<double>(0, 0) * 0.9;
    cv::Mat K_new = (cv::Mat_<double>(3, 3) << f, 0, 320,
                                           0, f, 240,
                                            0, 0, 1);
    int roi_row = 0;
    cv::Mat K_roi = K_new.clone();
    K_roi.at<double>(1, 2) -= roi_row;
    cv::Mat T_c_b = (cv::Mat_<double>(4, 4) << -0.0119137 ,  -0.562969 ,   0.826392,     0.595,
                                            -0.999923 , 0.00964968, -0.00784175  , -0.036,
                                            -0.00355976,   -0.826422,    -0.56304 , 0.515,
                                            0.0,0.0,0.0,1.0);
#endif
    
#if 0
    cv::Mat T_c_b = cv::Mat::eye(4, 4, CV_64FC1);
    double pitch = -deg2rad(70);//- atan2(240 - 60, 250);
    T_c_b(cv::Rect(0, 0, 3, 3)) = (cv::Mat_<double>(3, 3)<<cos(pitch), 0, sin(pitch),
                                    0, 1, 0, sin(pitch), 0, cos(pitch)) *
                                (cv::Mat_<double>(3, 3)<<0, 0, 1, -1, 0, 0, 0, -1, 0);
    T_c_b.at<double>(0, 3) = 0.4;
    T_c_b.at<double>(1, 3) = 0;
    T_c_b.at<double>(2, 3) = 0.3;
#endif

    cv::VideoCapture cap(argv[1]);
    if(!cap.isOpened()) return;
    cv::Mat img;
    int start_idx = 0;//3500;//13268;
    for(int i = 0; cap.read(img); i++)
    {
        printf("======%d=========\n", i);
        if(i < start_idx) continue;
    #if 1
        cv::fisheye::undistortImage(img, img, K, D, K_new);
        img = img.rowRange(roi_row, img.rows);
    #else
        cv::Mat timg;
        cv::undistort(img, timg, K, D, K_new);
        img = timg.rowRange(roi_row, img.rows);
    #endif
        LaneList lanes;
        laneDetect(img, lanes, K_roi, T_c_b, 3, 1);
        static bool halt = true;
        uchar key = cv::waitKey(halt ? 0 : 10);
        if(key == 27) break;
        else if(key == 's') halt = !halt;
        else if(key == 'n') start_idx = i + 100;
    }
}

void testCamCapture()
{
    CamCapture cap;
    cap.init(0, "out.avi");
    cap.start();
    usleep(1e6);
    double prev_ts = 0;
    cv::Mat img;
    while(1)
    {
        double ts = cap.getLatest(img);
        if(ts > prev_ts)
        {
            prev_ts = ts;
            cv::imshow("img", img);
            uchar key = cv::waitKey(10);
            if(key == 27) break;
        }
        usleep(5e3);
    }
}

void testUndistortImages()
{
    cv::Mat K = (cv::Mat_<double>(3, 3) << 451.0356167313587, 0, 323.92094563577683,
                                           0, 451.0540055780082, 244.13028827466718,
                                           0, 0, 1);
    cv::Mat D = (cv::Mat_<double>(1, 4) << -0.3792862607865829, 0.1333768262529482, 
                                           -0.00046334666522035564, 0.0038021330351276496);
    double f = K.at<double>(0, 0)*0.9;
    cv::Mat K_new = (cv::Mat_<double>(3, 3) << f, 0, 320,
                                           0, f, 240,
                                            0, 0, 1);

    for(auto f : listdir("/home/symao/data/yellow_line/todo/", 1))
    {
        cv::Mat img = cv::imread(f);
        cv::Mat timg;
        cv::undistort(img, timg, K, D, K_new);
        cv::imshow("img", timg);
        cv::imwrite(f, timg);
        cv::waitKey(0);
    }
}

void testTimeBuffer()
{
    TimeBuffer<cv::Point2f> buf;
    buf.add(0, cv::Point2f(0, 0));
    buf.add(1, cv::Point2f(1, -2));
    buf.add(2, cv::Point2f(2, -3));
    auto f = [&buf](double t)
    {
        cv::Point2f res(0, 0);
        bool flag = buf.get(t, res);
        printf("ts:%f find:%d res:(%f %f)\n", t, flag, res.x, res.y);
    };
    f(0);
    f(1);
    f(0.2);
    f(0.4);
    f(1.001);
    f(2.0001);
    f(1.999);
    f(-1);
    TimeBuffer<double> angle_buf([](double k1, const double& a1,
                                double k2, const double& a2){
        double a3 = normalizeRad(a2 - a1) + a1;
        return normalizeRad(k1 * a1 + k2 * a3);
    });

    angle_buf.add(0, -0.1);
    angle_buf.add(1, 0.1);
    angle_buf.add(2, 1.5);
    angle_buf.add(3, 3.141);
    angle_buf.add(4, -3.14);
    auto f2 = [&angle_buf](double t)
    {
        double res;
        bool flag = angle_buf.get(t, res);
        printf("ts:%f find:%d res:%f\n", t, flag, res);
    };
    f2(0);
    f2(0.1);
    f2(1.3);
    f2(2.4);
    f2(3.2);
    f2(3.6);
    f2(3.9);
    f2(4.1);
}

void testPCA()
{
    Eigen::MatrixXd data(6, 2);
    data << 0, 1, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10;

    Eigen::MatrixXd eig_val, eig_coef, center;
    bool ok = PCA(data, eig_val, eig_coef, center);
    std::cout << "ok:" << ok << std::endl;
    std::cout << "eig_val:" << eig_val.transpose() << std::endl;
    std::cout << "eig_coef:" << eig_coef << std::endl;
    std::cout << "center:" << center.transpose() << std::endl;
}

void testLineFit()
{
    Eigen::MatrixXd data(6, 3);
    data << 0, 1, 0, 1, 2, 0, 3, 4, 0, 5, 6, 0, 7, 8, 0, 9, 10, 0;

    Eigen::MatrixXd p1, p2;
    lineFit(data, p1, p2);

    std::cout<<p1.transpose()<<" | "<<p2.transpose()<<std::endl;
}

void testSyslog()
{
    const char* logdir = "/media/symao/PI_BOOT/log/sfvision";
    printf("Dir %s, All %.1fM, Free %.1fM\n", logdir,
            disksize(logdir) * 1e-9, diskfree(logdir) * 1e-9);
    initSysLog(logdir, 4e9/*, "/home/symao/syslog", 5e6*/);
}

void testRandSample()
{
    std::vector<int> vec;
    int N = 1000;
    for(int i = 0; i < N; i++)
    {
        vec.push_back(randi(5));
    }

    for(int idx = 0; idx < 100000; idx++)
    {
        std::map<int, int> hist;
        for(auto i : vec)
            hist[i]++;
        printf("#%d: ", idx);
        for(auto i : hist)
            printf("%d(%d) ", i.first, i.second);
        printf("\n");

        std::vector<double> weights;
        for(auto i : vec)
            weights.push_back(1000 + i);
        auto ids = weightedSample(weights, N, 2);
        std::vector<int> new_vec;
        for(auto i : ids)
            new_vec.push_back(vec[i]);
        vec = new_vec;
        usleep(10000);
    }
}

void testKDTree()
{
    auto frand = [](){return std::vector<float>(
                {(float)randf(5000), (float)randf(5000), (float)randf(5000)});};
    int N = 10000000;
    std::vector<std::vector<float> > data;
    for(int i = 0; i < N; i++)
    {
        data.push_back(frand());
    }
    std::vector<float> query = frand();

    Timer t;
    auto kdt = createKDTree(0);
    t.start();
    kdt->build(data);
    double t1 = t.stop();
    int K = 50000;
    int nouse = 0;
    t.start();
    KDTree::DataArray res;
    for(int i = 0; i < K; i++)
    {
        kdt->k_nearest(query, res, 10);
        if(i == 0) printf("k_nearest: %d\n", (int)res.size());
        nouse += res.size();
    }
    double t2 = t.stop();
    t.start();
    for(int i = 0; i < K; i++)
    {
        kdt->r_nearest(query, res, 100);
        if(i == 0) printf("r_nearest: %d\n", (int)res.size());
        nouse += res.size();
    }
    double t3 = t.stop();

    printf("query:(%f %f %f)\n", query[0], query[1], query[2]);
    kdt->k_nearest(query, res, 1);
    printf("nearest:(%f %f %f)\n", res[0][0], res[0][1], res[0][2]);

    printf("Build %d data tree: %.1f ms,\n"
           "Ksearch %d times: %.1f ms,\n"
           "Rsearch %d times: %.1f ms\n", N, t1, K, t2, K, t3);
}

std::vector<double> foo()
{
    std::vector<double> a = {1,2,3};
    a.reserve(100000);
    return a;
}

int main(int argc, char** argv)
{
    // test1();
    // test2();
    // testAlias();
    // testWeightedSample();
    // testShuffle();
    // testRaycast2D();
    // testStereoRectifier();
    // testDataSaver();
    // testLineMatch();
    // testLineMatch2();
    // testCamOdomCalib();
    // testColor(argc, argv);
    testLaneDetect(argc, argv);
    // testCamCapture();
    // testUndistortImages();
    // testTimeBuffer();
    // testPCA();
    // testLineFit();
    // testSyslog();
    // testRandSample();
    // testKDTree();
}


