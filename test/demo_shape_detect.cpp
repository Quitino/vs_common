#include "vs_shape_detect.h"
#include "vs_tictoc.h"

int processImage(const cv::Mat& gray, int marker_type = 0)
{
    std::vector<cv::Point2f> centers;
    int n_markers = vs::detectMarker(gray, centers, marker_type, 1);
    return n_markers;
}

int main(int argc, char** argv)
{
    int marker_type = 0;
    int start_index = 0;
    cv::VideoCapture cap;

    if(argc < 3)
        cap.open(-1);
    else
        cap.open(argv[2]);

    if(argc > 1)
        marker_type = atoi(argv[1]);
    if(argc > 3)
        start_index = atoi(argv[3]);

    float cost_sum = 0;
    int cost_cnt = 0;

    bool halt = 1;
    if(cap.isOpened())
    {
        cv::Mat img;
        for(int idx = 0; cap.read(img); idx++)
        {
            if(idx < start_index) continue;
            cv::Mat gray;
            cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
            // gray = gray.rowRange(0, gray.rows/2);
            if(gray.rows >= 480)
            {
                cv::resize(gray, gray, cv::Size(gray.cols/2, gray.rows/2));
            }
            vs::Timer t;
            processImage(gray, marker_type);
            t.stop();
            double cost = t.getMsec();
            printf("#%d cost:%.1f ms\n", idx, cost);
            cost_sum += cost;
            cost_cnt++;
            char key = cv::waitKey(halt ? 0 : 10);
            if(key == 27) break;
            else if(key == 's') halt = !halt;
        }
    }
    if(cost_cnt > 0)
        printf("Process %d images, average cost %.1f ms\n", cost_cnt, cost_sum/cost_cnt);
}