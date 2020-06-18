#include "perspective-transform.h"
#include <iostream>
#include <opencv2/opencv.hpp>

#define IMAGE_WIDTH             1920
#define IMAGE_HEIGHT            1080
#define FRAME_SIZE_UYVY         (IMAGE_WIDTH * IMAGE_HEIGHT * 2)

int main(int argc, char* argv[])
{
    if (argc < 2)
    {
        std::cout << std::endl << "Usage:" << std::endl;
        std::cout << "./perspective-transform <videoID> <drawPoints>" << std::endl;
        std::cout << "<videoID> - 1 | 2" << std::endl;
        std::cout << "<drawPoints> - y" << std::endl;
        std::cout << "" << std::endl;
        exit(-11);
    }

    char filename[200];
    std::ifstream uyvy_stream;
    snprintf(filename, sizeof(filename),
        "/home/rtrk/Desktop/Faculty/Master-rad/01-frame-grabber/camera-inputs/stalak-2020-05-21/SSD-1/Out%d.yuv",
        atoi(argv[1]));

    uyvy_stream.open(filename, std::ios_base::binary);

    if (uyvy_stream.is_open())
        std::cout << "Stream open" << std::endl;
    else
    {
        std::cout << "Can not open stream" << std::endl;
        exit(-11);
    }

    unsigned char uyvy_frame[FRAME_SIZE_UYVY];
    cv::Mat opencv_uyvy_frame(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC2);
    cv::Mat opencv_bgr_frame(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3);
    int frame_cnt = 0;
    bool finished_clicking = false;
    while (!uyvy_stream.eof())
    {
        uyvy_stream.read((char*)uyvy_frame, FRAME_SIZE_UYVY);
        opencv_uyvy_frame.data = uyvy_frame;
        cv::cvtColor(opencv_uyvy_frame, opencv_bgr_frame, cv::COLOR_YUV2BGR_UYVY);

        if (finished_clicking)
            get_perspective_transform((void*)&opencv_bgr_frame);
        else
            finished_clicking = mouse_click_and_param_init((void*)&opencv_bgr_frame);
    }
    return 0;
}