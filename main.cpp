#include <iostream>
#include <fstream>
#include <cmath>
#include <opencv2/opencv.hpp>

#include "IPM.h"
#include "mouse-event.h"

#define WIDTH 1920
#define HEIGHT 1080
#define FRAME_SIZE_UYVY (WIDTH * HEIGHT * 2)
#define PI 3.1415926

enum CameraView
{
    LEFT,
    RIGHT
};

static cv::Point2f rotate_point(cv::Point2f p0, cv::Point2f p1, double sin_Q, double cos_Q)
{
    cv::Point2f result;
    result.x = ((p1.x-p0.x)*cos_Q) - ((p1.y-p0.y)*sin_Q) + p0.x;
    result.y = ((p1.x-p0.x)*sin_Q) + ((p1.y-p0.y)*cos_Q) + p0.y;
    return result;
}

int main(int argc, char* argv[])
{
    int draw_points;
    if (argc < 2)
    {
        std::cout << std::endl << "Usage:" << std::endl;
        std::cout << "./perspective-transform <videoID> <drawPoints>" << std::endl;
        std::cout << "<videoID> - 1 | 2" << std::endl;
        std::cout << "<drawPoints> - y" << std::endl;
        std::cout << "" << std::endl;
        exit(-11);
    }

    std::ifstream uyvy_stream;
    unsigned char uyvy_frame[FRAME_SIZE_UYVY];
    cv::Mat opencv_uyvy_frame(HEIGHT, WIDTH, CV_8UC2);
    cv::Mat opencv_bgr_frame(HEIGHT, WIDTH, CV_8UC3);
    char filename[200];
    char out_file_warp[200];
    char out_file_inverse[200];

    snprintf(filename, sizeof(filename),
        "/home/rtrk/Desktop/Faculty/Master-rad/01-frame-grabber/camera-inputs/stalak-2020-05-21/SSD-1/Out%d.yuv",
        atoi(argv[1]));

    uyvy_stream.open(filename, std::ios_base::binary);

    if (uyvy_stream.is_open())
    {
        std::cout << "Stream open" << std::endl;
    }
    else
    {
        std::cout << "Can not open stream" << std::endl;
        exit(-11);
    }

    cv::Point2f dst_warp[4];
    std::vector<cv::Point2f> src_inverse;
    std::vector<cv::Point2f> dst_inverse;

    // if (atoi(argv[1]) == 1)
    // {
    //     // src_warp[0] = cv::Point(1520, 355);
    //     // src_warp[1] = cv::Point(1915, 425);
    //     // src_warp[2] = cv::Point(670, 675);
    //     // src_warp[3] = cv::Point(1690, 935);
    // }
    // else if (atoi(argv[1]) == 2)
    // {
    //     // src_warp[0] = cv::Point(80, 265);
    //     // src_warp[1] = cv::Point(460, 230);
    //     // src_warp[2] = cv::Point(180, 525);
    //     // src_warp[3] = cv::Point(1550, 680);
    // }

    cv::namedWindow("input", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("output_warp", cv::WINDOW_NORMAL);
    //cv::namedWindow("output_inverse", cv::WINDOW_AUTOSIZE);

    snprintf(out_file_warp, sizeof(out_file_warp), "output_warp_%d.mp4", atoi(argv[1]));
    snprintf(out_file_inverse, sizeof(out_file_inverse), "output_inverse_%d.mp4", atoi(argv[1]));
    cv::VideoWriter out_video_warp(out_file_warp, CV_FOURCC('M','J','P','G'), 10, opencv_bgr_frame.size());
    cv::VideoWriter out_video_inverse(out_file_inverse, CV_FOURCC('M','J','P','G'), 10, opencv_bgr_frame.size());

    bool clickEvent = true;

    while (!uyvy_stream.eof())
    {
        cv::Mat result_warp(HEIGHT, WIDTH, CV_8UC3, cv::Scalar(255, 255, 255));
        cv::Mat result_inverse;
        uyvy_stream.read((char*)uyvy_frame, FRAME_SIZE_UYVY);
        opencv_uyvy_frame.data = uyvy_frame;
        cv::cvtColor(opencv_uyvy_frame, opencv_bgr_frame, cv::COLOR_YUV2BGR_UYVY);
        if (clickEvent)
        {
            if(!click_finished)
                cv::setMouseCallback("input", MouseCallBackFunc, (void*)&opencv_bgr_frame);
            else
            {
                enum CameraView cameraView;;
                int start_point_low = 0;
                int start_point_high = 0;
                int modification_point_low;
                int modification_point_high;
                for (int s = 1; s < NUM_KEY_POINTS; s++)
                {
                    if (coordinates[s].y > coordinates[start_point_low].y)
                        start_point_low = s;
                    else if (coordinates[s].y < coordinates[start_point_high].y)
                        start_point_high = s;
                }

                if ((start_point_low + 1) < NUM_KEY_POINTS)
                {
                    cameraView = LEFT;
                    modification_point_low = start_point_low + 1;
                    modification_point_high = start_point_high - 1;
                }
                else
                {
                    cameraView = RIGHT;
                    modification_point_low = start_point_low - 1;
                    modification_point_high = start_point_high + 1;
                }   

                int line_high = (int)cv::norm(coordinates[start_point_high] - coordinates[modification_point_high]);
                int line_low = (int)cv::norm(coordinates[start_point_low] - coordinates[modification_point_low]);
                int line_offset_high = (WIDTH - line_high) / 2;
                int line_offset_low = (WIDTH - line_low) / 2;

                cv::Point2f new_points_warp[NUM_KEY_POINTS];

                new_points_warp[0] = cv::Point2f(line_offset_high, coordinates[start_point_high].y);
                new_points_warp[1] = cv::Point2f(line_offset_high + line_high, coordinates[start_point_high].y);
                new_points_warp[2] = cv::Point2f(line_offset_low, coordinates[start_point_low].y);
                new_points_warp[3] = cv::Point2f(line_offset_low + line_low, coordinates[start_point_low].y);

                for (int n = 0; n < NUM_KEY_POINTS; n++)
                    dst_warp[n] = new_points_warp[n];

                int line_middle = (line_high + line_low) / 2;
                int offset_inverse = (WIDTH - line_middle) / 2;

                new_points_warp[0] = cv::Point2f(offset_inverse, coordinates[start_point_high].y);
                new_points_warp[1] = cv::Point2f(offset_inverse + line_middle, coordinates[start_point_high].y);
                new_points_warp[2] = cv::Point2f(offset_inverse, coordinates[start_point_low].y);
                new_points_warp[3] = cv::Point2f(offset_inverse + line_middle, coordinates[start_point_low].y);


                src_inverse.push_back(dst_warp[2]);
                src_inverse.push_back(dst_warp[3]);
                src_inverse.push_back(dst_warp[1]);
                src_inverse.push_back(dst_warp[0]);

                dst_inverse.push_back(new_points_warp[2]);
                dst_inverse.push_back(new_points_warp[3]);
                dst_inverse.push_back(new_points_warp[1]);
                dst_inverse.push_back(new_points_warp[0]);
                
                clickEvent = false;
            }
        }
        else
        {
            // 1) Warp perspective
            cv::Mat warpMatrix = cv::getPerspectiveTransform(coordinates, dst_warp);
            cv::warpPerspective(opencv_bgr_frame, result_warp, warpMatrix, result_warp.size());

            // 2) Inverse perspective transform
            IPM ipm(cv::Size(WIDTH, HEIGHT), cv::Size(WIDTH, HEIGHT), src_inverse, dst_inverse);
            ipm.applyHomography(result_warp, result_inverse);
            ipm.drawPoints(src_inverse, result_warp);
            ipm.drawPoints(dst_inverse, result_inverse);
            cv::imshow("output_warp", result_warp);

            out_video_warp.write(result_warp);
            out_video_inverse.write(result_inverse);
            cv::imshow("output_inverse", result_inverse);
        }
        for (int m = 0; m < mouse_move_cnt; m++)
            circle(opencv_bgr_frame, coordinates[m], 5, cv::Scalar(0, 0, 255), -1);
        cv::imshow("input", opencv_bgr_frame);
        if (cv::waitKey(20) >= 0)
            break;
    }

    cv::destroyAllWindows();
    out_video_warp.release();
    out_video_inverse.release();
    uyvy_stream.close();

    return 0;
}