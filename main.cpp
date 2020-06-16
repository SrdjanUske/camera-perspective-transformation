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

static bool defined_warp_pole = false;
static bool defined_inverse_pole = false;
static cv::Point2f pole_locations_warp[4];
static std::vector<cv::Point2f> pole_locations_inverse;

enum line_Mofidy {
    SPREAD = 0,
    COMPRESS = 1,
    TRANSLATE_UP = 2,
    TRANSLATE_DOWN = 3
};

enum CameraView
{
    LEFT,
    RIGHT
};

// Line modification only for input camera image
static void vertical_road_line_modification(cv::Point2f src[], cv::Point2f dst[],
int weight, enum line_Mofidy mode, enum CameraView camView)
{
    int diff_x = abs(src[0].x - src[1].x);
    int diff_y = abs(src[0].y - src[1].y);
    double theta = atan2(diff_y, diff_x);

    switch (camView)
    {
    case LEFT:
        std::cout << __func__ << " : Left camera angle." << std::endl;
        switch (mode)
        {
        case SPREAD:
            dst[0].x = int(src[0].x - weight * cos(theta));
            dst[0].y = int(src[0].y - weight * sin(theta));
            dst[1].x = int(src[1].x + weight * cos(theta));
            dst[1].y = int(src[1].y + weight * sin(theta));
            break;
        case COMPRESS:
            dst[0].x = int(src[0].x + weight * cos(theta));
            dst[0].y = int(src[0].y + weight * sin(theta));
            dst[1].x = int(src[1].x - weight * cos(theta));
            dst[1].y = int(src[1].y - weight * sin(theta));
            break;
        case TRANSLATE_UP:
            dst[0].x = int(src[0].x - weight * cos(theta));
            dst[0].y = int(src[0].y - weight * sin(theta));
            dst[1].x = int(src[1].x - weight * cos(theta));
            dst[1].y = int(src[1].y - weight * sin(theta));
            break;
        case TRANSLATE_DOWN:
            dst[0].x = int(src[0].x + weight * cos(theta));
            dst[0].y = int(src[0].y + weight * sin(theta));
            dst[1].x = int(src[1].x + weight * cos(theta));
            dst[1].y = int(src[1].y + weight * sin(theta));
            break;
        default:
            std::cout << "LEFT: Unrecognized line modification mode!" << std::endl;
            break;
        }
        break;
    case RIGHT:
        std::cout << __func__ << " : Right camera angle." << std::endl;
        switch (mode)
        {
        case SPREAD:
            dst[0].x = int(src[0].x + weight * cos(theta));
            dst[0].y = int(src[0].y - weight * sin(theta));
            dst[1].x = int(src[1].x - weight * cos(theta));
            dst[1].y = int(src[1].y + weight * sin(theta));
            break;
        case COMPRESS:
            dst[0].x = int(src[0].x - weight * cos(theta));
            dst[0].y = int(src[0].y + weight * sin(theta));
            dst[1].x = int(src[1].x + weight * cos(theta));
            dst[1].y = int(src[1].y - weight * sin(theta));
            break;
        case TRANSLATE_UP:
            dst[0].x = int(src[0].x + weight * cos(theta));
            dst[0].y = int(src[0].y - weight * sin(theta));
            dst[1].x = int(src[1].x + weight * cos(theta));
            dst[1].y = int(src[1].y - weight * sin(theta));
            break;
        case TRANSLATE_DOWN:
            dst[0].x = int(src[0].x - weight * cos(theta));
            dst[0].y = int(src[0].y + weight * sin(theta));
            dst[1].x = int(src[1].x - weight * cos(theta));
            dst[1].y = int(src[1].y + weight * sin(theta));
            break;
        default:
            std::cout << "RIGHT: Unrecognized line modification mode!" << std::endl;
            break;
        }
        break;
    default:
        std::cout << __func__ << " : Unrecognized angle." << std::endl;
        break;
    }
    if (dst[0].x >= WIDTH)
        dst[0].x = WIDTH - 1;
    if (dst[0].y >= HEIGHT)
        dst[0].y = HEIGHT - 1;
    if (dst[1].x >= WIDTH)
        dst[1].x = WIDTH - 1;
    if (dst[1].y >= HEIGHT)
        dst[1].y = HEIGHT - 1;
}

static void prepare_destination_warp(cv::Point2f dst_warp[], int start_point_high,
int modification_point_high, int start_point_low, int modification_point_low)
{

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
}

static void prepare_source_destination_inverse(cv::Point2f dst_warp[], std::vector<cv::Point2f>& src_inverse,
std::vector<cv::Point2f>& dst_inverse, int start_point_high, int modification_point_high,
int start_point_low, int modification_point_low)
{
    int line_high = (int)cv::norm(coordinates[start_point_high] - coordinates[modification_point_high]);
    int line_low = (int)cv::norm(coordinates[start_point_low] - coordinates[modification_point_low]);
    int line_offset_high = (WIDTH - line_high) / 2;
    int line_offset_low = (WIDTH - line_low) / 2;

    cv::Point2f new_points_warp[NUM_KEY_POINTS];
    int offset_height_inverse = abs(coordinates[start_point_high].y - coordinates[start_point_low].y) / 2;

    new_points_warp[0] = cv::Point2f(line_offset_high, HEIGHT - offset_height_inverse);
    new_points_warp[1] = cv::Point2f(line_offset_high + line_high, HEIGHT - offset_height_inverse);
    new_points_warp[2] = cv::Point2f(line_offset_high, HEIGHT);
    new_points_warp[3] = cv::Point2f(line_offset_high + line_high, HEIGHT);

    src_inverse.push_back(dst_warp[2]);
    src_inverse.push_back(dst_warp[3]);
    src_inverse.push_back(dst_warp[1]);
    src_inverse.push_back(dst_warp[0]);

    dst_inverse.push_back(new_points_warp[2]);
    dst_inverse.push_back(new_points_warp[3]);
    dst_inverse.push_back(new_points_warp[1]);
    dst_inverse.push_back(new_points_warp[0]);
}

static void next_rect(std::vector<cv::Point2f> src_points, std::vector<cv::Point2f>& dst_points)
{
    cv::Point2f tmp[4];

    tmp[0] = src_points[3];
    tmp[1] = src_points[2];

    int diff_x_1 = abs(src_points[2].x - src_points[1].x);
    int diff_y_1 = abs(src_points[2].y - src_points[1].y);
    int distance_1 = cv::norm(src_points[2] - src_points[1]);
    double theta_1 = atan2(diff_y_1, diff_x_1);

    tmp[2].x = int(src_points[2].x - distance_1 * cos(theta_1));
    tmp[2].y = int(src_points[2].y - distance_1 * sin(theta_1));

    int diff_x_2 = abs(src_points[3].x - src_points[0].x);
    int diff_y_2 = abs(src_points[3].y - src_points[0].y);
    int distance_2 = cv::norm(src_points[3] - src_points[0]);
    double theta_2 = atan2(diff_y_2, diff_x_2);

    tmp[3].x = int(src_points[3].x - distance_2 * cos(theta_2));
    tmp[3].y = int(src_points[3].y - distance_2 * sin(theta_2));

    dst_points.push_back(tmp[0]);
    dst_points.push_back(tmp[1]);
    dst_points.push_back(tmp[2]);
    dst_points.push_back(tmp[3]);
}

static void perspective_transform_element_wise(cv::Point2f src, cv::Point2f& dest, cv::Mat warpMatrix)
{
    dest.x = (int)((warpMatrix.at<double>(0,0)*src.x + warpMatrix.at<double>(0,1)*src.y + warpMatrix.at<double>(0,2)) /
            (warpMatrix.at<double>(2,0)*src.x + warpMatrix.at<double>(2,1)*src.y + warpMatrix.at<double>(2,2)));
    dest.y = (int)((warpMatrix.at<double>(1,0)*src.x + warpMatrix.at<double>(1,1)*src.y + warpMatrix.at<double>(1,2)) /
            (warpMatrix.at<double>(2,0)*src.x + warpMatrix.at<double>(2,1)*src.y + warpMatrix.at<double>(2,2)));
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
    std::vector<cv::Point2f> dst_inverse_next_rect_1;
    std::vector<cv::Point2f> dst_inverse_next_rect_2;
    std::vector<cv::Point2f> dst_inverse_next_rect_3;

    cv::namedWindow("input", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("output_warp", cv::WINDOW_NORMAL);
    cv::namedWindow("output_inverse", cv::WINDOW_AUTOSIZE);

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
                for (int n = 0; n < NUM_KEY_POINTS; n++)
                    pole_locations_warp[n] = coordinates[n];

                enum CameraView cameraView;
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

                cv::Point2f line_modify_l[2];
                cv::Point2f line_modify_r[2];
                switch (cameraView)
                {
                case LEFT:
                    line_modify_l[0] = coordinates[start_point_high - 1];
                    line_modify_l[1] = coordinates[start_point_low];
                    line_modify_r[0] = coordinates[start_point_high];
                    line_modify_r[1] = coordinates[start_point_low + 1];
                    vertical_road_line_modification(line_modify_l, line_modify_l, 200, SPREAD, LEFT);
                    vertical_road_line_modification(line_modify_r, line_modify_r, 200, SPREAD, LEFT);
                    coordinates[start_point_high - 1] = line_modify_l[0];
                    coordinates[start_point_low] = line_modify_l[1];
                    coordinates[start_point_high] = line_modify_r[0];
                    coordinates[start_point_low + 1] = line_modify_r[1];
                    break;
                case RIGHT:
                    line_modify_l[0] = coordinates[start_point_high];
                    line_modify_l[1] = coordinates[start_point_low - 1];
                    line_modify_r[0] = coordinates[start_point_high + 1];
                    line_modify_r[1] = coordinates[start_point_low];
                    vertical_road_line_modification(line_modify_l, line_modify_l, 200, SPREAD, RIGHT);
                    vertical_road_line_modification(line_modify_r, line_modify_r, 200, SPREAD, RIGHT);
                    coordinates[start_point_high] = line_modify_l[0];
                    coordinates[start_point_low - 1] = line_modify_l[1];
                    coordinates[start_point_high + 1] = line_modify_r[0];
                    coordinates[start_point_low] = line_modify_r[1];
                    break;
                }

                prepare_destination_warp(dst_warp, start_point_high, modification_point_high,
                    start_point_low, modification_point_low);
                prepare_source_destination_inverse(dst_warp, src_inverse, dst_inverse, start_point_high,
                    modification_point_high, start_point_low, modification_point_low);

                next_rect(dst_inverse, dst_inverse_next_rect_1);
                next_rect(dst_inverse_next_rect_1, dst_inverse_next_rect_2);
                next_rect(dst_inverse_next_rect_2, dst_inverse_next_rect_3);

                clickEvent = false;
            }
        }
        else
        {
            // 1) Warp perspective
            cv::Mat warpMatrix = cv::getPerspectiveTransform(coordinates, dst_warp);
            if (!defined_warp_pole)
            {
                for (int i = 0; i < NUM_KEY_POINTS; i++)
                    perspective_transform_element_wise(pole_locations_warp[i], pole_locations_warp[i], warpMatrix);
                defined_warp_pole = true;
            }
            cv::warpPerspective(opencv_bgr_frame, result_warp, warpMatrix, result_warp.size());
            for (int m = 0; m < mouse_move_cnt; m++)
                circle(result_warp, pole_locations_warp[m], 5, cv::Scalar(255, 0, 255), -1);

            // 2) Inverse perspective transform
            IPM ipm(cv::Size(WIDTH, HEIGHT), cv::Size(WIDTH, HEIGHT), src_inverse, dst_inverse);
            if (!defined_inverse_pole)
            {
                pole_locations_inverse.push_back(pole_locations_warp[0]);
                pole_locations_inverse.push_back(pole_locations_warp[1]);
                pole_locations_inverse.push_back(pole_locations_warp[2]);
                pole_locations_inverse.push_back(pole_locations_warp[3]);
                for (int i = 0; i < NUM_KEY_POINTS; i++)
                    perspective_transform_element_wise(pole_locations_inverse[i], pole_locations_inverse[i], ipm.getH());
                defined_inverse_pole = true;
            }
            ipm.applyHomography(result_warp, result_inverse);
            ipm.drawPoints(src_inverse, result_warp, cv::Scalar(0,205,205));
            ipm.drawPoints(dst_inverse, result_inverse, cv::Scalar(0,205,205));
            ipm.drawPoints(dst_inverse_next_rect_1, result_inverse, cv::Scalar(255,0,0));
            ipm.drawPoints(dst_inverse_next_rect_2, result_inverse, cv::Scalar(0,0,255));

            for (int m = 0; m < mouse_move_cnt; m++)
                circle(result_inverse, pole_locations_inverse[m], 5, cv::Scalar(255, 0, 255), -1);

            out_video_warp.write(result_warp);
            out_video_inverse.write(result_inverse);
            cv::imshow("output_warp", result_warp);
            cv::imshow("output_inverse", result_inverse);
        }
        for (int m = 0; m < mouse_move_cnt; m++)
            circle(opencv_bgr_frame, coordinates[m], 5, cv::Scalar(0, 0, 255), -1);
        cv::imshow("input", opencv_bgr_frame);
        if (cv::waitKey(1) >= 0)
            break;
    }

    cv::destroyAllWindows();
    out_video_warp.release();
    out_video_inverse.release();
    uyvy_stream.close();

    return 0;
}