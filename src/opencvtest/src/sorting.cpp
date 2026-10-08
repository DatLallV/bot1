/********************************* ROS ***********************************************/
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

/********************************** OPENCV LIBRARIES **********************************/
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <opencvtest/pixel_point0.h>
#include <iostream>
#include <vector>

using namespace std;
using namespace cv;

// 用于 HSV 取色的全局变量
cv::Mat g_hsv_display;
bool g_hsv_selected = false;
int g_hsv_x = 0, g_hsv_y = 0;
cv::Vec3b g_hsv_val, g_bgr_val;

// 鼠标回调函数
void onMouse(int event, int x, int y, int flags, void* userdata)
{
    if (event == cv::EVENT_LBUTTONDOWN && !g_hsv_display.empty())
    {
        cv::Mat hsv;
        cv::cvtColor(g_hsv_display, hsv, cv::COLOR_BGR2HSV);
        g_hsv_val = hsv.at<cv::Vec3b>(y, x);
        g_bgr_val = g_hsv_display.at<cv::Vec3b>(y, x);
        g_hsv_x = x;
        g_hsv_y = y;
        g_hsv_selected = true;
    }
}

/********************************** HSV 阈值定义 **************************************/

// 颜色结构体：存储颜色名称和HSV范围
struct ColorConfig
{
    string name;
    int h_min, h_max;
    int s_min, s_max;
    int v_min, v_max;
    int id;  // 颜色ID，用于switch-case
};

// 预设6种颜色的HSV范围 (H: 0-180, S: 0-255, V: 0-255)
// 注意：实际使用时需根据环境光照微调

// 红色：分为0-10和156-180两个区间（因为红色在HSV中跨越0°边界）
ColorConfig red1   = {"red1",   0, 10,   43, 255,  46, 255, 1};
ColorConfig red2   = {"red2", 156, 180,  43, 255,  46, 255, 1};

ColorConfig orange = {"orange", 11, 25,   43, 255,  46, 255, 2};
ColorConfig yellow = {"yellow", 26, 34,   43, 255,  46, 255, 3};
ColorConfig green  = {"green",  35, 77,   43, 255,  46, 255, 4};
ColorConfig blue   = {"blue",  100, 124,  43, 255,  46, 255, 5};
ColorConfig purple = {"purple",125, 155,  43, 255,  46, 255, 6};

/********************************** 全局变量 ******************************************/

static const string WINDOW_ORIGINAL = "Original Image";
static const string WINDOW_BINARY = "Binary Image";
static const string WINDOW_RESULT = "Result Image";

/********************************** ImageConverter 类 *********************************/

class ImageConverter
{
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    ros::Publisher center_point_pub_;

    opencvtest::pixel_point0 msgs;  // 自定义消息

public:
    ImageConverter()
        : it_(nh_)
    {
        // 订阅 usb_cam 发布的原始图像
        image_sub_ = it_.subscribe("/usb_cam/image_raw", 1, &ImageConverter::imageCb, this);
        
        // 发布像素中心坐标
        center_point_pub_ = nh_.advertise<opencvtest::pixel_point0>("pixel_center_axis", 1000);

        // 创建显示窗口
        namedWindow(WINDOW_ORIGINAL, WINDOW_NORMAL);
        namedWindow(WINDOW_BINARY, WINDOW_NORMAL);
        namedWindow(WINDOW_RESULT, WINDOW_NORMAL);

        // 设置鼠标回调（绑定到 Result Image 窗口）
        setMouseCallback(WINDOW_RESULT, onMouse);

        cout << "=== 物块识别节点已启动 ===" << endl;
        cout << "订阅话题: /usb_cam/image_raw" << endl;
        cout << "发布话题: pixel_center_axis" << endl;
        cout << "识别颜色: 红、橙、黄、绿、蓝、紫" << endl;
        cout << "在 Result Image 窗口点击物块可查看 HSV 值" << endl;
        cout << "============================" << endl;
    }

    ~ImageConverter()
    {
        destroyWindow(WINDOW_ORIGINAL);
        destroyWindow(WINDOW_BINARY);
        destroyWindow(WINDOW_RESULT);
    }

    /**
     * @brief 处理单个颜色的识别
     * @param hsv      HSV图像
     * @param color    颜色配置
     * @param drawmap  绘制画布
     * @param color_id 颜色ID
     */
    void processColor(const Mat& hsv, const ColorConfig& color, Mat& drawmap, int color_id)
    {
        Mat mask, binary;

        // 1. 提取指定颜色范围 -> 二值图
        inRange(hsv, 
                Scalar(color.h_min, color.s_min, color.v_min),
                Scalar(color.h_max, color.s_max, color.v_max),
                mask);

        // 2. 中值滤波去噪
        medianBlur(mask, binary, 25);

        // 3. 查找轮廓
        vector<vector<Point>> contours;
        vector<Vec4i> hierarchy;
        findContours(binary, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE);

        if (contours.empty())
            return;

        // 4. 遍历轮廓，筛选符合面积要求的物块
        for (size_t i = 0; i < contours.size(); i++)
        {
            Rect rect = boundingRect(contours[i]);
            double area = rect.area();

            // 面积阈值：根据相机高度调整（修改为 1000~15000 像素²）
            if (area > 1000 && area < 15000)
            {
                // 计算中心像素坐标
                double center_u = 0.5 * (rect.tl().x + rect.br().x);
                double center_v = 0.5 * (rect.tl().y + rect.br().y);

                // 绘制外接矩形 (红色边框)
                rectangle(drawmap, rect, Scalar(0, 0, 255), 2);

                // 绘制中心点 (蓝色大圆点)
                circle(drawmap, Point2d(center_u, center_v), 8, Scalar(255, 0, 0), -1);

                // 根据颜色ID存储到对应的消息数组
                switch (color_id)
                {
                case 1: // 红色
                    msgs.red_u.push_back(center_u);
                    msgs.red_v.push_back(center_v);
                    putText(drawmap, "Red", Point(rect.x, rect.y - 10), 
                            FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 255), 2);
                    break;
                case 2: // 橙色
                    msgs.orange_u.push_back(center_u);
                    msgs.orange_v.push_back(center_v);
                    putText(drawmap, "Orange", Point(rect.x, rect.y - 10), 
                            FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 165, 255), 2);
                    break;
                case 3: // 黄色
                    msgs.yellow_u.push_back(center_u);
                    msgs.yellow_v.push_back(center_v);
                    putText(drawmap, "Yellow", Point(rect.x, rect.y - 10), 
                            FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 255), 2);
                    break;
                case 4: // 绿色
                    msgs.green_u.push_back(center_u);
                    msgs.green_v.push_back(center_v);
                    putText(drawmap, "Green", Point(rect.x, rect.y - 10), 
                            FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 0), 2);
                    break;
                case 5: // 蓝色
                    msgs.blue_u.push_back(center_u);
                    msgs.blue_v.push_back(center_v);
                    putText(drawmap, "Blue", Point(rect.x, rect.y - 10), 
                            FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 0, 0), 2);
                    break;
                case 6: // 紫色
                    msgs.purple_u.push_back(center_u);
                    msgs.purple_v.push_back(center_v);
                    putText(drawmap, "Purple", Point(rect.x, rect.y - 10), 
                            FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 0, 255), 2);
                    break;
                default:
                    break;
                }
            }
        }
    }

    /**
     * @brief 处理红色（红色跨越0°边界，需要两个区间合并）
     */
    void processRed(const Mat& hsv, Mat& drawmap)
    {
        Mat mask1, mask2, mask_combined;

        // 红色区间1: 0-10
        inRange(hsv, Scalar(0, 43, 46), Scalar(10, 255, 255), mask1);
        // 红色区间2: 156-180
        inRange(hsv, Scalar(156, 43, 46), Scalar(180, 255, 255), mask2);

        // 合并两个红色区间
        bitwise_or(mask1, mask2, mask_combined);

        // 中值滤波
        medianBlur(mask_combined, mask_combined, 25);

        // 查找轮廓
        vector<vector<Point>> contours;
        vector<Vec4i> hierarchy;
        findContours(mask_combined, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE);

        if (contours.empty())
            return;

        for (size_t i = 0; i < contours.size(); i++)
        {
            Rect rect = boundingRect(contours[i]);
            double area = rect.area();

            // 面积阈值：修改为 1000~15000 像素²
            if (area > 1000 && area < 15000)
            {
                double center_u = 0.5 * (rect.tl().x + rect.br().x);
                double center_v = 0.5 * (rect.tl().y + rect.br().y);

                rectangle(drawmap, rect, Scalar(0, 0, 255), 2);
                circle(drawmap, Point2d(center_u, center_v), 8, Scalar(255, 0, 0), -1);

                msgs.red_u.push_back(center_u);
                msgs.red_v.push_back(center_v);
                putText(drawmap, "Red", Point(rect.x, rect.y - 10), 
                        FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 255), 2);
            }
        }
    }

    /**
     * @brief 图像回调函数
     */
    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        Mat original = cv_ptr->image.clone();
        Mat hsv;
        Mat drawmap = original.clone();

        // BGR -> HSV 转换
        cvtColor(original, hsv, COLOR_BGR2HSV);

        // 保存当前帧用于 HSV 取色
        g_hsv_display = original.clone();

        // ---- 识别各颜色物块 ----
        
        // 1. 红色（特殊处理，跨越0°边界）
        processRed(hsv, drawmap);

        // 2. 橙色
        processColor(hsv, orange, drawmap, 2);

        // 3. 黄色
        processColor(hsv, yellow, drawmap, 3);

        // 4. 绿色
        processColor(hsv, green, drawmap, 4);

        // 5. 蓝色
        processColor(hsv, blue, drawmap, 5);

        // 6. 紫色
        processColor(hsv, purple, drawmap, 6);

        // ---- 如果有点击，在图像上显示 HSV 值 ----
        if (g_hsv_selected)
        {
            char text[100];
            sprintf(text, "H:%d S:%d V:%d", g_hsv_val[0], g_hsv_val[1], g_hsv_val[2]);
            putText(drawmap, text, Point(g_hsv_x, g_hsv_y - 10), 
                    FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 255), 2);
            circle(drawmap, Point(g_hsv_x, g_hsv_y), 8, Scalar(0, 255, 255), 2);
            
            printf("========================================\n");
            printf("位置 (%d, %d)\n", g_hsv_x, g_hsv_y);
            printf("BGR: [%d, %d, %d]\n", g_bgr_val[0], g_bgr_val[1], g_bgr_val[2]);
            printf("HSV: [%d, %d, %d]\n", g_hsv_val[0], g_hsv_val[1], g_hsv_val[2]);
            printf("========================================\n");
            
            g_hsv_selected = false;
        }

        // ---- 显示图像 ----
        imshow(WINDOW_ORIGINAL, original);
        imshow(WINDOW_RESULT, drawmap);

        // 显示二值图像（以绿色为例）
        Mat green_binary;
        inRange(hsv, Scalar(green.h_min, green.s_min, green.v_min),
                     Scalar(green.h_max, green.s_max, green.v_max), green_binary);
        imshow(WINDOW_BINARY, green_binary);

        waitKey(1);

        // ---- 发布消息 ----
        msgs.name = "pixel_center_axis";
        center_point_pub_.publish(msgs);

        // 打印调试信息
        printDebugInfo();

        // 清空消息，准备下一帧
        clearMsgs();
    }

    /**
     * @brief 打印调试信息
     */
    void printDebugInfo()
    {
        static int frame_count = 0;
        frame_count++;
        
        if (frame_count % 30 == 0)  // 每30帧打印一次
        {
            cout << "===== 检测结果 =====" << endl;
            cout << "红色: " << msgs.red_u.size() << " 个" << endl;
            cout << "橙色: " << msgs.orange_u.size() << " 个" << endl;
            cout << "黄色: " << msgs.yellow_u.size() << " 个" << endl;
            cout << "绿色: " << msgs.green_u.size() << " 个" << endl;
            cout << "蓝色: " << msgs.blue_u.size() << " 个" << endl;
            cout << "紫色: " << msgs.purple_u.size() << " 个" << endl;
            cout << "==================" << endl;
        }
    }

    /**
     * @brief 清空消息容器
     */
    void clearMsgs()
    {
        msgs.red_u.clear();
        msgs.red_v.clear();
        msgs.orange_u.clear();
        msgs.orange_v.clear();
        msgs.yellow_u.clear();
        msgs.yellow_v.clear();
        msgs.green_u.clear();
        msgs.green_v.clear();
        msgs.blue_u.clear();
        msgs.blue_v.clear();
        msgs.purple_u.clear();
        msgs.purple_v.clear();
    }
};

/********************************** main 函数 ****************************************/

int main(int argc, char** argv)
{
    ros::init(argc, argv, "color_distinguish");
    ImageConverter ic;
    ros::spin();
    return 0;
}
