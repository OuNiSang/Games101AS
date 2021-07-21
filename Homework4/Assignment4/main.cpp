#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;
bool isAA;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
        if(isAA) window.at<cv::Vec3b>(point.y, point.x)[1] = 255;   //为了达到更好的AA的效果
    }
}

inline cv::Point2f LinearLerp(const cv::Point2f &_pointA, const cv::Point2f &_pointB, float t){return _pointA + (_pointB - _pointA) * t;}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // TODO: Implement de Casteljau's algorithm
    /* 如果序列只包含一个点，则返回该点并终止 */
    if (control_points.size() <= 1) return control_points[0];


    //得到的分割点作为新的控制点序列
    std::vector<cv::Point2f> newControlOrder;

    for (int i = 0; i < control_points.size() -1; i++)
    {
        /* 用 t : (1 − t) 的比例细分每个线段，并找到该分割点*/
        newControlOrder.emplace_back(LinearLerp(control_points[i], control_points[i + 1], t));
    }
    
    //使用新的控制点序列并转到步骤 1
    return recursive_bezier(newControlOrder, t);
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.

    //init param
    double _iterateStep = 0.0001;   /* 它会使 t 在 0 到 1 的范围内进行迭代，并在每次迭代中使 t 增加一个微小值 */ 

    for (double _t = 0.0; _t <= 1.0; _t += _iterateStep)
    {
        /*对于每个需要计算的 t，将调用另一个函数 recursive_bezier，然后该函数将返回在 Bézier 曲线上t处的点*/
        cv::Point2f _drawPoint = recursive_bezier(control_points, _t);
        window.at<cv::Vec3b>(_drawPoint.y, _drawPoint.x)[1] = 255; //ref: https://www.bilibili.com/read/cv12149543?from=articleDetail
        if (isAA)
        {
            /*实现对 Bézier 曲线的反走样。(对于一个曲线上的点，不只把它对应于一个像素，你需要根据到像素中心的距离来考虑与它相邻的像素的颜色。) */
            //实现过程ref: https://blog.csdn.net/miyu1994/article/details/107002644
            float _xMin = std::floor(_drawPoint.x);
            float _yMin = std::floor(_drawPoint.y);
            float _xFlag = _drawPoint.x - _xMin < 0.5f ? -1.0f : 1.0f;     //这里我的理解是处理边界情况
            float _yFlag = _drawPoint.y - _yMin < 0.5f ? -1.0f : 1.0f;

            cv::Point2f p00,p01,p10,p11;                                    //采样最近的4个点
            p00 = cv::Point2f(_xMin + 0.5f, _yMin + 0.5f);
            p01 = cv::Point2f(_xMin + _xFlag + 0.5f, _yMin + 0.5f);
            p10 = cv::Point2f(_xMin + 0.5f, _yMin + _yFlag + 0.5f);
            p11 = cv::Point2f(_xMin +_xFlag + 0.5f, _yMin + _yFlag + 0.5f);

            std::vector<cv::Point2f> _tempVec;                              //采样点temp容器
            _tempVec.emplace_back(p01);
            _tempVec.emplace_back(p10);
            _tempVec.emplace_back(p11);

            cv::Point2f _disVec = p00 - _drawPoint;                         // 计算最近的坐标点与采样点距离
            float _distancePrime = std::sqrt(std::pow(_disVec.x,2) + std::pow(_disVec.y, 2));

            for(cv::Point2f _point : _tempVec)                              //Shading
            {
                cv::Point2f _dVec = _point - _drawPoint;
                float _distance = std::sqrt(std::pow(_dVec.x,2) + std::pow(_dVec.y, 2));
                float _percentage =  255 * _distance / _distancePrime;             // 根据距离计算边缘点影响percentage
                
                cv::Vec3b _pointColor = (cv::Vec3b(0, _percentage, _percentage)); //配合naive-bezier的效果，从而加上了对红色通道的控制
                window.at<cv::Vec3b>(_point.y, _point.x) = _pointColor;
            }
        }
    }
}

int main() 
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == 4) 
        {
            isAA = false;
            naive_bezier(control_points, window);
            bezier(control_points, window);

            cv::imshow("Bezier Curve_NoAA", window);
            cv::imwrite("my_bezier_curve_noAA.png", window);

            isAA = true;
            bezier(control_points, window);
            naive_bezier(control_points, window);
            cv::imshow("Bezier Curve_AA", window);
            cv::imwrite("my_bezier_curve_AA.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

return 0;
}
