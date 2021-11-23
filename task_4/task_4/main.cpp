#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

//使用数学表达式（多项式）的实现方法。只能应用于4个控制点的情况。
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

		cout << point << endl;
        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // TODO: Implement de Casteljau's algorithm

	int n = control_points.size();
	if (1 == n)
		return control_points[0];
	std::vector<cv::Point2f> new_control_points;
	cv::Point2f a, b, c;
	for (int i = 0; i < n-1; i++) {
		a = control_points[i];
		b = control_points[i+1];
		c = b - a;
		new_control_points.emplace_back(a + c * t);
	}

    return recursive_bezier(new_control_points, t);
}

//使用de Casteljau算法的实现方法。可应用于任意数量控制点的情况。
void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
	
	cout << "function bezier begins" << endl;
	unsigned short number[700][700];
	for (int i = 0; i < 700; i++)
		for (int j = 0; j < 700; j++)
			number[i][j] = 0;
		
	float m, n;
	int m_, n_;
	for (float t = 0.; t <= 1.; t += 0.001) {
		auto point = recursive_bezier(control_points, t);
		
		m = point.y;
		n = point.x;
		m_ = (int)m;
		n_ = (int)n;
		number[m_][n_] += 1;
		float ratio = 1 - sqrt(pow((m - m_ - 0.5), 2) + pow((n - n_ - 0.5), 2)) / 1.;
		ratio = ratio >= 0. ? ratio : 0.;
		auto color = window.at<cv::Vec3b>(point.y, point.x)[1];
		unsigned short num = number[m_][n_];
		color = (color * (num-1) + 255 * ratio)/ num;
		cout << ratio << endl;
		window.at<cv::Vec3b>(point.y, point.x)[1] = color;
		//把下一行行解注释的话，就是原来的做法（曲线上一个点直接对应一个像素，颜色是纯绿色），否则就是改进后的做法（颜色经过加权平均）。
		//window.at<cv::Vec3b>(point.y, point.x)[1] = 255; 
	}
	cout << "function bezier ends" << endl;
}

int main() 
{
	for (int i = 0; i < 4; i++)
		control_points.emplace_back(100 + 100 * i, 100 + i *20);


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
            //naive_bezier(control_points, window);
            bezier(control_points, window);
			
            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

return 0;
}
