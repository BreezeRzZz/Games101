#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

#define CONTROL_POINTS_NUM 4

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void* userdata)
{
	if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < CONTROL_POINTS_NUM)
	{
		std::cout << "Left button of the mouse is clicked - position (" << x << ", "
			<< y << ")" << '\n';
		control_points.emplace_back(x, y);
	}
}

void antiAliasing(cv::Point2f point, cv::Mat& window, int c)
{
	int x = point.x;
	int y = point.y;
	float maxDis = std::sqrt(1.125);
	float color1 = 255 * (maxDis - std::sqrt(pow(point.x - x - 0.25f, 2) + pow(point.y - y - 0.25f, 2))) / maxDis;
	float color2 = 255 * (maxDis - std::sqrt(pow(point.x - x - 0.75f, 2) + pow(point.y - y - 0.25f, 2))) / maxDis;
	float color3 = 255 * (maxDis - std::sqrt(pow(point.x - x - 0.25f, 2) + pow(point.y - y - 0.75f, 2))) / maxDis;
	float color4 = 255 * (maxDis - std::sqrt(pow(point.x - x - 0.75f, 2) + pow(point.y - y - 0.75f, 2))) / maxDis;
	int color = (color1 + color2 + color3 + color4) / 4.f;
	window.at<cv::Vec3b>(point.y, point.x)[c] = std::min(255, color + window.at<cv::Vec3b>(point.y, point.x)[c]);
}

int comb(int m, int n)
{
	if (m < n)
		return 0;
	if (m == n)
		return 1;
	if (n == 0)
		return 1;
	if (n == 1)
		return m;

	return comb(m - 1, n) + comb(m - 1, n - 1);
}

void naive_bezier(const std::vector<cv::Point2f>& points, cv::Mat& window)
{
	cv::Point2f point;
	int n = points.size();

	for (double t = 0.0; t <= 1.0; t += 0.001)
	{
		for (int i = 0; i < n; ++i)
		{
			point += comb(n - 1, i) * pow(1 - t, n - 1 - i) * pow(t, i) * points[i];
		}
		antiAliasing(point, window, 1);
		//window.at<cv::Vec3b>(point.y, point.x)[1] = 255;
		point = { 0,0 };
	}
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f>& control_points, float t)
{
	// TODO: Implement de Casteljau's algorithm
	if (control_points.size() == 1)
		return control_points[0];

	std::vector<cv::Point2f> temp;
	for (int i = 0; i < control_points.size() - 1; ++i)
	{
		temp.push_back(control_points[i] * t + control_points[i + 1] * (1 - t));
	}
	return recursive_bezier(temp, t);
}

void bezier(const std::vector<cv::Point2f>& control_points, cv::Mat& window)
{
	// TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
	// recursive Bezier algorithm.
	float dt = 0.001;
	float t = 0;
	for (t = 0; t <= 1; t += dt)
	{
		auto point = recursive_bezier(control_points, t);

		antiAliasing(point, window, 2);
		//window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
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
		for (auto& point : control_points)
		{
			cv::circle(window, point, 3, { 255, 255, 255 }, 3);
		}

		if (control_points.size() == CONTROL_POINTS_NUM)
		{
			naive_bezier(control_points, window);
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
