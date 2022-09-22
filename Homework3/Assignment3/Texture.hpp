//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture {
private:
	cv::Mat image_data;

public:
	Texture(const std::string& name)
	{
		image_data = cv::imread(name);
		cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
		width = image_data.cols;
		height = image_data.rows;
	}

	int width, height;

	Eigen::Vector3f getColor(float u, float v)
	{
		if (u < 0 || u > 1 || v < 0 || v > 1)
			return Eigen::Vector3f(0, 0, 0);

		//auto u_img = u * width;
		//auto v_img = (1 - v) * height;
		//原计算方法在u,v=1时会越界，应修正如下:
		auto u_img = u * (width - 1);
		auto v_img = (1 - v) * (height - 1);

		//注意这里u,v代入是相反的，在双线性插值中必须按这个顺序
		auto color = image_data.at<cv::Vec3b>(v_img, u_img);
		return Eigen::Vector3f(color[0], color[1], color[2]);
	}

	//给定图片坐标获取颜色
	Eigen::Vector3f get(float x, float y)
	{
		auto color = image_data.at<cv::Vec3b>(x, y);
		return Eigen::Vector3f(color[0], color[1], color[2]);
	}

	//双线性插值
	Eigen::Vector3f getColorBilinear(float u, float v)
	{
		if (u < 0 || u > 1 || v < 0 || v > 1)
			return Eigen::Vector3f(0, 0, 0);

		auto y = u * (width - 1);
		auto x = (1 - v) * (height - 1);

		int x0 = x, x1 = std::min(x0 + 1, height - 1);
		int y0 = y, y1 = std::min(y0 + 1, width - 1);

		//双线性插值修正 
		auto u01 = get(x0, y1);
		auto u11 = get(x1, y1);
		auto u00 = get(x0, y0);
		auto u10 = get(x1, y0);

		float s = x - x0;
		float t = y - y0;

		Eigen::Vector3f u0 = u00 + s * (u10 - u00);
		Eigen::Vector3f u1 = u01 + s * (u11 - u01);
		Eigen::Vector3f color = u0 + t * (u1 - u0);

		return color;
	}

};
#endif //RASTERIZER_TEXTURE_H
