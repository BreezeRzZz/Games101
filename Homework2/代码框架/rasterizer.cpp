// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>

#define N 2

rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f>& positions)
{
	auto id = get_next_id();
	pos_buf.emplace(id, positions);

	return { id };
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i>& indices)
{
	auto id = get_next_id();
	ind_buf.emplace(id, indices);

	return { id };
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f>& cols)
{
	auto id = get_next_id();
	col_buf.emplace(id, cols);

	return { id };
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
	return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

//计算叉积的z值
float crossProductZ(const Vector2f u, const Vector2f v)
{
	return  u.x() * v.y() - v.x() * u.y();
}

static bool insideTriangle(float x, float y, const Vector4f* _v)
{
	// TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
	//思路：利用叉积判断即可
	float z1 = crossProductZ(Vector2f(_v[1].x() - _v[0].x(), _v[1].y() - _v[0].y()), Vector2f(x - _v[0].x(), y - _v[0].y()));
	float z2 = crossProductZ(Vector2f(_v[2].x() - _v[1].x(), _v[2].y() - _v[1].y()), Vector2f(x - _v[1].x(), y - _v[1].y()));
	float z3 = crossProductZ(Vector2f(_v[0].x() - _v[2].x(), _v[0].y() - _v[2].y()), Vector2f(x - _v[2].x(), y - _v[2].y()));

	if ((z1 > 0 && z2 > 0 && z3 > 0) || (z1 < 0 && z2 < 0 && z3 < 0))
		return true;
	return false;
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector4f* v)
{
	float c1 = (x * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * y + v[1].x() * v[2].y() - v[2].x() * v[1].y()) / (v[0].x() * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * v[0].y() + v[1].x() * v[2].y() - v[2].x() * v[1].y());
	float c2 = (x * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * y + v[2].x() * v[0].y() - v[0].x() * v[2].y()) / (v[1].x() * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * v[1].y() + v[2].x() * v[0].y() - v[0].x() * v[2].y());
	float c3 = (x * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * y + v[0].x() * v[1].y() - v[1].x() * v[0].y()) / (v[2].x() * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * v[2].y() + v[0].x() * v[1].y() - v[1].x() * v[0].y());
	return { c1,c2,c3 };
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
	auto& buf = pos_buf[pos_buffer.pos_id];
	auto& ind = ind_buf[ind_buffer.ind_id];
	auto& col = col_buf[col_buffer.col_id];

	float f1 = (50 - 0.1) / 2.0;
	float f2 = (50 + 0.1) / 2.0;

	Eigen::Matrix4f mvp = projection * view * model;
	for (auto& i : ind)
	{
		Triangle t;
		Eigen::Vector4f v[] = {
				mvp * to_vec4(buf[i[0]], 1.0f),
				mvp * to_vec4(buf[i[1]], 1.0f),
				mvp * to_vec4(buf[i[2]], 1.0f)
		};
		//Homogeneous division
		for (auto& vec : v) {
			vec.x() /= vec.w();
			vec.y() /= vec.w();
			vec.z() /= vec.w();
		}
		//Viewport transformation
		for (auto& vert : v)
		{
			vert.x() = 0.5 * width * (vert.x() + 1.0);
			vert.y() = 0.5 * height * (vert.y() + 1.0);
			vert.z() = -vert.z() * f1 + f2;
		}

		for (int i = 0; i < 3; ++i)
		{
			t.setVertex(i, v[i]);
		}

		auto col_x = col[i[0]];
		auto col_y = col[i[1]];
		auto col_z = col[i[2]];

		t.setColor(0, col_x[0], col_x[1], col_x[2]);
		t.setColor(1, col_y[0], col_y[1], col_y[2]);
		t.setColor(2, col_z[0], col_z[1], col_z[2]);

		rasterize_triangle(t);
	}
}


//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
	//auto v = t.toVector4();

	// TODO : Find out the bounding box of current triangle.
	// iterate through the pixel and find if the current pixel is inside the triangle
	int L = std::min(std::min(t.v[0].x(), t.v[1].x()), t.v[2].x());
	int R = ceil(std::max(std::max(t.v[0].x(), t.v[1].x()), t.v[2].x()));
	int B = std::min(std::min(t.v[0].y(), t.v[1].y()), t.v[2].y());
	int T = ceil(std::max(std::max(t.v[0].y(), t.v[1].y()), t.v[2].y()));

	//以下为每个像素采一次样的结果
	/*for (int x = L; x <= R; ++x)
	{
		for (int y = B; y <= T; ++y)
		{
			//检测像素的中心坐标是否在三角形内
			if (insideTriangle(x + 0.5, y + 0.5, t.v))
			{
				// If so, use the following code to get the interpolated z value.
				// 获取插值深度值
				auto [alpha, beta, gamma] = computeBarycentric2D(x + 0.5, y + 0.5, t.v);
				float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
				float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
				z_interpolated *= w_reciprocal;

				//与深度缓冲区的值比较
				int index = get_index(x, y);
				if (z_interpolated < depth_buf[index])
				{
					// TODO : set the current pixel (use the set_pixel function) to the color of
					// the triangle (use getColor function) if it should be painted.
					depth_buf[index] = z_interpolated;
					set_pixel(Vector3f(x, y, 0), t.getColor());
				}
			}
		}
	}*/

	//以下为使用MSAA的结果
	//N为划分数,dx,dy为差分,2d为2个采样点之间的横向、纵向距离
	//mindepth维护每个像素内的最小z值，最后与zbuffer的内容比较
	float d = 1.0 / (2 * N);
	std::vector<float> dx(N), dy(N);
	for (int i = 0; i < N; ++i)
	{
		dx[i] = dy[i] = (2 * i + 1) * d;
	}

	for (int x = L; x <= R; ++x)
	{
		for (int y = B; y <= T; ++y)
		{
			int index = get_index(x, y);
			//子采样点下标
			int subIndex = 0;
			for (int i = 0; i < N; ++i)
			{
				for (int j = 0; j < N; ++j)
				{
					if (insideTriangle(x + dx[i], y + dy[j], t.v))
					{
						auto [alpha, beta, gamma] = computeBarycentric2D(x + dx[i], y + dy[j], t.v);
						float w_reciprocal = 1.0 / (alpha / t.v[0].w() + beta / t.v[1].w() + gamma / t.v[2].w());
						float z_interpolated = alpha * t.v[0].z() / t.v[0].w() + beta * t.v[1].z() / t.v[1].w() + gamma * t.v[2].z() / t.v[2].w();
						z_interpolated *= w_reciprocal;

						if (z_interpolated < depth_buf[index][subIndex])
						{
							depth_buf[index][subIndex] = z_interpolated;
							color_buf[index][subIndex] = t.getColor();
						}
					}
					++subIndex;
				}
			}

			//将子采样点的颜色取平均
			subIndex = 0;
			Vector3f color = { 0,0,0 };
			for (int i = 0; i < N * N; ++i)
			{
				color += color_buf[index][subIndex];
				++subIndex;
			}
			color /= (N * N);
			set_pixel(Vector3f(x, y, 0), color);
		}
	}
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
	model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
	view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
	projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
	if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
	{
		std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{ 0, 0, 0 });
		for (int i = 0; i < width * height; ++i)
			std::fill(color_buf[i].begin(), color_buf[i].end(), Eigen::Vector3f{ 0, 0, 0 });
	}
	if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
	{
		for (int i = 0; i < width * height; ++i)
			std::fill(depth_buf[i].begin(), depth_buf[i].end(), std::numeric_limits<float>::infinity());
	}
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
	frame_buf.resize(w * h);
	color_buf.resize(w * h);
	depth_buf.resize(w * h);
	//将深度缓存和颜色缓存扩大
	for (int i = 0; i < w * h; ++i)
	{
		color_buf[i].resize(N * N);
		depth_buf[i].resize(N * N);
	}
}

int rst::rasterizer::get_index(int x, int y)
{
	return (height - 1 - y) * width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
	//old index: auto ind = point.y() + point.x() * width;
	auto ind = (height - 1 - point.y()) * width + point.x();
	frame_buf[ind] = color;

}

// clang-format on