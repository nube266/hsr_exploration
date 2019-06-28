#include "ros/ros.h"
#include "std_msgs/String.h"
#include "color_histogram_search/BoundingBox.h"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <ros/package.h>

#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include <Eigen/StdVector>
#include <sstream>

static const std::string OPENCV_WINDOW = "Image window";

#define USE_EIGEN
#define USE_PARTHISTO
#define USE_INTEGRATION_IMAGE
struct Region_answer
{
	int x, y; //Rect始点
	int k;	//一辺のサイズ
	double s; //類似度
};

struct R
{
	int xmin, ymin, xmax, ymax;
	int kmin, kmax;
	double s;
};

#ifdef USE_PARTHISTO
int gcd(int a, int b)
{
	if (a % b == 0)
	{
		return (b);
	}
	else
	{
		return (gcd(b, a % b));
	}
}
#endif

class ColorHistogramSearch
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	ColorHistogramSearch(const int image_width, const int image_height,
						 const int shift, const int init_w,
						 const int max_w, const int dw,
						 const double init_th, const double ex_th,
						 const double v_th, const double s_th,
						 const bool is_active_search);
	~ColorHistogramSearch();

	int search_hist(const cv::Mat search_img,
					const cv::Mat query_img,
					std::vector<struct Region_answer> &ans);
	void calc_intersect(const std::vector<struct Region_answer> &ans,
						double ratio, std::vector<struct R> &ans_ret);
#ifdef USE_EIGEN
	Eigen::VectorXd make_query_histogram(const cv::Mat query_img);
	int search_hist(const cv::Mat search_img,
					const Eigen::VectorXd &hist_query,
					std::vector<struct Region_answer> &ans);
#else
	void make_query_histogram(const cv::Mat query_img, double hist[26]);
	int search_hist(const cv::Mat search_img,
					const double hist_query[26],
					std::vector<struct Region_answer> &ans);
#endif
	cv::Mat make_mask(std::vector<struct Region_answer> ans,
					  double ratio);

private:
	int get_index(int R, int G, int B);
#ifdef USE_EIGEN
	Eigen::VectorXd make_h_histogram(const std::vector<std::vector<
														   Eigen::VectorXd,
														   Eigen::aligned_allocator<
															   Eigen::VectorXd>>,
													   Eigen::aligned_allocator<
														   Eigen::VectorXd>> &h_idx,
									 int x,
									 int y, int w);
	double matching_ratio(const Eigen::VectorXd &hist1,
						  const Eigen::VectorXd &hist2);
#else
	void make_h_histogram(double hist[26], int x, int y, int w, int h);
	double matching_ratio(const double hist_a[26],
						  const double hist_b[26]);
#endif
	void prepare_memory(int image_width, int image_height);
	void free_memory();

private:
	int shift;
	int init_w;
	int max_w;
	int dw;
	int r;
#ifndef USE_EIGEN
	int **h_idx;
#endif
	double ***e;
	int image_width;
	int image_height;
	const static int maxval = 255;
	int index_map[256][256][256];
	double v_th;
	double s_th;
	double init_th;
	double ex_th;
	const static int num_bins = 26;
	bool is_active_search;
#ifdef USE_EIGEN
#ifdef USE_INTEGRATION_IMAGE
	std::vector<std::vector<Eigen::VectorXd,
							Eigen::aligned_allocator<Eigen::VectorXd>>,
				Eigen::aligned_allocator<Eigen::VectorXd>>
		h_idx;
	std::vector<std::vector<Eigen::VectorXd,
							Eigen::aligned_allocator<Eigen::VectorXd>>,
				Eigen::aligned_allocator<Eigen::VectorXd>>
		I;
	std::vector<std::vector<Eigen::VectorXd,
							Eigen::aligned_allocator<Eigen::VectorXd>>,
				Eigen::aligned_allocator<Eigen::VectorXd>>
		Id;
#else
	std::vector<std::vector<Eigen::VectorXd,
							Eigen::aligned_allocator<Eigen::VectorXd>>,
				Eigen::aligned_allocator<Eigen::VectorXd>>
		h_idx;
#endif
#endif
};

ColorHistogramSearch::ColorHistogramSearch(const int image_width,
										   const int image_height,
										   const int shift,
										   const int init_w,
										   const int max_w, const int dw,
										   const double init_th,
										   const double ex_th,
										   const double v_th,
										   const double s_th,
										   const bool is_active_search)
{

#ifdef USE_EIGEN

#ifdef USE_PARTHISTO
	this->r = gcd(gcd(gcd(init_w, max_w), gcd(dw, shift)),
				  gcd(image_width, image_height));
#else
	this->r = 1;
#endif

	this->shift = shift / r;
	this->init_w = init_w / r;
	this->max_w = max_w / r;
	this->dw = dw / r;
#else
	this->r = 1;
	this->shift = shift;
	this->init_w = init_w;
	this->max_w = max_w;
	this->dw = dw;
#endif
	this->init_th = init_th;
	this->ex_th = ex_th;
	this->v_th = v_th;
	this->s_th = s_th;
	this->is_active_search = is_active_search;
	if (image_width > 0 && image_height > 0)
	{
		prepare_memory(image_width, image_height);
	}
	else
	{
		this->image_width = 0;
		this->image_height = 0;
	}
	for (int i = 0; i < 256; i++)
		for (int j = 0; j < 256; j++)
			for (int k = 0; k < 256; k++)
				index_map[i][j][k] = -1;
}

ColorHistogramSearch::~ColorHistogramSearch()
{
	free_memory();
}

void ColorHistogramSearch::prepare_memory(int image_width,
										  int image_height)
{
#ifdef USE_EIGEN
	this->image_width = image_width / r;
	this->image_height = image_height / r;
#else
	this->image_width = image_width;
	this->image_height = image_height;
#endif

#ifndef USE_EIGEN
	h_idx = new int *[image_height];
	for (int i = 0; i < image_height; i++)
	{
		h_idx[i] = new int[image_width];
	}
#endif
	int size1 = int((max_w - init_w) / dw) + 1;
	int size2, size3;
	e = new double **[size1];
	for (int i = 0; i < size1; i++)
	{
		size2 = int(image_height - (init_w + dw * i)) / shift + 1;
		e[i] = new double *[size2];
		for (int j = 0; j < size2; j++)
		{
			size3 = (image_width - (init_w + dw * i)) / shift + 1;
			e[i][j] = new double[size3];
		}
	}

#ifdef USE_EIGEN
#ifdef USE_INTEGRATION_IMAGE
	for (int y = 0; y < this->image_height; y++)
	{
		std::vector<Eigen::VectorXd,
					Eigen::aligned_allocator<Eigen::VectorXd>>
			work;
		std::vector<Eigen::VectorXd,
					Eigen::aligned_allocator<Eigen::VectorXd>>
			work_Id;
		std::vector<Eigen::VectorXd,
					Eigen::aligned_allocator<Eigen::VectorXd>>
			work_h_idx;
		for (int x = 0; x < this->image_width; x++)
		{
			work.push_back(Eigen::VectorXd::Zero(num_bins));
			work_Id.push_back(Eigen::VectorXd::Zero(num_bins));
			work_h_idx.push_back(Eigen::VectorXd::Zero(num_bins));
		}
		I.push_back(work);
		Id.push_back(work_Id);
		h_idx.push_back(work_h_idx);
	}
#else
	for (int y = 0; y < this->image_height; y++)
	{
		std::vector<Eigen::VectorXd,
					Eigen::aligned_allocator<Eigen::VectorXd>>
			work_h_idx;
		for (int x = 0; x < this->image_width; x++)
		{
			work_h_idx.push_back(Eigen::VectorXd::Zero(num_bins));
		}
		h_idx.push_back(work_h_idx);
	}
#endif
#endif
}

void ColorHistogramSearch::free_memory()
{
#ifndef USE_EIGEN
	for (int i = 0; i < this->image_height; i++)
	{
		delete h_idx[i];
	}
	delete h_idx;
#endif

#ifdef USE_EIGEN
#ifdef USE_INTEGRATION_IMAGE
	for (int y = 0; y < this->image_height; y++)
	{
		I[y].clear();
		Id[y].clear();
		h_idx[y].clear();
	}
	I.clear();
	Id.clear();
	h_idx.clear();
#else
	for (int y = 0; y < this->image_height; y++)
	{
		h_idx[y].clear();
	}
	h_idx.clear();
#endif
#endif

	for (int i = 0; i < (this->max_w - this->init_w) / dw + 1; i++)
	{
		for (int j = 0;
			 j <
			 (this->image_height - (this->init_w + dw * i)) / this->shift +
				 1;
			 j++)
		{
			delete e[i][j];
		}
		delete e[i];
	}
	delete e;
	this->image_width = 0;
	this->image_height = 0;
}

#ifdef USE_EIGEN
Eigen::VectorXd ColorHistogramSearch::
	make_query_histogram(const cv::Mat query_img)
{
	Eigen::VectorXd hist = Eigen::VectorXd::Zero(num_bins);
	//int count = 0;

	for (int i = 0; i < query_img.size().height; i++)
	{
		for (int j = 0; j < query_img.size().width; j++)
		{
			int r = query_img.at<cv::Vec3b>(i, j)[2];
			int g = query_img.at<cv::Vec3b>(i, j)[1];
			int b = query_img.at<cv::Vec3b>(i, j)[0];
			int index = get_index(r, g, b);
			hist(index) = hist(index) + 1.0;
		}
	}

	double sum = hist.sum();
	hist = hist / sum;
	return hist;
}
#else
void ColorHistogramSearch::make_query_histogram(const cv::Mat query_img,
												double hist[26])
{

	int i, j;
	//double hue, v, s;
	//int max, min;
	int r, g, b;
	double sum;

	for (i = 0; i < 26; i++)
	{
		hist[i] = 0.;
	}

	for (i = 0; i < query_img.size().height; i++)
	{
		for (j = 0; j < query_img.size().width; j++)
		{
			r = query_img.at<cv::Vec3b>(i, j)[2];
			g = query_img.at<cv::Vec3b>(i, j)[1];
			b = query_img.at<cv::Vec3b>(i, j)[0];
			int index = get_index(r, g, b);
			hist[index] += 1.0;
		}
	}

	sum = 0.;
	for (i = 0; i < 26; i++)
	{
		sum += hist[i];
	}
	for (i = 0; i < 26; i++)
	{
		hist[i] /= sum;
	}
}
#endif

int ColorHistogramSearch::get_index(int R, int G, int B)
{
	/*
    if (index_map[R][G][B] != -1) {
	return index_map[R][G][B];
    }
*/
	double hue, v, s;
	int max, min;

	max = min = R;
	if (max < G)
		max = G;
	if (max < B)
		max = B;
	if (min > G)
		min = G;
	if (min > B)
		min = B;

	v = (double)max / (double)maxval;

	//s=((double)(max-min))/((double)sertch_maxval);
	s = ((double)(max - min)) / ((double)max);
	if (max == R)
	{
		hue = 60 * ((double)(G - B)) / ((double)(max - min)) + 0.;
	}
	else if (max == G)
	{
		hue = 60 * ((double)(B - R)) / ((double)(max - min)) + 120.;
	}
	else
	{
		hue = 60 * ((double)(R - G)) / ((double)(max - min)) + 240.;
	}

	while (hue >= 360.)
	{
		hue -= 360.;
	}
	while (hue < 0.)
	{
		hue += 360.;
	}

	int ret = 0;
	if (v < v_th)
	{
		ret = 0;
	}
	else if (s < s_th)
	{
		ret = 1;
	}
	else if (s < (s_th + (1. - s_th) / 2.))
	{
		ret = 2 + (int)(hue / 30.);
	}
	else
	{
		ret = 2 + 12 + (int)(hue / 30.);
	}
	index_map[R][G][B] = ret;
	return ret;
}

#ifdef USE_EIGEN
Eigen::VectorXd ColorHistogramSearch::make_h_histogram(const std::vector<
														   std::vector<
															   Eigen::VectorXd,
															   Eigen::
																   aligned_allocator<
																	   Eigen::VectorXd>>,
														   Eigen::aligned_allocator<Eigen::VectorXd>> &h_idx,
													   int x,
													   int y, int w)
{
	Eigen::VectorXd hist = Eigen::VectorXd::Zero(num_bins);
#ifdef USE_INTEGRATION_IMAGE
	hist = h_idx[y + w - 1][x + w - 1];
	if (y > 0)
	{
		hist = hist - h_idx[y - 1][x + w - 1];
	}
	if (x > 0)
	{
		hist = hist - h_idx[y + w - 1][x - 1];
	}
	if (x > 0 && y > 0)
	{
		hist = hist + h_idx[y - 1][x - 1];
	}
	return hist / hist.sum();
#else
	for (int i = 0; i < w; i++)
	{
		for (int j = 0; j < w; j++)
		{
			auto work = h_idx[i + y][j + x];
			hist = hist + work;
		}
	}
	return hist / (double)(w * w);
#endif
}
#else

void ColorHistogramSearch::make_h_histogram(double hist[26], int x, int y,
											int w, int h)
{

	int i, j;
	for (i = 0; i < 26; i++)
	{
		hist[i] = 0.;
	}

	for (i = y; i < y + h; i++)
	{
		for (j = x; j < x + w; j++)
		{
			hist[h_idx[i][j]] += 1.;
		}
	}
	for (i = 0; i < 26; i++)
	{
		hist[i] /= (double)(h * w);
	}
}
#endif

#ifdef USE_EIGEN
double ColorHistogramSearch::matching_ratio(const Eigen::VectorXd &hist1,
											const Eigen::VectorXd &hist2)
{
	auto ret = (hist1.array().min(hist2.array())).sum();
	return ret;
}

#else
double ColorHistogramSearch::matching_ratio(const double hist_a[26],
											const double hist_b[26])
{
	double sum = 0.;
	for (int i = 0; i < 26; i++)
	{
		sum += (hist_a[i] < hist_b[i] ? hist_a[i] : hist_b[i]);
	}
	return sum;
}
#endif

void ColorHistogramSearch::calc_intersect(const std::vector<
											  struct Region_answer> &ans,
										  double ratio,
										  std::vector<struct R> &ans_ret)
{

	struct R r[(int)ans.size()];
	int exist[(int)ans.size()];
	//int inter[(int) ans.size()];
	for (int i = 0; i < (int)ans.size(); i++)
	{
		exist[i] = 0;
	}

	for (int i = 0; i < (int)ans.size(); i++)
	{
		//for (int j = 0; j < i; j++)
		//   inter[j] = 0;

		/*インターセクト計算 */

		int xmin = ans[i].x;
		int ymin = ans[i].y;
		int xmax = xmin + ans[i].k;
		int ymax = ymin + (int)(ans[i].k * ratio);
		int kmin = ans[i].k;
		int kmax = ans[i].k;
		double s = ans[i].s;

		for (int j = 0; j < i; j++)
		{
			if (exist[j] == 0)
				continue;
			if (((r[j].xmin <= ans[i].x && r[j].xmax >= ans[i].x) || (ans[i].x <= r[j].xmin && ans[i].x + ans[i].k >= r[j].xmin)) && ((r[j].ymin <= ans[i].y && r[j].ymax >= ans[i].y) || (ans[i].y <= r[j].ymin && ans[i].y + ans[i].k >= r[j].ymin)))
			{
				xmin = xmin < r[j].xmin ? xmin : r[j].xmin;
				ymin = ymin < r[j].ymin ? ymin : r[j].ymin;
				xmax = xmax > r[j].xmax ? xmax : r[j].xmax;
				ymax = ymax > r[j].ymax ? ymax : r[j].ymax;
				kmin = kmin < r[j].kmin ? kmin : r[j].kmin;
				kmax = kmax > r[j].kmax ? kmax : r[j].kmax;
				s = s > r[j].s ? s : r[j].s;
				exist[j] = 0;
			}
		}
		exist[i] = 1;
		r[i].xmin = xmin;
		r[i].xmax = xmax;
		r[i].ymin = ymin;
		r[i].ymax = ymax;
		r[i].kmin = kmin;
		r[i].kmax = kmax;
		r[i].s = s;
	}
	for (int i = 0; i < (int)ans.size(); i++)
	{
		if (exist[i] == 1)
		{
			struct R dat;
			dat.xmin = r[i].xmin * this->r;
			dat.ymin = r[i].ymin * this->r;
			dat.xmax = r[i].xmax * this->r;
			dat.ymax = r[i].ymax * this->r;
			dat.kmin = r[i].kmin * this->r;
			dat.kmax = r[i].kmax * this->r;
			dat.s = r[i].s;
			ans_ret.push_back(dat);
		}
	}
}

cv::Mat ColorHistogramSearch::make_mask(std::vector<
											struct Region_answer>
											ans,
										double ratio)
{

	cv::Mat mask(cv::Size(image_width * r, image_height * r), CV_8UC1);

	for (int i = 0; i < image_height * r; i++)
	{
		for (int j = 0; j < image_width * r; j++)
		{
			mask.at<uchar>(i, j) = 0;
		}
	}

	for (int i = 0; i < (int)ans.size(); i++)
	{
		for (int j = 0; j < (int)((double)ans[i].k * ratio * r); j++)
		{
			for (int k = 0; k < (int)(ans[i].k * r); k++)
			{
				//mask[ans[i].y + j][ans[i].x + k] = 1;
				mask.at<uchar>(ans[i].y * r + j, ans[i].x * r + k) =
					255;
			}
		}
	}
	return mask.clone();
}

int ColorHistogramSearch::search_hist(const cv::Mat search_img,
									  const cv::Mat query_img,
									  std::vector<
										  struct Region_answer> &ans)
{
#ifdef USE_EIGEN
	Eigen::VectorXd hist_query = make_query_histogram(query_img);
#else
	double hist_query[26];
	make_query_histogram(query_img, hist_query);
#endif
	return search_hist(search_img, hist_query, ans);
}

#ifdef USE_EIGEN

int ColorHistogramSearch::search_hist(const cv::Mat search_img,
									  const Eigen::VectorXd &hist_query,
									  std::vector<
										  struct Region_answer> &ans)
{
	if (search_img.size().width != image_width * r || search_img.size().height != image_height * r)
	{
		free_memory();
		prepare_memory(search_img.size().width, search_img.size().height);
	}

	for (int i = 0; i < (max_w - init_w) / (dw) + 1; i++)
	{
		for (int j = 0;
			 j < (image_height - ((init_w) + dw * i)) / (shift) + 1; j++)
		{
			for (int k = 0;
				 k < (image_width - ((init_w) + dw * i)) / (shift) + 1;
				 k++)
			{
				e[i][j][k] = 1.0;
			}
		}
	}

	struct timeval bt, at, dt;
	gettimeofday(&bt, NULL);
#ifdef USE_INTEGRATION_IMAGE
	for (int y = 0; y < image_height; y++)
	{
		for (int x = 0; x < image_width; x++)
		{
			Eigen::VectorXd work_v = Eigen::VectorXd::Zero(num_bins);
			for (int i = 0; i < r; i++)
			{
				for (int j = 0; j < r; j++)
				{
					int R = search_img.at<cv::Vec3b>(y * r + i,
													 x * r + j)[2];
					int G = search_img.at<cv::Vec3b>(y * r + i,
													 x * r + j)[1];
					int B = search_img.at<cv::Vec3b>(y * r + i,
													 x * r + j)[0];
					int idx = get_index(R, G, B);
					work_v(idx) = work_v(idx) + 1;
				}
			}
			I[y][x] = work_v;
			Id[y][x] = Eigen::VectorXd::Zero(num_bins);
			h_idx[y][x] = Eigen::VectorXd::Zero(num_bins);
		}
	}
	for (int y = 0; y < image_height; y++)
	{
		for (int x = 0; x < image_width; x++)
		{
			Eigen::VectorXd work = I[y][x];
			if (x != 0)
				work = work + Id[y][x - 1];
			Id[y][x] = work;
		}
	}
	for (int y = 0; y < image_height; y++)
	{
		for (int x = 0; x < image_width; x++)
		{
			Eigen::VectorXd work = Id[y][x];
			if (y != 0)
				work = work + h_idx[y - 1][x];
			h_idx[y][x] = work;
		}
	}
#else
	for (int y = 0; y < image_height; y++)
	{
		for (int x = 0; x < image_width; x++)
		{
			Eigen::VectorXd work_v = Eigen::VectorXd::Zero(num_bins);
			if (r > 1)
			{
				for (int i = 0; i < r; i++)
				{
					for (int j = 0; j < r; j++)
					{
						int R = search_img.at<cv::Vec3b>(y * r + i,
														 x * r + j)[2];
						int G = search_img.at<cv::Vec3b>(y * r + i,
														 x * r + j)[1];
						int B = search_img.at<cv::Vec3b>(y * r + i,
														 x * r + j)[0];
						int idx = get_index(R, G, B);
						work_v(idx) = work_v(idx) + 1;
					}
				}
			}
			else
			{
				int R = search_img.at<cv::Vec3b>(y, x)[2];
				int G = search_img.at<cv::Vec3b>(y, x)[1];
				int B = search_img.at<cv::Vec3b>(y, x)[0];
				int idx = get_index(R, G, B);
				work_v(idx) = work_v(idx) + 1;
			}
			h_idx[y][x] = work_v / (double)(r * r);
		}
	}
#endif
	gettimeofday(&at, NULL);
	timersub(&at, &bt, &dt);
	//double integ_image_time = (double)dt.tv_sec + (double)dt.tv_usec / (double)1e6;
	//std::cout << integ_image_time << std::endl;
	double max = init_th;
	double match = 0.0;

	//int count  = 0;
	for (int k = max_w; k >= init_w; k -= dw)
	{
		for (int i = 0; i <= image_height - k; i += shift)
		{
			for (int j = 0; j <= image_width - k; j += shift)
			{
				if (is_active_search)
				{
					if (e[(k - init_w) / dw][i / shift][j / shift] <
						max * ex_th)
					{
						continue;
					}
				}
				//count ++ ;
				Eigen::VectorXd work_hist =
					make_h_histogram(h_idx, j, i, k);
				match = matching_ratio(hist_query, work_hist);
				e[(k - init_w) / dw][i / shift][j / shift] = match;
				if (max < match)
				{
					max = match;
				}
				if (is_active_search)
				{
					//アクティブサーチ
					for (int n = k; n >= init_w; n -= dw)
					{
						for (int l = 0; l < k && i + l <= image_height - n;
							 l += shift)
						{
							for (int m = 0;
								 m < k && j + m <= image_width - n;
								 m += shift)
							{
								if (l == 0 && m == 0 && n == k)
									continue;

								double sh = match * k * k;
								if (l + n < k)
								{
									if (m + n < k)
									{
										if (n * n <= sh)
										{
											continue; //sh = n*n;
										}
									}
									else
									{
										if (n * (k - m) <= sh)
										{
											continue; //sh = n*(k-m);
										}
										sh += n * n - n * (k - m);
									}
								}
								else
								{
									if (m + n < k)
									{
										if ((k - l) * n <= sh)
										{
											continue; //sh = (k-l)*n;
										}
										sh += n * n - (k - l) * n;
									}
									else
									{
										if ((k - l) * (k - m) <= sh)
										{
											continue; //sh = (k-l)*(k-m);
										}
										sh += (n * n - (k - l) * (k - m));
									}
								}

								sh /= (double)(n * n);
								if (sh <
									e[(n - init_w) / dw][(i +
														  l) /
														 shift][(j +
																 m) /
																shift])
								{
									e[(n - init_w) / dw][(i +
														  l) /
														 shift][(j +
																 m) /
																shift] = sh;
								}
							}
						}
					}
				}
			}
		}
	}
	//std::cout << count << std::endl;
	int ret = 0;
	if (max != init_th)
	{
		double output_th = max * ex_th;
		for (int i = 0; i < (max_w - init_w) / dw + 1; i++)
		{
			for (int j = 0;
				 j < (image_height - (init_w + dw * i)) / shift + 1; j++)
			{
				for (int k = 0;
					 k < (image_width - (init_w + dw * i)) / shift + 1;
					 k++)
				{
					if (e[i][j][k] >= output_th)
					{
						struct Region_answer dat;
						dat.x = k * shift;
						dat.y = j * shift;
						dat.k = i * dw + init_w;
						dat.s = e[i][j][k];
						ans.push_back(dat);
					}
				}
			}
		}
	}
	else
	{
		ret = -1;
	}
	return ret;
}

#else
int ColorHistogramSearch::search_hist(const cv::Mat search_img,
									  const double hist_query[26],
									  std::vector<
										  struct Region_answer> &ans)
{

	if (search_img.size().width != image_width || search_img.size().height != image_height)
	{
		free_memory();
		prepare_memory(search_img.size().width, search_img.size().height);
	}

	for (int i = 0; i < (max_w - init_w) / (dw) + 1; i++)
	{
		for (int j = 0;
			 j < (image_height - ((init_w) + dw * i)) / (shift) + 1; j++)
		{
			for (int k = 0;
				 k < (image_width - ((init_w) + dw * i)) / (shift) + 1;
				 k++)
			{
				e[i][j][k] = 1.0;
			}
		}
	}

	for (int i = 0; i < image_height; i++)
	{
		for (int j = 0; j < image_width; j++)
		{
			int r = search_img.at<cv::Vec3b>(i, j)[2];
			int g = search_img.at<cv::Vec3b>(i, j)[1];
			int b = search_img.at<cv::Vec3b>(i, j)[0];
			h_idx[i][j] = get_index(r, g, b);
		}
	}

	double max = init_th;
	double match = 0.0;
	double work_hist[26] = {0};
	for (int k = max_w; k >= init_w; k -= dw)
	{
		for (int i = 0; i <= image_height - k; i += shift)
		{
			for (int j = 0; j <= image_width - k; j += shift)
			{
				int idx_k = (k - init_w) / dw;
				int idx_i = i / shift;
				int idx_j = j / shift;
				if (is_active_search)
				{
					if (e[idx_k][idx_i][idx_j] < max * ex_th)
					{
						continue;
					}
				}
				make_h_histogram(work_hist, j, i, k, k);
				match = matching_ratio(hist_query, work_hist);
				e[idx_k][idx_i][idx_j] = match;
				if (max < match)
				{
					max = match;
				}
				if (is_active_search)
				{
					//アクティブサーチ
					for (int n = k; n >= init_w; n -= dw)
					{
						if (match * k * k > n * n)
						{
							continue;
						}
						for (int l = 0; l < k && i + l <= image_height - n;
							 l += shift)
						{
							for (int m = 0;
								 m < k && j + m <= image_width - n;
								 m += shift)
							{
								if (l == 0 && m == 0 && n == k)
									continue;

								double sh = match * k * k;
								if (l + n < k)
								{
									if (m + n < k)
									{
										if (n * n <= sh)
										{
											continue; //sh = n*n;
										}
									}
									else
									{
										if (n * (k - m) <= sh)
										{
											continue; //sh = n*(k-m);
										}
										sh += n * n - n * (k - m);
									}
								}
								else
								{
									if (m + n < k)
									{
										if ((k - l) * n <= sh)
										{
											continue; //sh = (k-l)*n;
										}
										sh += n * n - (k - l) * n;
									}
									else
									{
										if ((k - l) * (k - m) <= sh)
										{
											continue; //sh = (k-l)*(k-m);
										}
										sh += (n * n - (k - l) * (k - m));
									}
								}

								sh /= (double)(n * n);
								int idx_n = (n - init_w) / dw;
								int idx_l = (i + l) / shift;
								int idx_m = (j + m) / shift;
								if (sh < e[idx_n][idx_l][idx_m])
								{
									e[idx_n][idx_l][idx_m] = sh;
								}
							}
						}
					}
				}
			}
		}
	}

	int ret = 0;
	if (max != init_th)
	{
		double output_th = max * ex_th;
		for (int i = 0; i < (max_w - init_w) / dw + 1; i++)
		{
			for (int j = 0;
				 j < (image_height - (init_w + dw * i)) / shift + 1; j++)
			{
				for (int k = 0;
					 k < (image_width - (init_w + dw * i)) / shift + 1;
					 k++)
				{
					if (e[i][j][k] >= output_th)
					{
						struct Region_answer dat;
						dat.x = k * shift;
						dat.y = j * shift;
						dat.k = i * dw + init_w;
						dat.s = e[i][j][k];
						ans.push_back(dat);
					}
				}
			}
		}
	}
	else
	{
		ret = -1;
	}
	return ret;
}
#endif

class ColorHistogramSearchNode
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW ros::NodeHandle nh_;
	ros::Subscriber image_sub_;
	ros::Publisher image_pub_;
	ros::Publisher boundingbox_pub_;
	ColorHistogramSearch *search;
	cv::Mat query_img;

#ifdef USE_EIGEN
	Eigen::VectorXd hist_query;
#else
	double hist_query[26];
#endif

	cv::Ptr<cv::Feature2D> features;
	std::vector<cv::KeyPoint> query_keypoints;
	cv::Mat query_descriptors;
	bool is_sift_search;

public:
	ColorHistogramSearchNode()
	{
		ros::NodeHandle nh_priv("~");
		//publish boundingbox message
		boundingbox_pub_ = nh_priv.advertise<color_histogram_search::BoundingBox>("/bounding_boxes", 1000);

		int min_w, max_w, dw, shift;
		std::string query_name;
		double ex_th, v_th, s_th, init_th;
		bool active;
		std::string defalut_query_name =
			ros::package::getPath("color_histogram_search") +
			std::string("/image/caffe.png");

		nh_priv.param<int>("min_window", min_w, 40);
		nh_priv.param<int>("max_window", max_w, 160);
		nh_priv.param<int>("delta_window", dw, 20);
		nh_priv.param<int>("window_shift", shift, 10);
		nh_priv.param<std::string>("query_image", query_name,
								   defalut_query_name);
		nh_priv.param<double>("ex_th", ex_th, 0.8);
		nh_priv.param<double>("init_th", init_th, 0.5);
		nh_priv.param<double>("v_th", v_th, 0.2);
		nh_priv.param<double>("s_th", s_th, 0.1);
		nh_priv.param<bool>("active_search", active, false);
		nh_priv.param<bool>("sift_search", is_sift_search, true);
		std::cout << "min_window : " << min_w << std::endl;
		std::cout << "max_window : " << max_w << std::endl;
		std::cout << "delta_window : " << dw << std::endl;
		std::cout << "window_shift : " << shift << std::endl;
		std::cout << "query_name : " << query_name << std::endl;
		std::cout << "active_search : " << active << std::endl;
		std::cout << "sift_search " << is_sift_search << std::endl;
		search = new ColorHistogramSearch(640, 480, shift, min_w, max_w, dw,
										  init_th, ex_th, v_th, s_th, active);
		// Subscrive to input video feed and publish output video feed
		image_sub_ =
			nh_priv.subscribe("input", 1,
							  &ColorHistogramSearchNode::Callbackfunction,
							  this);
		image_pub_ =
			nh_priv.advertise<sensor_msgs::Image>("output", 1);

		cv::namedWindow(OPENCV_WINDOW);
		query_img = cv::imread(query_name);
#ifdef USE_EIGEN
		hist_query = search->make_query_histogram(query_img);
#else
		search->make_query_histogram(query_img, hist_query);
#endif
		if (is_sift_search)
		{
			cv::Mat grayImg_query;
			cv::cvtColor(query_img, grayImg_query, CV_BGR2GRAY);
			features = cv::xfeatures2d::SIFT::create();
			features->detectAndCompute(grayImg_query, cv::noArray(), query_keypoints, query_descriptors);
		}
	}

	~ColorHistogramSearchNode()
	{
		delete search;
		cv::destroyWindow(OPENCV_WINDOW);
	}

	void Callbackfunction(const sensor_msgs::ImageConstPtr &msg)
	{

		cv_bridge::CvImagePtr cv_ptr;

		try
		{
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		}
		catch (cv_bridge::Exception &e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

		struct timeval bt, at, dt;
		std::vector<struct Region_answer> ans;
		gettimeofday(&bt, NULL);
		int ret = search->search_hist(cv_ptr->image, hist_query, ans);

		gettimeofday(&at, NULL);
		timersub(&at, &bt, &dt);
		double histgram_time = (double)dt.tv_sec + (double)dt.tv_usec / (double)1e6;
		double sift_time = 0.0;
		cv::Mat mask = cv::Mat(cv_ptr->image.size(), CV_8UC1, cv::Scalar(0));
		if (ret == 0)
		{
			mask = search->make_mask(ans, 1.0);
			if (is_sift_search)
			{
				gettimeofday(&bt, NULL);
				cv::Mat grayImg_input;
				cv::cvtColor(cv_ptr->image, grayImg_input, CV_BGR2GRAY);
				std::vector<cv::KeyPoint> input_keypoints;
				cv::Mat input_descriptors;
				features->detectAndCompute(grayImg_input, mask,
										   //features->detectAndCompute(grayImg_input, cv::noArray(),
										   input_keypoints,
										   input_descriptors);
				auto matchtype = features->defaultNorm(); // SIFT, SURF: NORM_L2
				// BRISK, ORB, KAZE, A-KAZE: NORM_HAMMING
				cv::BFMatcher matcher(matchtype);
				std::vector<std::vector<cv::DMatch>> knn_matches;
				matcher.knnMatch(query_descriptors, input_descriptors,
								 knn_matches, 2);
				const float match_par = .6f;

				std::vector<cv::DMatch> good_matches;
				std::vector<cv::Point2f> match_point1;
				std::vector<cv::Point2f> match_point2;
				for (size_t i = 0; i < knn_matches.size(); ++i)
				{
					auto dist1 = knn_matches[i][0].distance;
					auto dist2 = knn_matches[i][1].distance;

					//良い点を残す（最も類似する点と次に類似する点の類似度から）
					if (dist1 <= dist2 * match_par && knn_matches[i][0].queryIdx < (int)query_keypoints.size() && knn_matches[i][0].trainIdx < (int)input_keypoints.size())
					{
						good_matches.push_back(knn_matches[i][0]);
						match_point1.push_back(query_keypoints[knn_matches[i][0].queryIdx].pt);
						match_point2.push_back(input_keypoints[knn_matches[i][0].trainIdx].pt);
					}
				}

				cv::Mat H;
				cv::Mat H_masks;
				const int match_point_num = 10;

				if (match_point1.size() >= match_point_num && match_point2.size() >= match_point_num)
				{
					cv::Mat H_masks;
					H = cv::findHomography(match_point1, match_point2, H_masks, cv::RANSAC, 3.f);
					if (!H.empty())
					{
						// 対象物体画像からコーナーを取得 ( 対象物体が"検出"される )
						std::vector<cv::Point2f> obj_corners(4);
						obj_corners[0] = cv::Point2f(.0f, .0f);
						obj_corners[1] = cv::Point2f(static_cast<float>(query_img.cols), .0f);
						obj_corners[2] = cv::Point2f(static_cast<float>(query_img.cols), static_cast<float>(query_img.rows));
						obj_corners[3] = cv::Point2f(.0f, static_cast<float>(query_img.rows));

						// シーンへの射影を推定
						std::vector<cv::Point2f> scene_corners(4);
						cv::perspectiveTransform(obj_corners,
												 scene_corners, H);

						// コーナー間を線で結ぶ ( シーン中のマップされた対象物体 - シーン画像 )
						//float w = static_cast < float >(query_img.cols);
						cv::line(cv_ptr->image, scene_corners[0],
								 scene_corners[1], cv::Scalar(0, 255, 0),
								 4);
						cv::line(cv_ptr->image, scene_corners[1],
								 scene_corners[2], cv::Scalar(0, 255, 0),
								 4);
						cv::line(cv_ptr->image, scene_corners[2],
								 scene_corners[3], cv::Scalar(0, 255, 0),
								 4);
						cv::line(cv_ptr->image, scene_corners[3],
								 scene_corners[0], cv::Scalar(0, 255, 0),
								 4);
					}
				}
				gettimeofday(&at, NULL);
				timersub(&at, &bt, &dt);
				sift_time =
					(double)dt.tv_sec +
					(double)dt.tv_usec / (double)1e6;
			}
			std::vector<struct R> ans2;
			search->calc_intersect(ans, 1.0, ans2);

			color_histogram_search::BoundingBox boudingBox_result;
			boudingBox_result.Class = "bottle";
	
			for (int i = 0; i < (int)(ans2.size()); i++)
			{
				//std::cout << ans2[i].xmin << "  " << ans2[i].xmax << " " << ans2[i].ymin << " " <<ans2[i].ymax << std::endl;
				//double p = ans2[i].s;
				cv::rectangle(cv_ptr->image,
							  cv::Point(ans2[i].xmin, ans2[i].ymin),
							  cv::Point(ans2[i].xmax, ans2[i].ymax),
							  cv::Scalar(0, 0, 255), 3, 4);

				//publish boundingvox message
				boudingBox_result.probability = ans[i].s; //simularity
				boudingBox_result.xmin = ans2[i].xmin;
				boudingBox_result.ymin = ans2[i].ymin;
				boudingBox_result.xmax = ans2[i].xmax;
				boudingBox_result.ymax = ans2[i].ymax;
				boundingbox_pub_.publish(boudingBox_result);
			}
		}
		else
		{
			sift_time = 0.0;
		}
		fprintf(stderr, "calc time : %6lfsec %6lfsec\n", histgram_time, sift_time);

		// Update GUI Window
		cv::imshow("test", mask);
		cv::imshow(OPENCV_WINDOW, cv_ptr->image);
		cv::waitKey(3);

		// Output modified video stream
		//image_pub_.publish(cv_ptr->toImageMsg());
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ColorHistogramSearch");

	ColorHistogramSearchNode *chs = new ColorHistogramSearchNode();
	ros::spin();
	return 0;
}
