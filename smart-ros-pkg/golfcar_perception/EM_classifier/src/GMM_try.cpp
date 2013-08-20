#include "opencv2/core/core.hpp"
#include "opencv2/ml/ml.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <cstdio>
#include <vector>
#include <iostream>
#include <cmath>

using namespace cv;
using namespace std;

void draw_covMatrix_eclipse( Mat img, Mat CurrCovMat, Mat Means, int i)
{
	double cx = Means.at<double>(i,0);
	double cy = Means.at<double>(i,1);

	Mat eigenvalues;
	Mat eigenvectors;
	eigen( CurrCovMat, eigenvalues, eigenvectors );

	double eigenvec1_len = eigenvalues.at<double>(0,0);
	double len1          = sqrt(eigenvec1_len)*3;
	double eigenvec1_x   = eigenvectors.at<double>(0,0) * len1;
	double eigenvec1_y   = eigenvectors.at<double>(0,1) * len1;

	double eigenvec2_len = eigenvalues.at<double>(1,0);
	double len2          = sqrt(eigenvec2_len)*3;
	double eigenvec2_x   = eigenvectors.at<double>(1,0) * len2;
	double eigenvec2_y   = eigenvectors.at<double>(1,1) * len2;

	// Show axes of ellipse
	line( img, cv::Point(cx,cy), cv::Point(cx+eigenvec1_x, cy+eigenvec1_y), CV_RGB(255,255,0) );
	line( img, cv::Point(cx,cy), cv::Point(cx+eigenvec2_x, cy+eigenvec2_y), CV_RGB(0,255,0) );

	// Show ellipse rotated into direction of eigenvec1
	double dx = eigenvec1_x;
	double dy = eigenvec1_y;
	double angle_rad = atan2( dy, dx );
	double angle_deg = angle_rad * (180.0/M_PI); // convert radians (0,2PI) to degree (0°,360°)
	cv::RotatedRect* myRotatedRect = new cv::RotatedRect( cvPoint(cx,cy), cvSize(len1, len2), angle_deg );
	if (myRotatedRect != NULL)
	{
		int g = 1*255.0 * (i+1);
		cv::Scalar s = CV_RGB(g,g,g);
		ellipse( img, *myRotatedRect, s );
		delete myRotatedRect;
	}
	cout<<CurrCovMat<<endl;
	cout<<eigenvalues<<endl;
	cout<<eigenvectors<<endl;
	printf("cx, cy, eigenvec1_x, eigenvec1_y, eigenvec2_x, eigenvec2_y, angle_deg: (%3f, %3f), (%3f, %3f), (%3f, %3f), %3f", cx, cy, eigenvec1_x, eigenvec1_y, eigenvec2_x, eigenvec2_y, angle_deg);
}

double prob_2DGaussian(Mat sample, Mat Mean, Mat CovMat)
{
	//"sample" and "Mean" are row vectors (1x2); CovMat 2x2;
	Mat sample_tmp;
	sample.convertTo(sample_tmp, CV_64F);

	/*
	printf("sample_tmp (%f, %f)\t", sample_tmp.at<double>(0,0), sample_tmp.at<double>(0,1));
	printf("Mean (%f, %f`)\t", Mean.at<double>(0,0), Mean.at<double>(0,1));
	printf("CovMat (%f, %f, %f, %f)\t", CovMat.at<double>(0,0), CovMat.at<double>(0,1), CovMat.at<double>(1,0), CovMat.at<double>(1,1));
	*/

	double p1 = 1/(2*M_PI*sqrt(determinant(CovMat)));
	double p2 = std::exp(determinant((-0.5)*(sample_tmp-Mean)*CovMat.inv()*(sample_tmp-Mean).t()));

	//printf("prb %f\t",p1*p2);

	return p1*p2;
}

int main( int /*argc*/, char** /*argv*/ )
{
	//1st: to mimic our application, generate random samples, N points from X Gaussians, and M points from a uniform distribution;

	//a. initialize the map;
	Mat img = Mat::zeros( Size( 1000, 1000 ), CV_8UC3 );

	int i, j;
	const int N = 1;

    const Scalar colors[] =
    {
        Scalar(0,0,255), Scalar(0,255,0),
        Scalar(0,255,255),Scalar(255,255,0),
        Scalar(255,0,255)
    };

    Mat img_visual = img.clone();
	Mat kmeans_img = img.clone();

	vector<Vec2f> samples_vector;

	for(i=0; i<10; i++)
	{
		float x_tmp = rand()%100+500;
		float y_tmp = 0.5*x_tmp + rand()%10;
		Vec2f sample_tmp(x_tmp, y_tmp);
		samples_vector.push_back(sample_tmp);
	}


	int nsamples = (int)samples_vector.size();
	Mat all_samples( nsamples, 2, CV_32FC1 );
	//all_samples = all_samples.reshape(2, 0);
	for( i = 0; i < nsamples; i ++ )
	{
		all_samples.at<float>(i, 0) = samples_vector[i].val[0];
		all_samples.at<float>(i, 1) = samples_vector[i].val[1];
	}
	//all_samples = all_samples.reshape(1, 0);

	int max_iteration = 100;
	int iteration_time = 0;
	double likelihood_EPS = 0.005;
	double last_likelihood = 0.0;
	double gaussian_fi[N];

	Mat gaussian_means[N];
	Mat gaussian_cov[N];
	Mat probsMat( nsamples, N, CV_64FC1 );
	Mat weightMat( nsamples, N, CV_64FC1 );
	Mat kmeans_labels, kmeans_centers;
	kmeans(all_samples, N, kmeans_labels, TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 100, 1.0), 10, KMEANS_PP_CENTERS, kmeans_centers);

	typedef std::vector<int> label_vector;
	std::vector<label_vector> clusters_labels (N);
	for( i = 0; i < nsamples; i++ )
	{
		Point pt(cvRound(all_samples.at<float>(i, 0)), cvRound(all_samples.at<float>(i, 1)));
		circle( img_visual, pt, 1, colors[kmeans_labels.at<int>(i)], CV_FILLED );
		if(kmeans_labels.at<int>(i)<N) clusters_labels[kmeans_labels.at<int>(i)].push_back(i);
	}

	Mat guassian_sample[N];
	for(i=0; i<N; i++)
	{
		printf("\n cluster label: %d, number: %ld \n", i, clusters_labels[i].size());
		Mat samples_tmp( (int)clusters_labels[i].size(), 2, CV_32FC1 );
		//samples_tmp = samples_tmp.reshape(2, 0);
		for(j=0; j<clusters_labels[i].size(); j++)
		{
			samples_tmp.at<float>(j, 0) = all_samples.at<float>(clusters_labels[i][j], 0);
			samples_tmp.at<float>(j, 1) = all_samples.at<float>(clusters_labels[i][j], 1);
		}
		guassian_sample[i] = samples_tmp;
	}
	cout<<guassian_sample[0]<<endl;

	for(i=0; i<N; i++)
	{
		gaussian_fi[i]= double(clusters_labels[i].size())/double(nsamples);
		calcCovarMatrix(guassian_sample[i], gaussian_cov[i], gaussian_means[i], CV_COVAR_NORMAL|CV_COVAR_ROWS | CV_COVAR_SCALE);
		draw_covMatrix_eclipse(img_visual, gaussian_cov[i], gaussian_means[i], 0);
	}

	imwrite( "./SourceSinks.jpg", img_visual );
}
