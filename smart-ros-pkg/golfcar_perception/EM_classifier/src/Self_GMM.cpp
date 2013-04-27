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
	double eigenvec1_y   = eigenvectors.at<double>(1,0) * len1;

	double eigenvec2_len = eigenvalues.at<double>(1,0);
	double len2          = sqrt(eigenvec2_len)*3;
	double eigenvec2_x   = eigenvectors.at<double>(0,1) * len2;
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
	Mat img = Mat::zeros( Size( 600, 600 ), CV_8UC3 );

	int i, j;
	const int N = 4;
    const int N1 = (int)sqrt((double)N);
    const Scalar colors[] =
    {
        Scalar(0,0,255), Scalar(0,255,0),
        Scalar(0,255,255),Scalar(255,255,0),
        Scalar(255,0,255)
    };

    //Scalar uniform_color (255,0,255);

	rectangle(img, Point(img.rows/(N1+1), img.rows/(N1+1)), Point(N1*img.rows/(N1+1), N1*img.rows/(N1+1)), Scalar(255, 255, 255));

	Mat img_visual = img.clone();
	Mat kmeans_img = img.clone();

	int guassian_sample_n = 1000;
	int uniform_sample_n = 300;

    Mat samples( guassian_sample_n, 2, CV_32FC1 );
    samples = samples.reshape(2, 0);
    for( i = 0; i < N; i++ )
    {
        // form the training samples
        Mat samples_part = samples.rowRange(i*guassian_sample_n/N, (i+1)*guassian_sample_n/N );

        Scalar mean(((i%N1)+1)*img.rows/(N1+1),
                    ((i/N1)+1)*img.rows/(N1+1));
        Scalar sigma(10,10);
        randn(samples_part, mean, sigma);

        for( j = i*guassian_sample_n/N; j < (i+1)*guassian_sample_n/N; j ++ )
        {
            Point pt(cvRound(samples.at<float>(j, 0)), cvRound(samples.at<float>(j, 1)));
            circle( img, pt, 1, colors[i], CV_FILLED );
        }
    }

    Mat usamples( uniform_sample_n, 2, CV_32FC1 );
    usamples = usamples.reshape(2, 0);
    Scalar lower_bound(img.rows/(N1+1),img.rows/(N1+1));
    Scalar upper_bound(N1*img.rows/(N1+1), N1*img.rows/(N1+1));
    randu( usamples, lower_bound, upper_bound );
    for( i = 0; i < uniform_sample_n; i ++ )
    {
        Point pt(cvRound(usamples.at<float>(i, 0)), cvRound(usamples.at<float>(i, 1)));
        circle( img, pt, 1, colors[4], CV_FILLED );
    }

    Mat all_samples( guassian_sample_n + uniform_sample_n, 2, CV_32FC1 );
    all_samples = all_samples.reshape(2, 0);
    for( i = 0; i < guassian_sample_n; i ++ )
    {
    	all_samples.at<float>(i, 0) = samples.at<float>(i, 0);
    	all_samples.at<float>(i, 1) = samples.at<float>(i, 1);
    }
    for( i = 0; i < uniform_sample_n; i ++ )
    {
    	all_samples.at<float>(i+guassian_sample_n, 0) = samples.at<float>(i+guassian_sample_n, 0);
    	all_samples.at<float>(i+guassian_sample_n, 1) = samples.at<float>(i+guassian_sample_n, 1);
    }
    all_samples = all_samples.reshape(1, 0);

    //2nd: initialize the EM using prior information; Uniform + Gaussians, K-means for the Guassian part;

    EM em_model( N+1, EM::COV_MAT_GENERIC, TermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 300,  0.1));
    Mat loglikelihood, labels, probability;
    em_model.train( all_samples, loglikelihood, labels, probability);
    if(em_model.isTrained()) printf("\n trained\n");
    //draw the clustered samples
    for( i = 0; i < guassian_sample_n + uniform_sample_n; i++ )
    {
        Point pt(cvRound(all_samples.at<float>(i, 0)), cvRound(all_samples.at<float>(i, 1)));
        //printf("label %d\t", int(labels.at<int>(i)));
        circle( img_visual, pt, 1, colors[labels.at<int>(i)], CV_FILLED );
    }

    vector<Mat> covMats= em_model.get<vector<Mat> >("covs");
    Mat Means = em_model.get<Mat>("means");
    for(i=0; i<covMats.size(); i++) draw_covMatrix_eclipse(img_visual, covMats[i], Means, i);

    //self-implemented EM;
    int max_iteration = 100;
    int iteration_time = 0;
    double likelihood_EPS = 0.005;
    double last_likelihood = 0.0;

    //double uniform_fi;
    double gaussian_fi[5];
    Mat gaussian_means[5];
    Mat gaussian_cov[5];
    Mat probsMat( guassian_sample_n + uniform_sample_n, N+1, CV_64FC1 );
    Mat weightMat( guassian_sample_n + uniform_sample_n, N+1, CV_64FC1 );

    Mat kmeans_labels, kmeans_centers;
    kmeans(all_samples, N+1, kmeans_labels, TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 100, 1.0), 10, KMEANS_PP_CENTERS, kmeans_centers);

    typedef std::vector<int> label_vector;
    std::vector<label_vector> clusters_labels (N+1);
    for( i = 0; i < guassian_sample_n + uniform_sample_n; i++ )
    {
    	Point pt(cvRound(all_samples.at<float>(i, 0)), cvRound(all_samples.at<float>(i, 1)));
    	circle( kmeans_img, pt, 1, colors[kmeans_labels.at<int>(i)], CV_FILLED );

    	//printf("label %d\t", int(kmeans_labels.at<int>(i)));
    	if(kmeans_labels.at<int>(i)<N+1) clusters_labels[kmeans_labels.at<int>(i)].push_back(i);
    }

    Mat guassian_sample[5];
    for(i=0; i<N+1; i++)
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
    for(i=0; i<N+1; i++)
    {
    	gaussian_fi[i]= double(clusters_labels[i].size())/double(guassian_sample_n + uniform_sample_n);

    	calcCovarMatrix(guassian_sample[i], gaussian_cov[i], gaussian_means[i], CV_COVAR_NORMAL|CV_COVAR_ROWS | CV_COVAR_SCALE);

    	draw_covMatrix_eclipse(kmeans_img, gaussian_cov[i], gaussian_means[i], 0);
    	waitKey(500);
    }

    printf("\n Enter EM-iteration \n");

    /*
    Mat test_sample(1, 2, CV_32FC1 );
    test_sample.at<float>(0,0)= 0.0;
    test_sample.at<float>(0,1)= 0.0;
    Mat test_Mean(1, 2, CV_64FC1 );
    test_Mean.at<float>(0,0)= 0.0;
    test_Mean.at<float>(0,1)= 0.0;
    Mat test_Cov = Mat::eye(2, 2, CV_64FC1);
    double test_value = prob_2DGaussian(test_sample, test_Mean, test_Cov);
    printf("-------------------test value %lf---------------", test_value);
    */

    Mat em_labels(guassian_sample_n + uniform_sample_n, 1, CV_32SC1 );

    bool first_time = true;
    for(iteration_time = 0; iteration_time < max_iteration; iteration_time++)
    {
    	printf("\niteration_time %d\n", iteration_time);

    	//E-Step: calculate "probsMat";
    	double current_likelihood = 0.0;
    	for(i=0; i<N+1; i++) printf("means (%lf, %lf);\t fi:%lf\n", gaussian_means[i].at<double>(0,0), gaussian_means[i].at<double>(0,1), gaussian_fi[i]);

    	for(i=0; i<(guassian_sample_n + uniform_sample_n); i++)
        {
    		Mat sample_i = all_samples.rowRange(i,i+1);
    		//printf("sample_i (%f, %f)\t", sample_i.at<float>(0,0), sample_i.at<float>(0,1));

    		double sample_i_likelihood = 0;
    		double total_weights = 0.0;
            for(j=0; j<N+1; j++)
            {
            	//printf("sample: %d, %d; means: %d, %d; cov: %d, %d\n", sample_i.rows, sample_i.cols, gaussian_means[j].rows, gaussian_means[j].cols, gaussian_cov[j].rows, gaussian_cov[j].cols);
            	probsMat.at<double>(i,j) = prob_2DGaussian(sample_i, gaussian_means[j], gaussian_cov[j]);
            	//printf("probsMat %.12f\t", probsMat.at<double>(i,j));

            	sample_i_likelihood = sample_i_likelihood + probsMat.at<double>(i,j)*gaussian_fi[j];
            	total_weights = total_weights + probsMat.at<double>(i,j);
            }

            double largest_weight = 0.0;
            int largest_cluster = 0;
            for(j=0; j<N+1; j++)
            {
            	if(probsMat.at<double>(i,j)>=largest_weight){largest_weight = probsMat.at<double>(i,j); largest_cluster=j;}
            	//printf("sample: %d, %d; means: %d, %d; cov: %d, %d\n", sample_i.rows, sample_i.cols, gaussian_means[j].rows, gaussian_means[j].cols, gaussian_cov[j].rows, gaussian_cov[j].cols);
            	weightMat.at<double>(i,j) =  probsMat.at<double>(i,j)/total_weights;
            	//printf("weight %lf\t", weightMat.at<double>(i,j));
            }
            em_labels.at<int>(i,0) = largest_cluster;

            current_likelihood = current_likelihood+log(sample_i_likelihood);

            //printf("largest_cluster %d \n", largest_cluster);
        }

    	double delta_likelihood = current_likelihood - last_likelihood;
    	printf("delta_likelihood %lf;\n", delta_likelihood);
    	last_likelihood = current_likelihood;

    	if(first_time) { first_time=false;}
    	else if(delta_likelihood < likelihood_EPS) break;

    	//----------------------------------------M-Step: calculate "means, weights, covar matrices"--------------------------------------;
    	//pay attention to the format of the matrix;
    	kmeans_img = Scalar(0);

    	for(i=0; i<N+1; i++)
        {
    		gaussian_fi[i] = 0.0;
    	    gaussian_means[i] = 0.0;
    	    gaussian_cov[i] = 0.0;

    		for(j=0; j<(guassian_sample_n + uniform_sample_n); j++)
    		{
    			Mat sample_j = all_samples.rowRange(j,j+1);
    			Mat sample_j_tmp;
    			sample_j.convertTo(sample_j_tmp, CV_64F);
    			gaussian_fi[i] = gaussian_fi[i] + weightMat.at<double>(j,i);
    			gaussian_means[i] = gaussian_means[i] + weightMat.at<double>(j,i)*sample_j_tmp;
    		}
    		gaussian_means[i] = gaussian_means[i]/gaussian_fi[i];
    		gaussian_fi[i] = gaussian_fi[i]/(guassian_sample_n + uniform_sample_n);

    		printf("means (%lf, %lf);\t fi:%lf\n", gaussian_means[i].at<double>(0,0), gaussian_means[i].at<double>(0,1), gaussian_fi[i]);

    		for(j=0; j<(guassian_sample_n + uniform_sample_n); j++)
    		{
    			Mat sample_j = all_samples.rowRange(j,j+1);
				Mat sample_j_tmp;
				sample_j.convertTo(sample_j_tmp, CV_64F);
    			gaussian_cov[i] = gaussian_cov[i] + weightMat.at<double>(j,i)*(sample_j_tmp-gaussian_means[i]).t()*(sample_j_tmp-gaussian_means[i]);

    			Point pt(cvRound(all_samples.at<float>(j, 0)), cvRound(all_samples.at<float>(j, 1)));
        		circle( kmeans_img, pt, 1, colors[em_labels.at<int>(j)], CV_FILLED );
    		}
    		gaussian_cov[i] = gaussian_cov[i]/(gaussian_fi[i]*(guassian_sample_n + uniform_sample_n));
    		draw_covMatrix_eclipse(kmeans_img, gaussian_cov[i], gaussian_means[i], 0);

        }

    	imshow( "kmeans img", kmeans_img );
    	waitKey(100);
    }

	//3rd: the main part of EM;
	//4th: visualize the results;

    imshow( "EM-clustering result", img );
    imshow( "visual result", img_visual );
    imshow( "kmeans img", kmeans_img );
    waitKey(0);
    return 0;

}
