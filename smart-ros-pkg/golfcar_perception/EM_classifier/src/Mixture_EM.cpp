#include "opencv2/ml/ml.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <cstdio>

using namespace cv;

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

	//3rd: the main part of EM;
	//4th: visualize the results;
    imshow( "EM-clustering result", img );
    imshow( "visual result", img_visual );
    waitKey(0);
    return 0;

}
