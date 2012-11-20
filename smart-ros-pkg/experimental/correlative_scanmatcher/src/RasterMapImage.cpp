#include <fstream>
#include <fmutil/fm_stopwatch.h>
#include <cv.h>
#include <highgui.h>
#include <fmutil/fm_math.h>
using namespace std;                    // make std:: accessible

class RasterMapImage
{
public:

    cv::Mat image_;
    RasterMapImage(double resolution, double range_covariance): res_(resolution), range_covariance_(range_covariance),
            max_dist_ (sqrt(log(255)*range_covariance)),
            gausian_length_((int) (max_dist_ / res_ + 1)),
            min_pt_(1e99,1e99), max_pt_(-1e99,-1e99)
    {
        gaussian_mapping_ = makeGaussianLinearMapping();
    }

    ~RasterMapImage()
    {

    }

    inline cv::Point imageCoordinate(cv::Point2f pt)
    {
        return cv::Point((pt.x-min_pt_.x)/res_, (int) image_.rows -(pt.y-min_pt_.y)/res_);
    }
    void getInputPoints(vector<cv::Point2f> raster_pt)
    {
        fmutil::Stopwatch sw;
        sw.start("Raster Map");

        //very fast process, only needs 5 ms on 1620 points with 1000 loops

        for(size_t i=0; i<raster_pt.size(); i++)
        {
            if(raster_pt[i].x < min_pt_.x) min_pt_.x = raster_pt[i].x;
            if(raster_pt[i].x > max_pt_.x) max_pt_.x = raster_pt[i].x;

            if(raster_pt[i].y < min_pt_.y) min_pt_.y = raster_pt[i].y;
            if(raster_pt[i].y > max_pt_.y) max_pt_.y = raster_pt[i].y;
        }

        cout << "MinMax "<<min_pt_ << " " <<max_pt_<<endl;
        cv::Point2f map_size(max_pt_.x - min_pt_.x, max_pt_.y - min_pt_.y);
        map_size.x = ceil(map_size.x/res_); map_size.y =ceil(map_size.y/res_);
        cout << "Size "<<map_size<<endl;
        //lost about 10ms when using 32F instead of 8U, and total of 45 ms if draw circle function is called, perhaps too much
        image_ = cv::Mat::zeros( (int) map_size.y, (int) map_size.x  , CV_8UC1);


        for(int j=gaussian_mapping_.size()-1; j>=0; j--)
        {
            //openmp helps reduce the rastering time from 150+ to 60+ms
            //#pragma omp parallel for
            for(size_t i=0; i<raster_pt.size(); i++)
            {
                cv::Point pt =imageCoordinate(raster_pt[i]);
                if(pt.x >= image_.cols || pt.y >= image_.rows) continue;

                this->rasterCircle(pt, j, image_, gaussian_mapping_[j]);

            }

        }
        sw.end();
        cv::imwrite("map.png", image_);
    }

    double scorePoints(vector<cv::Point2f> search_pt)
    {
        //fmutil::Stopwatch sw;
        //sw.start("Score Pts");
        assert(image_.data != NULL);

        double score = 0;
        int count = 0;

        for(size_t i=0; i<search_pt.size(); i++)
        {
            cv::Point pt =imageCoordinate(search_pt[i]);
            if(outsideMap(image_, pt)) continue;
            score += getPixel(pt.x, pt.y);
            count++;
        }
        score/=count;
        //sw.end();
        //cout<<"Score = "<<score/255*100<<"%"<<endl;
        return score/255*100;
    }

    void searchTranslation(vector<cv::Point2f> search_pt)
    {
        fmutil::Stopwatch sw;
        sw.start("searching");
        double best_score = 0;
        cv::Point2f best_trans;
        for(double j=-10.0; j<=10.0; j+=1.0)
        {
            for(double i=-10.0; i<=10.0; i+=1.0)
            {
                vector<cv::Point2f> new_search_pt;
                for(size_t k=0; k<search_pt.size(); k++)
                    new_search_pt.push_back(cv::Point2f(search_pt[k].x+i, search_pt[k].y+j));
                double cur_score = scorePoints(new_search_pt);
                if(cur_score>best_score)
                {
                    best_trans.x = i;
                    best_trans.y = j;
                    best_score = cur_score;
                }
                cout<< cur_score<<" ";
            }
            cout<<endl;
        }

        sw.end();
        cout<<best_trans<<" "<<best_score<<endl;
    }

private:
    vector<int> gaussian_mapping_;
    vector<double> circle_segments_cos_, circle_segments_sin_;
    int draw_circle_segments_;
    double res_, range_covariance_, max_dist_, gausian_length_;
    cv::Point2f min_pt_, max_pt_;
    inline bool outsideMap(cv::Mat &img, cv::Point pt)
    {
        if(pt.x >= img.cols || pt.y >= img.rows) return true;
        if(pt.x < 0 || pt.y < 0) return true;
        return false;
    }

    vector<int> makeGaussianLinearMapping()
    {
        vector<int> mapping;
        for(int i=0; i<gausian_length_; i++)
        {
            double d = res_ * i;
            mapping.push_back((int) (255*exp(-d*d/range_covariance_)));
            //cout<<d<<":"<<mapping[mapping.size()-1]<<endl;
        }
        //cout<<endl;
        return mapping;
    }

    inline int getPixel(int x, int y)
    {
        return image_.data[y*image_.cols + x];
    }
    inline void setPixel(int x, int y, cv::Mat &image, int color)
    {
        if(outsideMap(image, cv::Point(x,y))) return;

        //image.at<uchar>(y, x) = color;
        //gain 40ms by direct assignment of color
        image.data[y*image.cols + x] = color;

        if(!outsideMap(image, cv::Point(x-1,y))) image.data[y*image.cols + x -1] = color;
        if(!outsideMap(image, cv::Point(x,y-1))) image.data[(y-1)*image.cols + x] = color;
        if(!outsideMap(image, cv::Point(x+1,y))) image.data[y*image.cols + x + 1] = color;
        if(!outsideMap(image, cv::Point(x,y+1))) image.data[(y+1)*image.cols + x] = color;
    }
    //http://en.wikipedia.org/wiki/Midpoint_circle_algorithm
    //tested with another algorithm that is more assembly code friendly but there is no measureable difference
    void rasterCircle(cv::Point pt, int radius, cv::Mat &image, int color)
    {
        int x0 = pt.x, y0 = pt.y;
        int f = 1 - radius;
        int ddF_x = 1;
        int ddF_y = -2 * radius;
        int x = 0;
        int y = radius;

        setPixel(x0, y0 + radius, image, color);
        setPixel(x0, y0 - radius, image, color);
        setPixel(x0 + radius, y0, image, color);
        setPixel(x0 - radius, y0, image, color);

        while(x < y)
        {
            // ddF_x == 2 * x + 1;
            // ddF_y == -2 * y;
            // f == x*x + y*y - radius*radius + 2*x - y + 1;
            if(f >= 0)
            {
                y--;
                ddF_y += 2;
                f += ddF_y;
            }
            x++;
            ddF_x += 2;
            f += ddF_x;
            setPixel(x0 + x, y0 + y, image, color);
            setPixel(x0 - x, y0 + y, image, color);
            setPixel(x0 + x, y0 - y, image, color);
            setPixel(x0 - x, y0 - y, image, color);
            setPixel(x0 + y, y0 + x, image, color);
            setPixel(x0 - y, y0 + x, image, color);
            setPixel(x0 + y, y0 - x, image, color);
            setPixel(x0 - y, y0 - x, image, color);
        }
    }


};

bool readPt(istream &in, cv::Point2f &p)            // read point (false on EOF)
{

    if(!(in >> p.x)) return false;
    if(!(in >> p.y)) return false;

    return true;
}

int main(int argc, char **argv)
{
    istream*        data_in         = NULL;         // input for data points
    istream*        query_in         = NULL;         // input for query points
    vector<cv::Point2f> raster_pts, query_pts;
    cv::Point2f data_pts;
    ifstream dataStreamSrc, dataStreamDst;
    dataStreamSrc.open(argv[1], ios::in);// open data file
    if (!dataStreamSrc) {
        cerr << "Cannot open data file\n";
        exit(1);
    }
    data_in = &dataStreamSrc;

    dataStreamDst.open(argv[2], ios::in);// open data file
    if (!dataStreamDst) {
        cerr << "Cannot open query file\n";
        exit(1);
    }
    query_in = &dataStreamDst;

    while (readPt(*data_in, data_pts)) {
               raster_pts.push_back(data_pts);
           }
    cout << raster_pts.size() << "points read"<<endl;

    while (readPt(*query_in, data_pts)) {         // read query points
                query_pts.push_back(data_pts);
               }
    cout << query_pts.size() << "points read"<<endl;

    RasterMapImage rm(1.0, 0.1);
    rm.getInputPoints(raster_pts);
    rm.searchTranslation(query_pts);

    return 0;
}
