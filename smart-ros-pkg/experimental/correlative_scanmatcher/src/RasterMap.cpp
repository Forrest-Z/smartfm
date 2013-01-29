#include <ANN/ANN.h>                    // ANN declarations
#include <fstream>
#include <fmutil/fm_stopwatch.h>
#include <cv.h>
#include <highgui.h>
using namespace std;                    // make std:: accessible

struct RasterPtXY
{
    RasterPtXY(double ptx, double pty)
    {
        x = ptx;
        y = pty;
    }

    RasterPtXY(){};
    double x,y;
};

class RasterMap
{
public:
    RasterMap(double resolution): dim(2), eps(0.0)
    {

    }

    ~RasterMap()
    {
        delete kdTree;
        annClose();                                 // done with ANN
    }

    void getInputPoints(vector<RasterPtXY> raster_pt)
    {
        ANNpointArray       dataPts;                // data points
        dataPts = annAllocPts(raster_pt.size(), dim);
        for(size_t i=0; i<raster_pt.size(); i++)
        {
            dataPts[i][0] = raster_pt[i].x;
            dataPts[i][1] = raster_pt[i].y;
        }

        kdTree = new ANNkd_tree(                    // build search structure
                            dataPts,                    // the data points
                            raster_pt.size(),                       // number of points
                            dim);                       // dimension of space
    }

    void findNNPoints(vector<RasterPtXY> query_pts)
    {
        fmutil::Stopwatch sw;
        sw.start("Query Pts");

        ANNpoint queryPt = annAllocPt(dim);                     // query point
        ANNidxArray nnIdx = new ANNidx(1);                  // near neighbor indices
        ANNdistArray dists = new ANNdist(1);                  // near neighbor distances

        vector<double> distances;distances.resize(query_pts.size());
        vector<int> idx;idx.resize(query_pts.size());
        queryPt = annAllocPt(dim);
        for(size_t i=0; i<query_pts.size(); i++)
        {
            // echo query point
            //printPt(cout, queryPts[i]);
            queryPt[0] = query_pts[i].x;
            queryPt[1] = query_pts[i].y;
            kdTree->annkSearch(                     // search
                    queryPt,                        // query point
                    1,                              // number of near neighbors
                    nnIdx,                          // nearest neighbors (returned)
                    dists,                          // distance (returned)
                    eps);                           // error bound
            distances[i]=(dists[0]);
            idx[i]=(nnIdx[0]);
        }
        sw.end();

        for(size_t i=0; i<distances.size(); i++)
        {
            //cout<<i<<"- Query: "<<query_pts[i].x<<" "<<query_pts[i].y<<" NN: "<<kdTree->thePoints()[idx[i]][0]<<" "<<kdTree->thePoints()[idx[i]][1]<<", "<<distances[i]<<endl;
        }
        cout<<"Time taken: "<<sw.total_<<endl;
    }

private:
    ANNkd_tree*         kdTree;                 // search structure

    const int dim;            // dimension
    const double eps;            // error bound


};

bool readPt(istream &in, RasterPtXY &p)            // read point (false on EOF)
{

    if(!(in >> p.x)) return false;
    if(!(in >> p.y)) return false;

    return true;
}

int main(int argc, char **argv)
{
    istream*        data_in         = NULL;         // input for data points
    istream*        query_in         = NULL;         // input for query points
    vector<RasterPtXY> raster_pts, query_pts;
    RasterPtXY data_pts;
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

    RasterMap rm(0.1);
    rm.getInputPoints(raster_pts);
    rm.findNNPoints(query_pts);

    return 0;
}
