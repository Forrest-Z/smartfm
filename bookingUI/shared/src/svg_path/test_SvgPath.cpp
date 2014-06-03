#include <iostream>
#include <stdexcept>

#include "StationPath.h"

using namespace std;

void print_vector(string msg, vector<int>* points)
{
	cout<<"here"<<endl;
    if(points->size()==0) return;
    cout<<msg;
    for(int i=0; i<points->size(); i++)
    {
        cout<<' '<<points->at(i)<<' ';
    }
    cout<<endl;
}

void print_signals(StationPath *pose)
{
    print_vector("Left signals:  ", &pose->leftsig_pts_);
    print_vector("Right signals: ", &pose->rightsig_pts_);
    print_vector("Off signals:   ", &pose->offsig_pts_);
    print_vector("Intersections: ", &pose->ints_pts_);
    cout<<"Total points:   "<<pose->size()<<endl;
}
int main(int argc, char* argv[])
{
    try
    {
        /*
        SvgPath sp;
        sp.loadFile(argv[1], 0.1);
        StationPath pose= sp.getPath("DCC_EA");
        StationPath pose2= sp.getPath("EA_DCC");
        cout <<"Size DCC_MCD=" <<pose.size() <<' ' <<" Size MCD_DCC=" <<pose2.size() <<endl;
        vector<SlowZone> sz = sp.getSlowZone();
        cout <<sz[0].x_ <<' ' <<sz[0].y_ <<' ' <<sz[0].r_ <<endl;

        cout << "DCC_EA_signals"<<endl;
        print_signals(&pose);

        cout << "EA_DCC_signals"<<endl;
        print_signals(&pose2);
*/
        StationPaths complete;
        //complete.svgpath_.loadFile(argv[1], 0.1);
        StationList sl;
		/*
        for (int i=0; i<sl.size(); i++)
        {
            for (int j=0; j<sl.size(); j++)
            {
                if(i==j) continue;
                StationPath path = complete.getPath(sl.get(i),sl.get(j));
		cout<<i<<" "<<j<<":"<<endl;
                print_signals(&path);
                cout<<endl;
            }
            cout<<endl<<endl;
        }*/
		StationPath path = complete.getPath(sl.get(0),sl.get(1));
		print_signals(&path);


    }
    catch (runtime_error & error)
    {
        cout <<"Error: " <<error.what() <<endl;
    }
}
