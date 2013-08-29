#include <fstream>
#include <string>
#include <list>

using namespace std;

#define stringArray std::vector<std::string>

void readInput(string fileName, stringArray &listString) {
//    printf("****************ReadInput***************\n");
 
    ifstream fin(fileName.c_str());
    string tmp = ""; // a line in the input file
    while(getline(fin, tmp)) {
        // add string into vector
        listString.push_back(tmp);
//          cout << tmp << endl;
//        cout << listString[listString.size() - 1] << endl;
    }
    printf("\n\n");
    fin.close();
}
 
/*
 * parseValues is to extract data in the input file
 */
void parseValues(sensor_msgs::PointCloud &pcl_scan, stringArray &listString) {
  printf("****************ParseValue***************\n");
  for (int i = 0; i < listString.size(); ++i) {
    char tmp[100000];
		
    strcpy(tmp, listString[i].c_str()); // copy string to char array
    stringArray tmpArray;
    // utilize string token to extract data
    char * pch;
    pch = strtok (tmp,",");

//    while (pch != NULL) {
    geometry_msgs::Point32 pt;

    pt.x = atof(pch);
    pch = strtok (NULL, ",");
    pt.y = atof(pch);
    pch = strtok (NULL, ",");
    pt.z = atof(pch);
    pch = strtok (NULL, ",");
//    }
    pcl_scan.points.push_back(pt);
  }
}
 
/*
 * parseValues is to write data at the beginning of file
 */
void writeToCSV(string fileName, sensor_msgs::PointCloud &pcl_scan) {
  ofstream fout(fileName.c_str());

  // for each row
  for (int i = 0; i < pcl_scan.points.size(); ++i) {
    // for each column
    fout << pcl_scan.points[i].x << ',' << pcl_scan.points[i].y << ',' << pcl_scan.points[i].z << "\r\n";
  }
  fout.close();
}

void CSVtoPointCloud(string fileName, sensor_msgs::PointCloud &pcl_scan)
{
  stringArray listString;

  readInput(fileName, listString);

  parseValues(pcl_scan, listString);

  listString.clear();
}

