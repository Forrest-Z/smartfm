#include "svm_classifier.h"
#include <iostream>
#include <fstream>

using namespace std;

//http://stackoverflow.com/questions/236129/how-to-split-a-string-in-c
vector<string> &split(const string &s, char delim, vector<string> &elems) {
    stringstream ss(s);
    string item;
    while (getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}

vector<string> split(const string &s, char delim) {
    vector<string> elems;
    split(s, delim, elems);
    return elems;
}

struct vehicleDetectionResult{
  double pos_x, pos_y;
  int grid_x, grid_y;
  int classify_result;
  int label;
  double grid_resolution;
  bool match_label;
  vehicleDetectionResult(vector<double> feature_vec, int _label, double _grid_resolution, double range, golfcar_ml::svm_classifier &svm):
  label(_label), grid_resolution(_grid_resolution){
    classify_result = svm.classify_objects(&feature_vec[0], feature_vec.size());
    match_label = -1;
    grid_x = -1;
    grid_y = -1;
    classify_result == label ? match_label=true : match_label=false;
    pos_x=0.0; pos_y=0.0;
    for(int j=5; j<11;j++){
      if(j%2 == 0) pos_y += feature_vec[j];
      else pos_x += feature_vec[j];
    }
    pos_x/=3.0;
    pos_y/=3.0;
    grid_x = (pos_x + range)/_grid_resolution;
    grid_y = (pos_y + range)/_grid_resolution;
  }
};

int main(int argc, char** argv){
  if(argc != 5){
    cout<<"verify_trained_data model_file scale_file input_file feature_length"<<endl;
    return 1;
  }
  string model_file = string(argv[1]);
  string scale_file = string(argv[2]);
  string input_file = string(argv[3]);
  int feature_length = atoi(argv[4]);
  
  cout<<"Received"<<endl;
  cout<<"model file: "<<model_file<<endl;
  cout<<"scale file: "<<scale_file<<endl;
  cout<<"input data: "<<input_file<<endl;
  cout<<"feature length: "<<feature_length<<endl;
  cout<<endl;
  cout<<"Reading input data"<<endl;
  ifstream input_filestream(input_file.c_str());
  vector<vector<double> > feature_vectors;
  vector<int> labels;
  if (input_filestream.is_open()){
    string line;
    while ( getline (input_filestream,line) ){
      vector<string> data_str = split(line, '\t');
      vector<double> feature_vec;

      assert((int)data_str.size() == feature_length + 1);

      labels.push_back(atoi(data_str[0].c_str()));
      for(int i=1; i<feature_length+1; i++){
	vector<string> data_temp = split(data_str[i], ':');
	double temp = atof(data_temp[1].c_str());
	feature_vec.push_back(temp);
      }
      feature_vectors.push_back(feature_vec);
      cout<<"Reading "<<feature_vectors.size()<<": length "<<(*--feature_vectors.end()).size();
      cout<<" label "<<labels[feature_vectors.size()-1]<<'\xd'<<flush;
    }
    
    cout<<endl<<"Done Reading "<<feature_vectors.size()<<": length size "<<(*--feature_vectors.end()).size();
      cout<<" label "<<labels.size()<<endl;
    assert(feature_vectors.size() == labels.size());
    input_filestream.close();
    
    golfcar_ml::svm_classifier svm(model_file, scale_file);
    int failed_label_count =0 ;
    vector<vehicleDetectionResult> results;
    double resolution = 5.0;
    double range = 50.0;
    for(size_t i=0; i<feature_vectors.size(); i++){
//       int classified_object = svm.classify_objects(&(feature_vectors[i])[0], feature_length);
      vehicleDetectionResult svm_result(feature_vectors[i], labels[i], resolution, range, svm);
      results.push_back(svm_result);
      if(!svm_result.match_label){//if(labels[i] != classified_object){
	
	cout<<" label "<<labels[i]<<" but classified as "<<svm_result.classify_result<<" "
	<<svm_result.pos_x<<" "<<svm_result.pos_y<<" "<<svm_result.grid_x<<" "<<svm_result.grid_y<<endl;
	failed_label_count++;
      }
      
    }
    cout<<"Total recall "<<feature_vectors.size()-failed_label_count<<"/"<<feature_vectors.size()<<"="<<(feature_vectors.size()-failed_label_count)/(double)feature_vectors.size()*100.0<<endl;
    vector<int> training_sample;
    vector<int> total_correct_count;
    int grid_length = range/resolution*2;
    training_sample.resize(grid_length*grid_length);
    total_correct_count.resize(training_sample.size());
    for(size_t i=0; i<results.size(); i++){
      if(results[i].grid_x < 0 || results[i].grid_y < 0) continue;
      if(results[i].grid_x > grid_length*2 || results[i].grid_y > grid_length*2) continue;
      int idx = results[i].grid_x + results[i].grid_y * grid_length;
      if(results[i].label == 2)
      {
	training_sample[idx]++;
	if(results[i].match_label) total_correct_count[idx]++;
      }
    } 
    int just_checking = 0;
    for(int i=0; i<grid_length; i++){
      for(int j=0; j<grid_length; j++){
	int idx = i + j*grid_length;
	int diff = training_sample[idx] - total_correct_count[idx];
	if(training_sample[idx] > 0)
 	  cout<<(double)diff/training_sample[idx]*100.0<<" ";
	  //cout<<training_sample[idx]<<" ";
	else
	  cout<<-1<<" ";
	just_checking+=total_correct_count[idx];
      }
      cout<<endl;
    }
    cout<<"Just checking: "<<just_checking<<endl;
  }
  else {
    cout<<"Failed to read input file"<<endl;
    return 1;
  }
  return 0;
}
