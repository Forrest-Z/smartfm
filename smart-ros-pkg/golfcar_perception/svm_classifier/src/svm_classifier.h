#ifndef GOLFCAR_ML_SVM_CLASSIFIER_H
#define GOLFCAR_ML_SVM_CLASSIFIER_H

#include <ros/ros.h>
#include <float.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <cstring>

#include "libsvm/svm.h"

namespace golfcar_ml{

	using namespace std;
    class svm_classifier {
		public:
		svm_classifier(string svm_model_path, string svm_scale_path);
		~svm_classifier();

		//the feature_length is actually recorded as "feature_index_", which is restored from svm_scale_path;
		//it is needed here to help check whether the input feature vector has proper length;
		int classify_objects (double * feature_vector, int feature_length);

		private:
		struct svm_model *svm_model_;
		void restore_scalefile(string filename, double* &feature_min, double* &feature_max, int &feature_index);
		double output(int index, double value);
		char* readline(FILE *input);
		double* feature_max_;
		double* feature_min_;
		//very important to denote the length of the feature vector;
		int feature_index_;
		double lower_, upper_;
    };
};

#endif
