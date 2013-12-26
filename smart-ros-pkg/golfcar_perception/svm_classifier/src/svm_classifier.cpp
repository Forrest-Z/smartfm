#include "svm_classifier.h"

namespace golfcar_ml{
  
	svm_classifier::svm_classifier(string svm_model_path, string svm_scale_path)
    { 
      string svm_model_file = svm_model_path;
      svm_model_ = svm_load_model(svm_model_file.c_str());
      cout<<" SVM loaded, type = "<< svm_model_->param.svm_type <<endl;
      
      string svm_scale_file;
      svm_scale_file = svm_scale_path;
      restore_scalefile(svm_scale_file, feature_min_, feature_max_, feature_index_);
    }

	//to be continued;
    int svm_classifier::classify_objects(double * feature_vector, int feature_length)
    {
    	if(feature_index_ != feature_length) ROS_ERROR("feature vector is not properly long");
    	//the defaut class_label is -1; it can be used to check whether the object is successfully classified;
        int class_label = -1;
        struct svm_node *x;
        int max_nr_attr = feature_length + 1;
        x = (struct svm_node *) malloc(max_nr_attr*sizeof(struct svm_node));
        for(int i=0; i < feature_length; i++)
		{
        	x[i].index = i+1;
        	x[i].value = feature_vector[i];
		}
        //this extra one is added to denote the end of feature vector;
        x[feature_length].index = -1;

        ROS_DEBUG("original features");
        for(int i=0; i < feature_length; i++)
		{
        	ROS_DEBUG("i, value: %d, %lf", i, x[i].value);
		}

        for(int i=0; i< feature_length; i++) x[i].value = output(x[i].index, x[i].value);

        ROS_DEBUG("scaled features");
        for(int i=0; i < feature_length; i++)
		{
        	ROS_DEBUG("i, value: %d, %lf", i, x[i].value);
		}

        ROS_DEBUG("try to predict");
        class_label = svm_predict(svm_model_,x);
        ROS_DEBUG("predict finished %d", class_label);

        free(x);
        return class_label;
    }
    
    void svm_classifier::restore_scalefile(string filename, double* &feature_min, double* &feature_max, int &feature_index)
	{
		int idx, c;
		FILE *fp_restore;
		const char *restore_filename = filename.c_str();
		double y_max = -DBL_MAX;
		double y_min = DBL_MAX;
		double y_lower,y_upper;
		int max_index=0;

		fp_restore = fopen(restore_filename,"r");
		if(fp_restore==NULL)
		{
			fprintf(stderr,"can't open file %s\n", restore_filename);
			exit(1);
		}
		cout<<"File opened"<<endl;
		c = fgetc(fp_restore);
		if(c == 'y')
		{
			readline(fp_restore);
			readline(fp_restore);
			readline(fp_restore);
		}
		cout<<readline(fp_restore)<<endl;
		cout<<readline(fp_restore)<<endl;
		cout<<"Retrieving maximum index"<<endl;
		while(fscanf(fp_restore,"%d %*f %*f\n",&idx) == 1)
			max_index = max(idx,max_index);
		rewind(fp_restore);
		cout<<"Max index retrieved "<<max_index<<endl;
		feature_max = (double *)malloc((max_index+1)* sizeof(double));
		feature_min = (double *)malloc((max_index+1)* sizeof(double));

		double fmin, fmax;
		int y_scaling = 0;
		if((c = fgetc(fp_restore)) == 'y')
		{
			fscanf(fp_restore, "%lf %lf\n", &y_lower, &y_upper);
			fscanf(fp_restore, "%lf %lf\n", &y_min, &y_max);
			y_scaling = 1;
		}
		else
			ungetc(c, fp_restore);

		if (fgetc(fp_restore) == 'x') {
			cout<<"got x"<<endl;
			fscanf(fp_restore, "%lf %lf\n", &lower_, &upper_);
			while(fscanf(fp_restore,"%d %lf %lf\n",&idx,&fmin,&fmax)==3)
			{
				if(idx<=max_index)
				{
					feature_min[idx] = fmin;
					feature_max[idx] = fmax;
				}
			}
		}
		feature_index = max_index;
		fclose(fp_restore);
		cout<<"File closed"<<endl;
	}
	
	char* svm_classifier::readline(FILE *input)
	{
		int len;
		char *line = NULL;
		int max_line_len = 1024;
		line = (char *) malloc(max_line_len*sizeof(char));
		if(fgets(line,max_line_len,input) == NULL)
			return NULL;

		while(strrchr(line,'\n') == NULL)
		{
			max_line_len *= 2;
			line = (char *) realloc(line, max_line_len);
			len = (int) strlen(line);
			if(fgets(line+len,max_line_len-len,input) == NULL)
				break;
		}
		return line;
	}

	double svm_classifier::output(int index, double value)
	{
		/* skip single-valued attribute */
		if(feature_max_[index] == feature_min_[index])
			return value;

		if(value == feature_min_[index])
			value = lower_;
		else if(value == feature_max_[index])
			value = upper_;
		else
			value = lower_ + (upper_-lower_) *
				(value-feature_min_[index])/
				(feature_max_[index]-feature_min_[index]);

		if(value != 0)
		{
			//printf("%d:%g ",index, value);
		}
		return value;
	}

	svm_classifier::~svm_classifier()
    {
    }
};
