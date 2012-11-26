#ifndef DATA_FORMATTING_GOLFCART_POINT
#define DATA_FORMATTING_GOLFCART_POINT

#include <cstdio>
#include <stdlib.h>

int main(int argc, char** argv) 
{
	 if(argc != 4){return -1;}
	 
	 int feature_length = atoi(argv[2]);
	 int total_lines = atoi(argv[3]);

	 if(feature_length < 1){printf("no feature, dude?"); return -1;}
    FILE *fp_input, *fp_output;
    if((fp_input=fopen(argv[1], "r"))==NULL){printf("cannot open input_file\n");return 0;}
    if((fp_output=fopen("./formatted_data", "a"))==NULL){printf("cannot open output_file\n");return 0;}

    int class_label;
	 double feature_vector[feature_length];
    for(int i=1; i<=total_lines; i++)
    {
    	fscanf(fp_input,  "%d\t", &class_label);
    	for(int j=0; j<feature_length-1; j++)
    	{
    		fscanf(fp_input,  "%lf\t", &feature_vector[j]);
    	}
    	fscanf(fp_input,  "%lf\t", &feature_vector[feature_length-1]);
		//prepare the input variables;

	  //write input variables;
	  fprintf(fp_output,  "%d\t", class_label);
	  for(int j=0; j<feature_length-1; j++)
		{
        	 fprintf(fp_output, "%d:%lf\t", j+1, feature_vector[j]);
		}
        fprintf(fp_output, "%d:%lf\n", feature_length, feature_vector[feature_length-1]);
    }
    
    fclose(fp_input);
    fclose(fp_output);
    
    return 0;
}

#endif
