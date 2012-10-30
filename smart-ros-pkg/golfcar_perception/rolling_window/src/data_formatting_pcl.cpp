#ifndef DATA_FORMATTING_GOLFCART_PCL
#define DATA_FORMATTING_GOLFCART_PCL

#include <cstdio>
#include <stdlib.h>

int main(int argc, char** argv) 
{
	 if(argc != 3){return -1;}
	 
	 int total_lines = atoi(argv[2]);
	 
    FILE *fp_input, *fp_output;
    if((fp_input=fopen(argv[1], "r"))==NULL){printf("cannot open input_file\n");return 0;}
    if((fp_output=fopen("./formatted_data", "a"))==NULL){printf("cannot open output_file\n");return 0;}
    
    unsigned int batch_serial, laser_serial;
    int class_label;
	 float pitch_speed, pitch, roll;
	 float curvature_vector[120];
	
    for(int i=1; i<=total_lines; i++)
    {
		  //prepare the input variables;
        
        fscanf(fp_input,  "%u\t%u\t", &batch_serial, &laser_serial);
        fscanf(fp_input,  "%d\t", &class_label);
        fscanf(fp_input,  "%f\t%f\t%f\t", &pitch_speed, &pitch, &roll);        
        for(int ic=0; ic<119; ic++) fscanf(fp_input, "%f\t", &curvature_vector[ic]);
        fscanf(fp_input, "%f\n", &curvature_vector[119]);
        
		  //write input variables;
        fprintf(fp_output,  "%u\t", class_label);
        fprintf(fp_output, "1:%f\t2:%f\t3:%f\t", pitch_speed, pitch, roll);
        int feature_serial_tmp;
        for(int ic=0; ic<119; ic++) 
        {
			  feature_serial_tmp = ic + 1 +3;
			  fprintf(fp_output, "%d:%f\t", feature_serial_tmp, curvature_vector[ic]);
        }
        feature_serial_tmp = 119+4;
        fprintf(fp_output, "%d:%f\n", feature_serial_tmp, curvature_vector[119]);
    }
    
    fclose(fp_input);
    fclose(fp_output);
    
    return 0;
}

#endif
