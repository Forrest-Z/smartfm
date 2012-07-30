#ifndef LANE_MARKER_GENERATE_TRAINING_DATA
#define LANE_MARKER_GENERATE_TRAINING_DATA

#include <cstdio>
#include <stdlib.h>

int main(int argc, char** argv) 
{
    FILE *fp_input, *fp_output;
    if((fp_input=fopen("./data/training_data", "r"))==NULL){printf("cannot open input_file\n");return 0;}
    if((fp_output=fopen("./data/formatted_data", "w"))==NULL){printf("cannot open output_file\n");return 0;}
    
    unsigned int image_serial, class_label;
    double weight, perimeter; int approxNum;
    double HMoment[7], Box_Sides[2];
    
    for(int i=1; i<=284; i++)
    {
        fscanf(fp_input,  "%u\t%u\t", &image_serial, &class_label);
        fscanf(fp_input,  "%lf\t%lf\t%d\t", &weight, &perimeter, &approxNum);
        
        fscanf(fp_input, "%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t", &HMoment[0], &HMoment[1], &HMoment[2], &HMoment[3], &HMoment[4], &HMoment[5], &HMoment[6]);
        fscanf(fp_input,  "%lf\t%lf\t\n", &Box_Sides[0], &Box_Sides[1]);
        
        //to scale these two dimensions;
        Box_Sides[0] = Box_Sides[0]/100.0;
        Box_Sides[1] = Box_Sides[1]/100.0;
        
        if(class_label==5) class_label = 4;
        
        fprintf(fp_output,  "%u\t", class_label);
        
        fprintf(fp_output, "1:%lf\t2:%lf\t3:%lf\t4:%lf\t5:%lf\t6:%lf\t7:%lf\t", HMoment[0], HMoment[1], HMoment[2], HMoment[3],HMoment[4], HMoment[5],HMoment[6]);
        fprintf(fp_output,  "8:%lf\t9:%lf\t", Box_Sides[0], Box_Sides[1]);
		//newly added;
        fprintf(fp_output,  "10:%lf\t11:%lf\t12:%d\t\n", weight, perimeter, approxNum); 
    }
    
    fclose(fp_input);
    fclose(fp_output);
    
    return 0;
}

#endif
