#ifndef LANE_MARKER_GENERATE_TRAINING_DATA
#define LANE_MARKER_GENERATE_TRAINING_DATA

#include <ros/ros.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "lane_marker_common.h"
#include <cstdio>
#include <vector>
#include <stdlib.h>

using namespace std;
using namespace golfcar_vision;

int main(int argc, char** argv) 
{
	FILE *fp;
	if((fp=fopen("/home/baoxing/word/OCR", "r"))==NULL){printf("cannot open file\n");return 0;}
    //--------------------------------------------------------------------------------------
    //there are multiple modules to be trained, 4 right now.
    int line_number;
    printf("Please key in the total line number\n");
    scanf("%d", &line_number);
    printf("line_number: %d\n", line_number);
    char letter = 0;
    unsigned int image_serial;

    for(int i=0; i<line_number;i++)
    {
    	FILE *fp_store;
    	if((fp_store=fopen("/home/baoxing/word/OCR_vectors", "a"))==NULL){printf("cannot open OCR_vectors file\n");return 0;}

    	printf("\n-----------------------------\n");
        fscanf(fp, "%d\t", &image_serial);
        fprintf(fp_store, "%d\t", image_serial);
        printf("image serial:%d\n", image_serial);

        int word_class;
		printf("Please key in the word_class\n");
		scanf("%d", &word_class);
		fprintf(fp_store, "%d\t", word_class);

        int character_number;
        printf("Please key in the current character number\n");
        scanf("%d", &character_number);
        //printf("character_number: %d\n", character_number);

        std::vector<int> feature_vector(27, 0);
        for(int j=0; j<character_number;j++)
        {
        	fscanf(fp,"%c\t",&letter);
        	printf("%c\t",letter);
        	if(letter>=65||letter<=90)feature_vector[letter-65]++;
        	else feature_vector[91-65]++;
        }
        fscanf(fp, "\n");
        printf("\n");

        for(size_t j=0; j<feature_vector.size(); j++)
        {
        	fprintf(fp_store, "%d\t", feature_vector[j]);
        	printf("%d\t", feature_vector[j]);
        }
        fprintf(fp_store, "\n");
        printf("\n\n");
        fclose(fp_store);
    }
    fclose(fp);
}

#endif
