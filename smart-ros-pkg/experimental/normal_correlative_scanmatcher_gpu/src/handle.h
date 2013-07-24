/*
 * Copyright 1993-2010 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property and
 * proprietary rights in and to this software and related documentation.
 * Any use, reproduction, disclosure, or distribution of this software
 * and related documentation without an express license agreement from
 * NVIDIA Corporation is strictly prohibited.
 *
 * Please refer to the applicable NVIDIA end user license agreement (EULA)
 * associated with this source code for terms and conditions that govern
 * your use of this NVIDIA software.
 *
 */


#ifndef __BOOK_H__
#define __BOOK_H__
#include <stdio.h>

struct MatchScore{
  int x;
  int y;
  int rot;
  float score;
};

struct MatchScoref{
  float x;
  float y;
  int rot;
  float score;
};

struct Point2fNormal {
  float x;
  float y;
  float normal;
};

struct Point2iNormal {
  int x;
  int y;
  float normal;
};

struct RawScore {
  int point;
  float normal;
  //RawScore(){point=0; normal=0.f;}
};


static void HandleError( cudaError_t err,
                         const char *file,
                         int line ) {
    if (err != cudaSuccess) {
        printf( "%s in %s at line %d\n", cudaGetErrorString( err ),
                file, line );
        exit( EXIT_FAILURE );
    }
}
#define HANDLE_ERROR( err ) (HandleError( err, __FILE__, __LINE__ ))


#define HANDLE_NULL( a ) {if (a == NULL) { \
                            printf( "Host memory failed in %s at line %d\n", \
                                    __FILE__, __LINE__ ); \
                            exit( EXIT_FAILURE );}}




#endif  // __BOOK_H__
