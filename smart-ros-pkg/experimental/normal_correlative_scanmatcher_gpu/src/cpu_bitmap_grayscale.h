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


#ifndef __CPU_BITMAP_H__
#define __CPU_BITMAP_H__

#include "gl_helper.h"

struct CPUBitmapFloat {
    float    *pixels;
    int     x, y;
    CPUBitmapFloat(){};
    void copyImg(int w, int h, float *f) {
        pixels = new float[w*h];
        x = w;
        y = h;
        pixels = f;
        printf("Copied %d %d\n", x, y);
    }

    CPUBitmapFloat( int width, int height) {
        pixels = new float[width * height];
        x = width;
        y = height;
    }

    ~CPUBitmapFloat() {
        delete [] pixels;
    }

    long image_size( void ) const { return x * y * sizeof(float); }
    float* get_ptr( void ) const   { return pixels; }
};

struct CPUBitmap {
    unsigned char    *pixels;
    float *normals_map;
    int     x, y, rot_size;
    CPUBitmap(){};
    void copyImg(int w, int h, unsigned char *f) {
        pixels = new unsigned char[w*h];
        x = w;
        y = h;
        pixels = f;
        printf("Copied %d %d\n", x, y);
    }

    CPUBitmap( int width, int height) {
        pixels = new unsigned char[width * height];
	normals_map = new float[width*height];
        x = width;
        y = height;
    }

    ~CPUBitmap() {
        delete [] pixels;
	delete [] normals_map;
    }

    unsigned char* get_ptr( void ) const   { return pixels; }
    float* get_normals_ptr( void ) const   { return normals_map;}
    long image_size( void ) const { return x * y; }
};


struct mapData {
    unsigned char* point_map;
    float* normal_map;
    int pm_x, pm_y;
    int nm_x, nm_y;
};

#endif  // __CPU_BITMAP_H__
