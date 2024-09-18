/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                        Intel License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000, Intel Corporation, all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of Intel Corporation may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/

// Taken from commit 54de51ef3c6f07fb80fe1bc0e81b3b2ab8306c89


//https://codeleading.com/article/20203188854/ <cstdio>
//Shi-Tomasi 算法是Harris 算法的改进。Harris 算法最原始的定义是将矩阵 M 的行列式值与 M 的迹相减，再将差值同预先给定的阈值进行比较。后来Shi 和Tomasi 提出改进的方法，若两个特征值中较小的一个大于最小阈值，则会得到强角点。
//参考论文：hi and C. Tomasi. Good Features to Track. Proceedings of the IEEE Conference on Computer Vision and Pattern Recognition, pages 593-600, June 1994.

//todo 这里的goodFeaturesToTrack函数是根据opencv源代码改的
#include <vector>
#include <iostream>
#include <functional>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>

#include "cvmodified.h"

using namespace cv;

namespace cvmodified {

struct greaterThanPtr
{
    bool operator () (const float * a, const float * b) const
    // Ensure a fully deterministic result of the sort
    { return (*a > *b) ? true : (*a < *b) ? false : (a > b); }
};

// ----------------------------------------------------------------------------

void goodFeaturesToTrack( InputArray _image, OutputArray _corners, OutputArray _scores,
                              int maxCorners, double qualityLevel, double minDistance,
                              InputArray _mask, int blockSize, int gradientSize,
                              bool useHarrisDetector, double harrisK )
{

    CV_Assert( qualityLevel > 0 && minDistance >= 0 && maxCorners >= 0 );
    CV_Assert( _mask.empty() || (_mask.type() == CV_8UC1 && _mask.sameSize(_image)) );

    // std::cout << std::endl << "** calling modified version **" << std::endl;

    Mat image = _image.getMat(), eig, tmp;
    if (image.empty())
    {
        _corners.release();
        _scores.release();  //todo    https://ieeexplore.ieee.org/abstract/document/8526341  分数部分修改应该是对应此论文F. Real Tests: Agile Navigation on MAV 2) Techniques
        return;
    }

    if( useHarrisDetector )
        cornerHarris( image, eig, blockSize, gradientSize, harrisK );//todo
    else
        cornerMinEigenVal( image, eig, blockSize, gradientSize );//todo按这个来看最小特征值越大越好 https://blog.csdn.net/m0_37957160/article/details/107690327    https://blog.csdn.net/qq_29146525/article/details/105019022

    double maxVal = 0;
    minMaxLoc( eig, 0, &maxVal, 0, 0, _mask );
    threshold( eig, eig, maxVal*qualityLevel, 0, THRESH_TOZERO );//https://blog.csdn.net/u012566751/article/details/77046445/  cv::threshold是OpenCV中的一个函数，用于将输入的图像进行阈值处理。该函数会将小于或等于阈值的像素值置为0，大于阈值的像素值置为255。
    dilate( eig, tmp, Mat());//https://blog.csdn.net/jndingxin/article/details/113741899

    Size imgsize = image.size();
    std::vector<const float*> tmpCorners;

    // collect list of pointers to features - put them into temporary image
    Mat mask = _mask.getMat();
    for( int y = 1; y < imgsize.height - 1; y++ )
    {
        const float* eig_data = (const float*)eig.ptr(y);
        const float* tmp_data = (const float*)tmp.ptr(y);
        const uchar* mask_data = mask.data ? mask.ptr(y) : 0;

        for( int x = 1; x < imgsize.width - 1; x++ )
        {
            float val = eig_data[x];
            if( val != 0 && val == tmp_data[x] && (!mask_data || mask_data[x]) )
                tmpCorners.push_back(eig_data + x);
        }
    }

    std::vector<Point2f> corners;
    std::vector<float> scores;
    size_t i, j, total = tmpCorners.size(), ncorners = 0;

    if (total == 0)
    {
        _corners.release();
        _scores.release();
        return;
    }

    std::sort( tmpCorners.begin(), tmpCorners.end(), greaterThanPtr() );

    if (minDistance >= 1)
    {
         // Partition the image into larger grids
        int w = image.cols;
        int h = image.rows;

        const int cell_size = cvRound(minDistance);
        const int grid_width = (w + cell_size - 1) / cell_size;
        const int grid_height = (h + cell_size - 1) / cell_size;

        std::vector<std::vector<Point2f> > grid(grid_width*grid_height);

        minDistance *= minDistance;

        for( i = 0; i < total; i++ )
        {
            int ofs = (int)((const uchar*)tmpCorners[i] - eig.ptr());
            int y = (int)(ofs / eig.step);
            int x = (int)((ofs - y*eig.step)/sizeof(float));

            bool good = true;

            int x_cell = x / cell_size;
            int y_cell = y / cell_size;

            int x1 = x_cell - 1;
            int y1 = y_cell - 1;
            int x2 = x_cell + 1;
            int y2 = y_cell + 1;

            // boundary check
            x1 = std::max(0, x1);
            y1 = std::max(0, y1);
            x2 = std::min(grid_width-1, x2);
            y2 = std::min(grid_height-1, y2);

            for( int yy = y1; yy <= y2; yy++ )
            {
                for( int xx = x1; xx <= x2; xx++ )
                {
                    std::vector <Point2f> &m = grid[yy*grid_width + xx];

                    if( m.size() )
                    {
                        for(j = 0; j < m.size(); j++)
                        {
                            float dx = x - m[j].x;
                            float dy = y - m[j].y;

                            if( dx*dx + dy*dy < minDistance )
                            {
                                good = false;
                                goto break_out;
                            }
                        }
                    }
                }
            }

            break_out:

            if (good)
            {
                grid[y_cell*grid_width + x_cell].push_back(Point2f((float)x, (float)y));

                corners.push_back(Point2f((float)x, (float)y));
                scores.push_back(*tmpCorners[i]);  //todo
                ++ncorners;

                if( maxCorners > 0 && (int)ncorners == maxCorners )
                    break;
            }
        }
    }
    else
    {
        for( i = 0; i < total; i++ )
        {
            int ofs = (int)((const uchar*)tmpCorners[i] - eig.ptr());
            int y = (int)(ofs / eig.step);
            int x = (int)((ofs - y*eig.step)/sizeof(float));

            corners.push_back(Point2f((float)x, (float)y));
            scores.push_back(*tmpCorners[i]);    //todo

            ++ncorners;
            if( maxCorners > 0 && (int)ncorners == maxCorners )
                break;
        }
    }

    Mat(corners).convertTo(_corners, _corners.fixedType() ? _corners.type() : CV_32F);
    Mat(scores).convertTo(_scores, _scores.fixedType() ? _scores.type() : CV_32F);
}

// ----------------------------------------------------------------------------
void goodFeaturesToTrack( InputArray _image, OutputArray _corners, OutputArray _scores,
                          int maxCorners, double qualityLevel, double minDistance,
                          InputArray _mask, int blockSize,
                          bool useHarrisDetector, double harrisK )
{
    cvmodified::goodFeaturesToTrack(_image, _corners, _scores, maxCorners, qualityLevel, minDistance,
                        _mask, blockSize, 3, useHarrisDetector, harrisK );
}

} // namespace cvmodified
