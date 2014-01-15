#ifndef MANDELBROT_HH
#define MANDELBROT_HH

#include <opencv2/core/core.hpp>

#define MAX_ITER 3000

int iterations(double x0, double y0)
{
  double x=x0, y=y0; int n;
  for(n=0; n<MAX_ITER && x*x+y*y <= 4.0; ++n) { double t=x*x-y*y+x0; y=2.0*x*y+y0; x=t; }
  return n ;
}

void makeMandelbrot(cv::Mat& img, const float x0, const float y0, const float radius)
{
	for (int i=0; i<img.rows; ++i)
		for (int j=0; j<img.cols; ++j)
		{
			int iter = iterations(x0+radius*(j-img.cols/2), y0 + radius*(i-img.rows/2));
			img.data[(i*img.cols+j)*3+0] = 128+127*sin(0+0.07*iter);
			img.data[(i*img.cols+j)*3+1] = 128+127*sin(1+0.11*iter);
			img.data[(i*img.cols+j)*3+2] = 128+127*sin(2+0.13*iter);
		}		
}


#endif