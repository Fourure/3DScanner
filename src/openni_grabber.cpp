#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/image_viewer.h>
 
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

class SimpleOpenNIViewer
{
	public:
		SimpleOpenNIViewer ()  : viewer("Point Cloud") {}
		
		cv::Mat convertToCvMat(const openni_wrapper::DepthImage::ConstPtr dim)
		{
			cv::Mat frameDepth(dim->getHeight(),dim->getWidth(), CV_32FC1);
			
			dim->fillDepthImage(frameDepth.cols,frameDepth.rows, (float*)frameDepth.data,frameDepth.step);
			
			return frameDepth;
		}
		
		cv::Mat convertToCvMat(const openni_wrapper::Image::ConstPtr im)
		{
			cv::Mat frameRGB(im->getHeight(),im->getWidth(), CV_8UC3);
			
			im->fillRGB(frameRGB.cols,frameRGB.rows,frameRGB.data,frameRGB.step);
			
			return frameRGB;
		}

		void camerasCallback (	const boost::shared_ptr<openni_wrapper::Image>& im, 
							const boost::shared_ptr<openni_wrapper::DepthImage>& dim, 
							float constant	  )
		{
			cv::Mat depth_image_ = convertToCvMat(dim);
			cv::Mat rgb_image_ = convertToCvMat(im);
			
			cv::cvtColor(rgb_image_, rgb_image_, CV_RGB2BGR );
			
			cv::imshow("RGB", rgb_image_);
			cv::imshow("Depth", depth_image_);
			cv::waitKey(1);
		}
		
		void cloudCallback (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
		{
			if (!viewer.wasStopped())
				viewer.showCloud (cloud);
		}

		void run ()
		{
			pcl::Grabber* interface = new pcl::OpenNIGrabber();
			
			boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> c_cb =
				boost::bind (&SimpleOpenNIViewer::cloudCallback, this, _1);
			
			boost::function<void (const boost::shared_ptr<openni_wrapper::Image>&, 
				const boost::shared_ptr<openni_wrapper::DepthImage>&, 
				float constant)>  nc_cb = boost::bind (&SimpleOpenNIViewer::camerasCallback, this, _1, _2, _3);
			
			interface->registerCallback (nc_cb);
			interface->registerCallback (c_cb);
		     
			interface->start();

			while (!viewer.wasStopped())
				boost::this_thread::sleep (boost::posix_time::seconds (1));

			interface->stop();
		}
		
		pcl::visualization::CloudViewer viewer;
};

int main ()
{ 
	SimpleOpenNIViewer v;
	v.run ();
	
	return 0;
}
