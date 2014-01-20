#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/image_viewer.h>
 
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

#define XTION_CLOUD

class SimpleOpenNIViewer
{
	public:
		SimpleOpenNIViewer ()  : viewer("Point Cloud"), rgb_(480, 640, CV_8UC3), depth_(480, 640, CV_32FC1), tmp_rgb_(480, 640, CV_8UC3) 
		{
			frame_point_cloud_ = new pcl::PointCloud<pcl::PointXYZRGBA>;
		}
		
		void convertToCvMat(const openni_wrapper::DepthImage::ConstPtr dim, cv::Mat& frameDepth)
		{
			dim->fillDepthImage(frameDepth.cols,frameDepth.rows, (float*)frameDepth.data,frameDepth.step);
		}
		
		void convertToCvMat(const openni_wrapper::Image::ConstPtr im, cv::Mat& frameRGB)
		{
			im->fillRGB(frameRGB.cols,frameRGB.rows,frameRGB.data,frameRGB.step);
		}

		//Gets transform between clouds and call for reconstruction
		void camerasCallback (	const boost::shared_ptr<openni_wrapper::Image>& im, 
							const boost::shared_ptr<openni_wrapper::DepthImage>& dim, 
							float constant	  )
		{
			tmp_rgb_ = rgb_;
			
			convertToCvMat(dim, depth_);
			convertToCvMat(im, rgb_);
			cv::cvtColor(rgb_, rgb_, CV_RGB2BGR);
			
			computeTransformation();
		}
		
		void computeTransformation()
		{
			cv::SiftFeatureDetector detector( 0.05, 5.0 );
			cv::SiftDescriptorExtractor extractor( 3.0 );
			
			cv::Mat im1 = tmp_rgb_;
			cv::Mat im2 = rgb_;
			
			std::vector<cv::KeyPoint> key1, key2;
			detector.detect( im1, key1 );
			detector.detect( im2, key2 );
			
			drawKeypoints(im1,key1,rgb_,cv::Scalar(255, 0, 0));
			
			cv::Mat desc1, desc2;
			extractor.compute( im1, key1, desc1 );
			extractor.compute( im2, key2, desc2 );
			
			cv::FlannBasedMatcher matcher;
			std::vector< cv::DMatch > matches;
			matcher.match( desc1, desc2, matches );
			
			cv::Mat result;
			drawMatches( im1, key1, im2, key2, matches, result );
			
			//cv::imwrite("toto.png", result); Works as hell =D !
			
			show();
		}
		
		//Display Flows
		void show()
		{
			cv::imshow("RGB", rgb_);
			cv::imshow("Depth", depth_);
			cv::waitKey(1);
		}
		
		//called if the XTION point cloud is used
		void cloudCallback (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
		{
			viewer.showCloud(cloud); //to move in show but f****** boost and its denominations prevent me from doing it
			delete frame_point_cloud_;
			frame_point_cloud_ = new pcl::PointCloud<pcl::PointXYZRGBA>(*cloud);
			show();
		}

		void run ()
		{
			pcl::Grabber* interface = new pcl::OpenNIGrabber();
			
			boost::function<void (const boost::shared_ptr<openni_wrapper::Image>&, 
				const boost::shared_ptr<openni_wrapper::DepthImage>&, 
				float constant)>  nc_cb = boost::bind (&SimpleOpenNIViewer::camerasCallback, this, _1, _2, _3);
			interface->registerCallback (nc_cb);
			
			#ifdef XTION_CLOUD
			boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> c_cb =
				boost::bind (&SimpleOpenNIViewer::cloudCallback, this, _1);
			interface->registerCallback (c_cb);
			#endif
			
			interface->start();

			while (!viewer.wasStopped())
				boost::this_thread::sleep (boost::posix_time::seconds (1));

			interface->stop();
		}
		
		//cloud viewer, pcl facilities
		pcl::visualization::CloudViewer viewer;
		cv::Mat rgb_;
		cv::Mat depth_;
		pcl::PointCloud<pcl::PointXYZRGBA>* frame_point_cloud_;
		
		cv::Mat tmp_rgb_; //last frame, used for SIFT transformation computing
};

int main ()
{ 
	SimpleOpenNIViewer v;
	v.run ();
	
	return 0;
}
