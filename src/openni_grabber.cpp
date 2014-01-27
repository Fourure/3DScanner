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
			full_point_cloud_ = new pcl::PointCloud<pcl::PointXYZRGBA>;
		}
		
		void convertToCvMat(const openni_wrapper::DepthImage::ConstPtr dim, cv::Mat& frameDepth)
		{
			dim->fillDepthImage(frameDepth.cols,frameDepth.rows, (float*)frameDepth.data,frameDepth.step);
		}
		
		void convertToCvMat(const openni_wrapper::Image::ConstPtr im, cv::Mat& frameRGB)
		{
			im->fillRGB(frameRGB.cols,frameRGB.rows,frameRGB.data,frameRGB.step);
		}
	
		void camerasCallback (	const boost::shared_ptr<openni_wrapper::Image>& im, 
							const boost::shared_ptr<openni_wrapper::DepthImage>& dim, 
							float constant	  )
		{
			focal = 1.f/constant;
			
			tmp_rgb_ = rgb_;
			
			convertToCvMat(dim, depth_);
			convertToCvMat(im, rgb_);
			cv::cvtColor(rgb_, rgb_, CV_RGB2BGR);
			
			computeTransformation();
		}
		
		void transformPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>* pc, const cv::Mat& rot, cv::Mat& tr)
		{
			for(int i=0; i<pc->points.size(); i++)
			{
				double x = pc->points[i].x;
				double y = pc->points[i].y;
				double z = pc->points[i].z;
				
				pc->points[i].x = x*rot.at<double>(0, 0) + y*rot.at<double>(0, 1) + z*rot.at<double>(0, 2) + tr.at<double>(0, 0);
				pc->points[i].y = x*rot.at<double>(1, 0) + y*rot.at<double>(1, 1) + z*rot.at<double>(1, 2) + tr.at<double>(1, 0);
				pc->points[i].z = x*rot.at<double>(2, 0) + y*rot.at<double>(2, 1) + z*rot.at<double>(2, 2) + tr.at<double>(2, 0);
			}
		}
		
		//Gets transform between clouds and call for reconstruction
		void computeTransformation()
		{
			//-- Step 1: Detect the keypoints using SURF Detector
			int minHessian = 400;
			cv::SurfFeatureDetector detector( minHessian );
			std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
			detector.detect( rgb_, keypoints_1 );
			detector.detect( tmp_rgb_, keypoints_2 );

			//-- Step 2: Calculate descriptors (feature vectors)
			cv::SurfDescriptorExtractor extractor;
			cv::Mat descriptors_1, descriptors_2;
					
			extractor.compute( rgb_, keypoints_1, descriptors_1 );
			extractor.compute( tmp_rgb_, keypoints_2, descriptors_2 );

			//-- Step 3: Matching descriptor vectors using FLANN matcher
			cv::FlannBasedMatcher matcher;
			std::vector< cv::DMatch > matches;
			matcher.match( descriptors_1, descriptors_2, matches );

			double max_dist = 0; double min_dist = 0.00001;

			//-- Quick calculation of max and min distances between keypoints
			for( int i = 0; i < descriptors_1.rows; i++ )
			{ 
				double dist = matches[i].distance;
				if( dist < min_dist ) min_dist = dist;
				if( dist > max_dist ) max_dist = dist;
			}

			//-- Get sets of points from image left and right
			std::vector<cv::Point2f> points1, points2;
			std::vector< cv::DMatch > good_matches;
			for( int i = 0; i < descriptors_1.rows; i++ )
			{
				if( matches[i].distance <= min_dist )
				{
					good_matches.push_back(matches[i]);
					points1.push_back(keypoints_1[matches[i].trainIdx].pt);
					points2.push_back(keypoints_2[matches[i].queryIdx].pt);
				}
			}
			
			cv::Mat img_matches;
			cv::drawMatches( rgb_, keypoints_1, tmp_rgb_, keypoints_2,
					good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
					std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

			cv::imshow( "Matches", img_matches );

			//-- Fundamental Matrix  x'Fx = 0
			//-- Where x = point in image 1 and x' = match in image 2 (homogenous coordinates)
			cv::Mat fundamental = cv::findFundamentalMat(points1, points2, CV_FM_LMEDS);

			//-- Intrinsic parameters from the camera
			cv::Mat intrinsics(3, 3, CV_64FC1, cv::Scalar(0));
			intrinsics.at<double>(0, 2) = 320;
			intrinsics.at<double>(1, 2) = 240;
			intrinsics.at<double>(0, 0) = 525;
			intrinsics.at<double>(1, 1) = 525;
			intrinsics.at<double>(2, 2) = 1;

			//-- Essential matrix  y'Ey = 0
			//-- y, y' normalized image coordinates (see wikipedia)
			cv::Mat essential = intrinsics.t() * fundamental * intrinsics;
			
			//-- SVD decomposition
			//-- gives U, Vt and Sigma matrices (no idea dafuq they correspond to)
			cv::Mat u, sigma_diag, vt;
			cv::SVD svd;
			svd.compute(essential, sigma_diag, u, vt);
			
			cv::Mat sigma(3, 3, CV_64FC1, cv::Scalar(0));
			sigma.at<double>(0, 0) = sigma_diag.at<double>(0, 0);
			sigma.at<double>(1, 1) = sigma_diag.at<double>(1, 0);
			
			//-- Builds a W matrix (Wt = W-1)
			//     0 -1  0
			// W = 1  0  0
			//     0  0  1
			cv::Mat w(3, 3, CV_64FC1, cv::Scalar(0));
			w.at<double>(1, 0) = 1;
			w.at<double>(0, 1) = -1;
			w.at<double>(2, 2) = 1;
			
			//-- Compute rotation matrix R
			//-- R = U Wt Vt
			cv::Mat rotation = u * w.t() * vt;

			//-- Compute matrix [t]x
			//-- [t]x = v * Z * Vt
			// with
			//     0 -1  0
			// Z = 1  0  0
			//     0  0  0
			cv::Mat z = w;
			z.at<double>(2, 2) = 0;
			cv::Mat tx = vt.t() * z * vt;
			cv::Mat translation(3, 1, CV_64FC1, cv::Scalar(0));

			//-- Compute Vector T
			//         0  -tz  ty
			// [t]x = tz  0  -tx
			//       -ty  tx  0
			// T = tx ty tz
			/*translation.at<double>(0, 0) = tx.at<double>(2, 1);
			translation.at<double>(1, 0) = tx.at<double>(0, 2);
			translation.at<double>(2, 0) = tx.at<double>(1, 0);
			
			for(int i=0; i<3; i++)
				for(int j=0; j<3; j++)
					if (rotation.at<double>(i, j) < 0.0001)
						rotation.at<double>(i, j)  = 0;

			std::cout << "Rotation matrix" << std::endl;
			std::cout << rotation << std::endl;
			std::cout << "Translation vector" << std::endl;
			std::cout << translation << std::endl;*/
			
			transformPointCloud(frame_point_cloud_, rotation, translation);
			*full_point_cloud_ += *frame_point_cloud_;
			
			show();
		}
		
		//Display Flows
		void show()
		{
			cv::imshow("RGB", rgb_);
			cv::imshow("Depth", depth_);
			cv::waitKey(1);
			
			viewer.showCloud(boost::shared_ptr < pcl::PointCloud<pcl::PointXYZRGBA> >( new pcl::PointCloud<pcl::PointXYZRGBA>(*full_point_cloud_)));
		}
		
		//called if the XTION point cloud is used
		void cloudCallback (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
		{
			delete frame_point_cloud_;
			frame_point_cloud_ = new pcl::PointCloud<pcl::PointXYZRGBA>(*cloud);
			show();
		}

		void run ()
		{
			pcl::OpenNIGrabber* interface = new pcl::OpenNIGrabber();
			
			boost::function<void (const boost::shared_ptr<openni_wrapper::Image>&, 
				const boost::shared_ptr<openni_wrapper::DepthImage>&, 
				float constant)>  nc_cb = boost::bind (&SimpleOpenNIViewer::camerasCallback, this, _1, _2, _3);
			interface->registerCallback (nc_cb);
			
			#ifdef XTION_CLOUD
			boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> c_cb =
				boost::bind (&SimpleOpenNIViewer::cloudCallback, this, _1);
			interface->registerCallback (c_cb);
			#endif
			
			cu = 320;
			cv = 240;
			
			name = "i";
			
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
		pcl::PointCloud<pcl::PointXYZRGBA>* full_point_cloud_;
		
		double focal, cu, cv;
		
		std::string name;
		cv::Mat tmp_rgb_; //last frame, used for SIFT transformation computing
};

int main ()
{ 
	SimpleOpenNIViewer v;
	v.run ();
	
	return 0;
}
