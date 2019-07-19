#include <ros/ros.h>
#include <vector>
#include <numeric>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Point.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ras_msgs/detected_objects.h>

#include <time.h>

//static const std::string OPENCV_WINDOW = "Image window";
int cluster_step = 20;
int cluster_radius = 20;

class ImageDetect
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  //image_transport::Publisher image_pub_;
  ros::Publisher objects_pub;

public:
  ImageDetect()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/rgb/image_rect_color", 1, &ImageDetect::imageCb, this);
    //image_pub_ = it_.advertise("/image_converter/output_video", 1);
    objects_pub = nh_.advertise<ras_msgs::detected_objects>("/object/detection", 1);

    //cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageDetect()
  {
    //cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    // make a copy of the ros image in an opencv image
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    const clock_t start = clock();

    cv::Mat HSVImage;
    cv::Mat image_new;
    cv::Mat mask_purple;

    // convert image to HSV
    cv::cvtColor(cv_ptr->image, HSVImage, CV_BGR2HSV);

    // split channel
    std::vector<cv::Mat> hsv_planes;
    cv::split(HSVImage, hsv_planes);


    cv::inRange(hsv_planes[0], 125.0, 165.0, mask_purple);
    cv::threshold(mask_purple, mask_purple, 1.0, 255.0, cv::THRESH_BINARY);
    cv::threshold(hsv_planes[1], image_new, 220.0, 255.0, cv::THRESH_BINARY);//image_old.mul(mask_green_l / 255);
    image_new = image_new + mask_purple;


    // clustering

    int matrix[(640-2*cluster_radius)/cluster_step+1][(480-2*cluster_radius)/cluster_step+1];
    int ci = 0, cj;
    for(int i = cluster_radius; i < 640; i+=cluster_step) {
        cj = 0;
        for(int j = cluster_radius; j < 480; j+=cluster_step) {
            cv::Rect ROI(cv::Point(i-cluster_step, j-cluster_step), cv::Point(i+cluster_step, j+cluster_step));
            matrix[ci][cj] = cv::countNonZero(image_new(ROI));

            cj++;
        }
        ci++;
    }

    std::vector<int> tmp_centroids_x;
    std::vector<int> tmp_centroids_y;
    for(int i = 0; i < ((640-2*cluster_radius)/cluster_step+1); i++) {
        for(int j = 0; j < ((480-2*cluster_radius)/cluster_step+1); j++) {

            
            
            if(matrix[i][j] > ((double) (cluster_radius * cluster_radius) * (3.0/4.0))) {

                int count = 0;
                if(i - 1 >= 0) {
                    if(matrix[i-1][j] > matrix[i][j])
                        continue;
                }
                if(j - 1 >= 0) {
                    if(matrix[i][j-1] > matrix[i][j])
                        continue;
                }
                if(i + 1 < ((640-2*cluster_radius)/cluster_step+1)) {
                    if(matrix[i+1][j] > matrix[i][j])
                        continue;
                }
                if(j + 1 < ((480-2*cluster_radius)/cluster_step+1)) {
                    if(matrix[i][j+1] > matrix[i][j])
                        continue;
                }

                if(i - 1 >= 0 && j - 1 >= 0) {
                    if(matrix[i-1][j-1] > matrix[i][j])
                        continue;
                }
                if(i + 1 < ((640-2*cluster_radius)/cluster_step+1) && j - 1 >= 0) {
                    if(matrix[i+1][j-1] > matrix[i][j])
                        continue;
                }
                if(i - 1 >= 0 && j + 1 < ((480-2*cluster_radius)/cluster_step+1)) {
                    if(matrix[i-1][j+1] > matrix[i][j])
                        continue;
                }
                if(i + 1 < ((640-2*cluster_radius)/cluster_step+1) && j + 1 < ((480-2*cluster_radius)/cluster_step+1)) {
                    if(matrix[i+1][j+1] > matrix[i][j])
                        continue;
                }

                tmp_centroids_x.push_back(cluster_radius+cluster_step*i);
                tmp_centroids_y.push_back(cluster_radius+cluster_step*j);
            }
        }
    }

    std::vector<int> centroids_x;
    std::vector<int> centroids_y;
    std::vector<int> tmp_x;
    std::vector<int> tmp_y;
    for(int i = 0; i < tmp_centroids_x.size(); i++) {

        tmp_x.push_back(tmp_centroids_x.at(i));
        tmp_y.push_back(tmp_centroids_y.at(i));
        for(int j = i+1; j < tmp_centroids_x.size(); j++) {
            for(int k = 0; k < tmp_x.size(); k++) {
                if(abs(tmp_x.at(k) - tmp_centroids_x.at(j)) <= 5 * cluster_radius && abs(tmp_y.at(k) - tmp_centroids_y.at(j)) <= 5 * cluster_radius) {
                    tmp_x.push_back(tmp_centroids_x.at(j));
                    tmp_y.push_back(tmp_centroids_y.at(j));
                    tmp_centroids_x.erase(tmp_centroids_x.begin() + j);
                    tmp_centroids_y.erase(tmp_centroids_y.begin() + j);
                    j--;
                    k = tmp_x.size();
                }
            }
        }
        if(tmp_x.size() > 0) {
		    int x_new = std::accumulate( tmp_x.begin(), tmp_x.end(), 0.0) / tmp_x.size();
		    int y_new = std::accumulate( tmp_y.begin(), tmp_y.end(), 0.0) / tmp_y.size();
		    centroids_x.push_back(x_new);
		    centroids_y.push_back(y_new);
		    //cv::circle(cv_ptr->image, cv::Point(x_new, y_new), cluster_radius, CV_RGB(255,0,0));
		    tmp_x.clear();
		    tmp_y.clear();
        }
    }

    //send all the centroids in cetroids_x and centroids_y
    if(centroids_x.size() > 0) {
	    ras_msgs::detected_objects centroids;
	    geometry_msgs::Point p;

	    centroids.N_points = centroids_x.size();
	    for(int i = 0; i < centroids_x.size(); i++) {
	        p.x = centroids_x.at(i);
	        p.y = centroids_y.at(i);
	        p.z = 0;
	        centroids.objects.push_back(p);
	    }
	    objects_pub.publish(centroids);
    }



/*
    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, image_new); // cv_ptr->image
    cv::waitKey(3);

    cv_bridge::CvImage out_msg;
    out_msg.header   = msg->header; // Same timestamp and tf frame as input image
    out_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1; // Or whatever
    out_msg.image    = image_new; // Your cv::Mat


    // Output modified video stream
    image_pub_.publish(out_msg.toImageMsg());
    */

  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_detection");
  ImageDetect ic;
  ros::Duration(5.0).sleep();
  ros::spin();
  return 0;
}



