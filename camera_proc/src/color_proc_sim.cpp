#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/video/tracking.hpp>
#include <stdio.h>

#include <math.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <sstream>
#include <string>
#include <iostream>
#include <vector>
#include <sys/time.h>


using namespace std;
using namespace cv;
static const std::string OPENCV_WINDOW = "Image window";
int frame_no = 0 ;
int ptx = 0 ;
int pty = 0 ;
float h_obj = 0.0;
double last_c_loc =0.0;
double last_g_loc = 0.0;
double c_x = 0.0;
double c_y = 0.0;
double c_loc = 0.0;
double c_d = 0.0;
double c_sxx =0.0;
double c_sxy =0.0;
double c_syx =0.0;
double c_syy =0.0;

double g_x = 0.0;
double g_y = 0.0;
double g_loc = 0.0;
double g_d = 0.0;
double g_sxx =0.0;
double g_sxy =0.0;
double g_syx =0.0;
double g_syy =0.0;


Point p, s, Hcp ,Hcs;
#define PI 3.14159265
cv::KalmanFilter KF;
cv::Mat_<float> measurement(2,1);
Mat_<float> state(4, 1);

bool DetectWrenchs = true;
bool DetectValve = true;
bool HoughCirclesUse = true;
bool HoughLinesUse = false;
std::vector<float> results;
std::vector<long int> myTime;
std::vector<long int> mynTime;
geometry_msgs::PoseWithCovarianceStamped color_msg;
geometry_msgs::PoseWithCovarianceStamped geometry_msg;

class ImageConverter
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::Publisher color_pub;
    ros::Publisher geometry_pub;

    Point kalmanPredict()
    {
        Mat prediction = KF.predict();
        Point predictPt(prediction.at<float>(0),prediction.at<float>(1));
        return predictPt;
    }
    Point kalmanCorrect(float x, float y)
    {
        measurement(0) = x;
        measurement(1) = y;
        Mat estimated = KF.correct(measurement);
        Point statePt(estimated.at<float>(0),estimated.at<float>(1));
        return statePt;
    }
public:
    ImageConverter()
        : it_(nh_)
    {

        image_sub_ = it_.subscribe("usb_cam/image_raw", 1,
                                   &ImageConverter::imageCb, this);
        color_pub = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped >("/target/camera_color", 1000);
        geometry_pub = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped >("/target/camera_geom", 1000);

        // namedWindow("seg", 0);
        // namedWindow("feed", 0);
    }
    void overlayImage(Mat* src, Mat* overlay, const Point& location)
    {
        for (int y = max(location.y, 0); y < src->rows; ++y)
        {
            int fY = y - location.y;

            if (fY >= overlay->rows)
                break;

            for (int x = max(location.x, 0); x < src->cols; ++x)
            {
                int fX = x - location.x;

                if (fX >= overlay->cols)
                    break;

                double opacity = ((double)overlay->data[fY * overlay->step + fX * overlay->channels() + 3]) / 255;

                for (int c = 0; opacity > 0 && c < src->channels(); ++c)
                {
                    unsigned char overlayPx = overlay->data[fY * overlay->step + fX * overlay->channels() + c];
                    unsigned char srcPx = src->data[y * src->step + x * src->channels() + c];
                    src->data[y * src->step + src->channels() * x + c] = srcPx * (1. - opacity) + overlayPx * opacity;
                }
            }
        }
    }
    ~ImageConverter()
    {
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        //get header info
        std_msgs::Header h = msg->header;






        int Theight = msg-> height;
        int Twidth = msg-> width;
        int Total_pixels = Theight*Twidth;
        // cout << "height x width = " << Theight <<" x " << Twidth <<" = " << Total_pixels << endl << endl;

        // cout<<h<<endl; //all at once
        // cout<<h.stamp<<endl; //specific parts of it
        // cout<<h.stamp.sec<<endl;
        // cout<<h.stamp.nsec<<endl;
        // cout<<h.seq<<endl;

        // Image processing
        /////////////////////////////////////////////////////declearation////////////
        Mat frame, FrameOut, invFrameOut,test;
        frame = cv_ptr->image;
        /////////////////////////////////////////////// Image Enhancement////////////
        int rec_x = 850;
        int rec_y = 855;
        int rec_width = 200;
        int rec_height = 220;
        Mat mask_frame(rec_height,rec_width, CV_8UC3, Scalar(255,255,255));
        Mat mask_frame2(20,1920, CV_8UC3, Scalar(255,255,255));

        // and its top left corner...
        cv::Point pt1(rec_x, rec_y);
        // and its bottom right corner.
        cv::Point pt2(rec_x + rec_width, rec_y + rec_height);
        //        cv::rectangle(frame, pt1, pt2, Scalar(255, 255, 255), -1, 8);
        //        rectangle( frame, Point(25,300), 800, Scalar( 0, 0, 255 ), -1, 2 );
        overlayImage( &frame, &mask_frame, pt1 );
        overlayImage( &frame, &mask_frame2, Point(0,1060) );

        frame.convertTo(FrameOut, -1, 2, -50); //increase the contrast (double)
        //        FrameOut.convertTo(FrameOut, -1, 2, -50); //increase the contrast (double)
        //        FrameOut.convertTo(FrameOut, -1, 2, 0); //increase the contrast (double)





        test = FrameOut;
        cvtColor(FrameOut, FrameOut, CV_BGR2GRAY);
        //        GaussianBlur(FrameOut, FrameOut, Size(9,9), 1, 1);
        threshold( FrameOut, FrameOut, 2, 255,1 );
        int Mo_size = 20;
        cv::erode(FrameOut, FrameOut, getStructuringElement(MORPH_ELLIPSE, Size(Mo_size, Mo_size)) );
        cv::dilate(FrameOut, FrameOut, getStructuringElement(MORPH_ELLIPSE, Size(Mo_size, Mo_size)) );

        cv::dilate(FrameOut, FrameOut, getStructuringElement(MORPH_ELLIPSE, Size(Mo_size, Mo_size)) );
        cv::erode(FrameOut, FrameOut, getStructuringElement(MORPH_ELLIPSE, Size(Mo_size, Mo_size)) );
        ////////////////////////////////////////////
        //     if(0)
        //     {
        //     const int connectivity_8 = 8;
        //     Mat labels, stats, centroids;
        //     int nLabels = connectedComponentsWithStats(FrameOut, labels, stats, centroids, connectivity_8, CV_32S)-1;
        //     // cout << "Number of connected components = " << nLabels << endl << endl;
        //     // cout << "CC_STAT_AREA   = " << stats.at<int>(1,CC_STAT_AREA) << endl;


        //     int area = 0;
        //     if (nLabels > 0 )
        //     {
        //         for (int i=0;i<=nLabels;i++)
        //         {
        //             if(stats.at<int>(i+1,CC_STAT_AREA) > 600 && stats.at<int>(i+1,CC_STAT_AREA) < Total_pixels){
        //                 area = area + stats.at<int>(i+1,CC_STAT_AREA);
        //             }

        //         }
        //     }
        //     int a = 10;
        //     int b = 100;
        //     float dArea = (float)area / (float)Total_pixels;
        //     long int secs = h.stamp.sec;
        //     long int nsecs = h.stamp.nsec;
        //     //        cout << "dArea   = " << dArea << endl;


        //     mynTime.push_back(nsecs);
        //     myTime.push_back(secs);
        //     results.push_back(dArea);
        // }
        // std::cout << "saving " << results.back() << std::endl;
        // cout << "BOX_AREA   = " << area << endl;
        ////////////////////////////////////////////
        bitwise_not ( FrameOut, invFrameOut );
        // Setup SimpleBlobDetector parameters.
        SimpleBlobDetector::Params params;
        // Change thresholds
        // params.minThreshold = 10;
        // params.maxThreshold = 200;

        // Filter by Area.
        params.filterByArea = false;
        params.minArea = 200;
        params.maxArea = 100000000;

        // Filter by Circularity
        params.filterByCircularity = false;
        //        params.minCircularity = 0.1;

        // Filter by Convexity
        params.filterByConvexity = false;
        //        params.minConvexity = 0.00;

        // Filter by Inertia
        params.filterByInertia = false;
        //        params.minInertiaRatio = 0.5;

        std::vector<KeyPoint> keypoints;

#if CV_MAJOR_VERSION < 3
        // Set up detector with params
        SimpleBlobDetector detector(params);

        detector.detect( invFrameOut, keypoints);
#else
        // Set up detector with params
        Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);
        // std::vector<KeyPoint> keypoints;
        detector->detect( invFrameOut, keypoints);
#endif
        // Storage for blobs
        // std::vector<KeyPoint> keypoints;
        // detector.detect( invFrameOut, keypoints);
        drawKeypoints( frame, keypoints, frame, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
        p = kalmanPredict();

        for(std::vector<cv::KeyPoint>::iterator blobIterator = keypoints.begin(); blobIterator != keypoints.end(); blobIterator++){
            ptx = blobIterator->pt.x;
            pty = blobIterator->pt.y;
            h_obj = blobIterator->size;


            //            std::cout << "h_obj size : " << h_obj << std::endl;
        }
        // The "correct" phase that is going to use the predicted value and our measurement
        s = kalmanCorrect(ptx, pty);
        // std::cout << "Box detected at: " << p.x << " " << p.y  << std::endl;
        // std::cout << "Box estimated (KF) at: " << s.x << " " << s.y << std::endl;

        ////////////////////////////////////////////// show data on FrameOut

        frame_no++;
        string text = static_cast<ostringstream*>( &(ostringstream() << frame_no) )->str();
        circle( frame, Point(25,30), 8, Scalar( 0, 0, 255 ), -1, 2 );
        putText(frame, "Frames", cvPoint(40,35),
                FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0,0,255), 1, CV_AA);
        putText(frame, text, cvPoint(120,35),
                FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0,0,255), 1, CV_AA);
        putText(frame, "Box estimated (KF) at: ", cvPoint(20,50),
                FONT_HERSHEY_COMPLEX_SMALL, 0.6, cvScalar(0,0,0), 1, CV_AA);
        text = static_cast<ostringstream*>( &(ostringstream() << s.x) )->str();
        putText(frame, text , cvPoint(170,50),
                FONT_HERSHEY_COMPLEX_SMALL, 0.6, cvScalar(0,0,0), 1, CV_AA);
        text = static_cast<ostringstream*>( &(ostringstream() << s.y) )->str();
        putText(frame, text , cvPoint(200,50),
                FONT_HERSHEY_COMPLEX_SMALL, 0.6, cvScalar(0,0,0), 1, CV_AA);
        if(ptx && pty)
        {
            circle( frame, p, 10, Scalar( 255, 0, 0 ), +2, 2 );
            circle( frame, s, 5, Scalar( 0, 255, 0 ), -1, 2 );
        }

        // Update GUI Window
        //imshow("Output stream", imthresholdw);
        //imshow("feed", frame);
        //imshow("seg", FrameOut);
        //imshow("feed", frame);

        float error = (float) rand()/RAND_MAX;

        float ry = 1.0 / h_obj;
        float theta = (90.0 * h_obj)/frame.rows;
        float FOV = 90;
        float the_w = ((FOV*s.x)-(frame.cols*0.5*FOV))/(frame.cols);
        float Epsilon = 0.1;

        c_d = (h_obj * ry )/( tan ( theta * PI / 180.0 ));
        c_x = c_d * cos(the_w * PI / 180.0);
        c_y = -1* c_d * sin(the_w * PI / 180.0);
        g_d = c_d + error;
        g_x = g_d * cos(the_w * PI / 180.0);
        g_y = -1* g_d * sin(the_w * PI / 180.0);
        c_loc = 0.8 * exp(-1*c_d/20);
        g_loc = 0.6 * exp(-1*g_d/20);
        //c_loc = 0.5 - 0.025 * c_d ; // loc@d=20 is 0 & loc@d=0 is 0.5
        //g_loc = 0.5 - 0.025 * g_d ; // loc@d=20 is 0 & loc@d=0 is 0.5


        // modification////////////////////////////////////
        if (c_loc < Epsilon || isnan(c_loc)  ){
            c_loc = 0.0;
            last_c_loc = c_loc;
            g_loc = 0.0;
            c_x = 0.0;
            c_y = 0.0;
        }
        if (c_loc == last_c_loc ){
            c_loc = 0.0;
            g_loc = 0.0;
            c_x = 0.0;
            c_y = 0.0;
            c_sxx = pow(10.0,0.0);
            c_sxy = 0.0; //pow(10.0,0.0);
            c_syx = 0.0; //pow(10.0,0.0);
            c_syy = pow(10.0,0.0);
        }else{
            last_c_loc = c_loc;
            c_sxx = (1.0)/(1 + 10.0*c_loc);
            c_syy = (0.00001)/(1 + 10.0*c_loc);
            c_syx = 0.0;
            c_sxy = 0.0;

        }

        if (g_loc < Epsilon || isnan(g_loc)  ){
            g_loc = 0.0;
            last_g_loc = g_loc;
            g_x = 0.0;
            g_y = 0.0;
        }
        if (g_loc == last_g_loc ){
            g_loc = 0.0;
            g_x = 0.0;
            g_y = 0.0;
            g_sxx = pow(10.0,0.0);
            g_sxy = 0.0; //pow(10.0,0.0);
            g_syx = 0.0; //pow(10.0,0.0);
            g_syy = pow(10.0,0.0);
        }else{
            last_g_loc = g_loc;
            g_sxx = (1.0)/(1 + 10.0*g_loc);
            g_syy = (0.00001)/(1 + 10.*g_loc);
            g_syx = 0.0;
            g_sxy = 0.0;
        }
       //std::cout << c_loc <<","<<  g_loc << std::endl;

        /////////////////////////////////////////////////

        //your modification///////////////////
//        if (c_loc != c_loc) {
//            c_loc = 0;
//        }
//        if (g_loc != g_loc) {
//            g_loc = 0;
//        }
//        if (c_loc < Epsilon){
//            c_loc = 0.0;
//            c_sxx = pow(10.0,6.0);
//            c_sxy = pow(10.0,6.0);
//            c_syx = pow(10.0,6.0);
//            c_syy = pow(10.0,6.0);
//        } else {
//            c_sxx = (1)/(1 + c_loc);
//            c_syy = (0.1)/(1 + c_loc);
//        }

//        if (g_loc < Epsilon){
//            g_loc = 0.0;
//            g_sxx = pow(10.0,6.0);
//            g_sxy = pow(10.0,6.0);
//            g_syx = pow(10.0,6.0);
//            g_syy = pow(10.0,6.0);
//        } else {
//            g_sxx = (1)/(1 + g_loc);
//            g_syy = (0.1)/(1 + g_loc);
//        }
        ////////////////////////////////////////////////

        // transformation
        //        tf2_ros::Buffer tfBuffer;
        //        tf2_ros::TransformListener tfListener(tfBuffer);
        //        geometry_msgs::TransformStamped gripper_cam_to_base_link;
        //        try{
        //            gripper_cam_to_base_link = tfBuffer.lookupTransform("base_link", "gripper_camera", ros::Time(0));
        //        }
        //        catch (tf2::TransformException &ex) {
        //            ROS_WARN("%s",ex.what());
        //            //          ros::Duration(0.1).sleep();
        //            //          continue;
        //        }

        color_msg.header = h;
        color_msg.pose.pose.position.x = c_x;
        color_msg.pose.pose.position.y = c_y;
        color_msg.pose.pose.position.z = c_loc;
        color_msg.pose.covariance[0]  = c_sxx;
        color_msg.pose.covariance[1]  = c_sxy;
        color_msg.pose.covariance[6]  = c_syx;
        color_msg.pose.covariance[7]  = c_syy;


        geometry_msg.header = h;
        geometry_msg.pose.pose.position.x = g_x;
        geometry_msg.pose.pose.position.y = g_y;
        geometry_msg.pose.pose.position.z = g_loc;
        geometry_msg.pose.covariance[0]  = g_sxx;
        geometry_msg.pose.covariance[1]  = g_sxy;
        geometry_msg.pose.covariance[6]  = g_syx;
        geometry_msg.pose.covariance[7]  = g_syy;

        //        tf2::doTransform(color_msg.pose.pose,color_msg.pose.pose, gripper_cam_to_base_link);

        color_pub.publish(color_msg);
        geometry_pub.publish(geometry_msg);
        geometry_msg.pose.pose.position.z = 0;
        color_msg.pose.pose.position.z = 0;

        //    imshow("seg", FrameOut);
        //    imshow("stat", test);
        // if( waitKey(1) == 27 ) // exit
        // {
        //     //            std::cout << "exiting and saving" << std::endl;
        //     //            ofstream results_data("results_data.xls");
        //     //            ofstream results_time("results_time.xls");
        //     //            ofstream results_ntime("results_ntime.xls");
        //     //                        int vsize = results.size();
        //     //                        // int Tsize = myTime.size();
        //     //                        // std::cout << "vsize " <<  vsize << std::endl;
        //     //                        // std::cout << "Tsize " <<  Tsize << std::endl;
        //     //                        for(int n=0; n<vsize; n++)
        //     //                        {
        //     //                        results_ntime << mynTime[n] << '\t';
        //     //                        results_time << myTime[n] << '\t';
        //     //                        results_data << results[n] << '\t';
        //     ros::shutdown();
        //     //                                 }
        // }

        // Output modified video stream
        //    image_pub_.publish(cv_ptr->toImageMsg());
    }

};
void initKalman(float x, float y)
{
    // Instantate Kalman Filter with
    // 4 dynamic parameters and 2 measurement parameters,
    // where my measurement is: 2D location of object,
    // and dynamic is: 2D location and 2D velocity.
    KF.init(4, 2, 0);

    measurement = Mat_<float>::zeros(2,1);
    measurement.at<float>(0, 0) = x;
    measurement.at<float>(0, 0) = y;


    KF.statePre.setTo(0);
    KF.statePre.at<float>(0, 0) = x;
    KF.statePre.at<float>(1, 0) = y;

    KF.statePost.setTo(0);
    KF.statePost.at<float>(0, 0) = x;
    KF.statePost.at<float>(1, 0) = y;

    setIdentity(KF.transitionMatrix);
    setIdentity(KF.measurementMatrix);
    setIdentity(KF.processNoiseCov, Scalar::all(.0005)); //adjust this for faster convergence - but higher noise
    setIdentity(KF.measurementNoiseCov, Scalar::all(1e-3));
    setIdentity(KF.errorCovPost, Scalar::all(.1));
}

int main(int argc, char** argv)
{

    initKalman(0, 0);
    ros::init(argc, argv, "image_LOC");

    ImageConverter ic;
    ros::spin();
    return 0;
}
