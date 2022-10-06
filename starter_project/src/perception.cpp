#include "perception.hpp"

// ROS Headers, ros namespace
#include <ros/init.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "starter_project_perception"); // Our node name (See: http://wiki.ros.org/Nodes)

    [[maybe_unused]] mrover::Perception perception;

    // "spin" blocks until our node dies
    // It listens for new messages and calls our subscribed functions with them
    ros::spin();

    return EXIT_SUCCESS;
}

namespace mrover {

    Perception::Perception() : mNodeHandle{}, mImageTransport{mNodeHandle} {
        // Subscribe to camera image messages
        // Every time another node publishes to this topic we will be notified
        // Specifically the callback we passed will be invoked
        mImageSubscriber = mImageTransport.subscribe("camera/color/image_raw", 1, &Perception::imageCallback, this);

        // Create a publisher for our tag topic
        // See: http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
        // TODO: uncomment me!
        mTagPublisher = mNodeHandle.advertise<StarterProjectTag>("tag", 1);

        mTagDetectorParams = cv::aruco::DetectorParameters::create();
        mTagDictionary = cv::aruco::getPredefinedDictionary(0);
    }

    void Perception::imageCallback(sensor_msgs::ImageConstPtr const& image) {
        cv_bridge::CvImagePtr cvImage = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
        // Detect tags in the image pixels
        findTagsInImage(cvImage, mTags);
        // Select the tag that is closest to the middle of the screen
        StarterProjectTag tag = selectTag(mTags);
        // Publish the message to our topic so navigation or others can receive it
        publishTag(tag);
    }

    void Perception::findTagsInImage(cv_bridge::CvImagePtr const& image, std::vector<StarterProjectTag>& tags) {
        // hint: you have mTagDictionary, mTagCorners, mTagIds, and mTagDetectorParams member variables already defined!
        // hint: you can access the raw image (cv::Mat) with image->image
        // hint: write and use the "getCenterFromTagCorners" and "getClosenessMetricFromTagCorners" functions

        tags.clear(); // Clear old tags in output vector
        cv::aruco::detectMarkers(image->image, mTagDictionary, mTagCorners, mTagIds, mTagDetectorParams);
        //todo find closest

        for(size_t i = 0; i < mTagCorners.size(); ++i){
            StarterProjectTag tag;
            tag.id = mTagIds[i];
            float metric = getClosenessMetricFromTagCorners(image->image, mTagCorners[i]);
            std::pair<float, float> center = getCenterFromTagCorners(mTagCorners[i]);
            //center of image is now 0,0 bc of normalization
            tag.x = (center.first - image->image.cols/2)/image->image.cols;
            tag.y = (center.second - image->image.rows/2)/image->image.rows;
            tag.dist = metric;
            tags.push_back(tag);
        }
        std::cout << "SIZE1 " << tags.size() << "\n";

    }

    StarterProjectTag Perception::selectTag(std::vector<StarterProjectTag> const& tags) {
        // TODO: remove below & implement me!
        //(void) tags;
        std::cout << "SIZE " << tags.size()  << "\n";
        if(tags.size() < 1){
            StarterProjectTag tag;
            tag.x = -1;
            tag.y = -1;
            tag.dist = -1;
            return tag;
        }
        StarterProjectTag min = tags[0];
        float minDist = sqrt(((tags[0].x) * (tags[0].x)) + ((tags[0].y) * (tags[0].y)));;
        for(size_t i = 1; i < tags.size(); ++i){
            float dist = sqrt(((tags[i].x) * (tags[i].x)) + ((tags[i].y) * (tags[i].y)));
            if (dist < minDist){
                min = tags[i];
                minDist = dist;
            }
        }

        return min;

    }

    void Perception::publishTag(StarterProjectTag const& tag) {
        // TODO: remove below & implement me!
        mTagPublisher.publish(tag);
        
    }

    float Perception::getClosenessMetricFromTagCorners(cv::Mat const& image, std::vector<cv::Point2f> const& tagCorners) {
        // hint: think about how you can use the "image" parameter
        // hint: this will be used later by navigation to stop "close enough" to a tag. units are your choice!
        // hint: do not overcomplicate, this metric does not have to be perfectly accurate, it just has to be correlated with distance away

        // TODO: remove below & implement me!
        //stop when the size of the area of the tag gets to a certain size because that means it will be bigger when you get closer
        (void) image;
        (void) tagCorners;
        float x_dist = abs(tagCorners[0].x - tagCorners[1].x);
        float y_dist = abs(tagCorners[1].y - tagCorners[2].y);
        float area = x_dist * y_dist;

        float imageArea = image.rows * image.cols;
        area/=imageArea;
        //if return is 0 that means ar tag is right next ot camera
        return 1-area;
    }

    std::pair<float, float> Perception::getCenterFromTagCorners(std::vector<cv::Point2f> const& tagCorners) {
        // TODO: remove below & implement me!
        //(void) tagCorners;
        std::pair<float, float> center = {0,0};
        for(cv::Point2f pt : tagCorners){
            center.first += pt.x;
            center.second += pt.y;
        }
        center.first /= 4;
        center.second /= 4;

        return center;
    }

} // namespace mrover