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

        // Image is the input and mTagCorners and mTagIds are the output
        tags.clear(); // Clear old tags in output vector

        cv::aruco::detectMarkers(image->image, mTagDictionary, mTagCorners, mTagIds, mTagDetectorParams);
        for(size_t i = 0; i < mTagCorners.size(); ++i){
            float metric = getClosenessMetricFromTagCorners(image->image, mTagCorners[i]);
            std::pair<float, float> center = getCenterFromTagCorners(mTagCorners[i]);
            StarterProjectTag tag;
            tag.tagId = mTagIds[i];
            tag.closenessMetric = metric;
            // Normalize the centers to that (0,0) is in the center of the image, rather than the top left corner
            // The values will be between -1 and 1
            tag.xTagCenterPixel =(center.first - image->image.cols/2)/image->image.cols;
            tag.yTagCenterPixel = (center.second - image->image.rows/2)/image->image.rows;
            tags.push_back(tag);
        }
    }


    StarterProjectTag Perception::selectTag(std::vector<StarterProjectTag> const& tags) {
        if (tags.empty()) {
            StarterProjectTag tag{};
            tag.tagId = -1;
            return tag;
        }
        float current_min = (tags[0].xTagCenterPixel * tags[0].xTagCenterPixel) + (tags[0].yTagCenterPixel * tags[0].yTagCenterPixel);
        StarterProjectTag current_min_tag = tags[0];
        for(size_t i = 1; i < tags.size(); ++i){
            float distance = (tags[i].xTagCenterPixel * tags[i].xTagCenterPixel) + (tags[i].yTagCenterPixel * tags[i].yTagCenterPixel);
            if(distance < current_min){
                current_min = distance;
                current_min_tag = tags[i];
            }
        }
        return current_min_tag;
    }

    void Perception::publishTag(StarterProjectTag const& tag) {
        mTagPublisher.publish(tag);
    }

    float Perception::getClosenessMetricFromTagCorners(cv::Mat const& image, std::vector<cv::Point2f> const& tagCorners) {
        // hint: think about how you can use the "image" parameter
        // hint: this will be used later by navigation to stop "close enough" to a tag. units are your choice!
        // hint: do not overcomplicate, this metric does not have to be perfectly accurate, it just has to be correlated with distance away

        // Find the area of the tag
        // tagCorners goes clockwise starting from top left
        // 0 is right in front of tag and 1 is farthest away
        float top_left_x = tagCorners[0].x;
        float top_right_x = tagCorners[1].x;
        float top_right_y = tagCorners[1].y;

        float width_tag = abs(top_right_x - top_left_x);
        float height_tag = abs(top_right_y - top_left_x); 
        float area_tag = width_tag*height_tag;

        float area_img = image.rows*image.cols;
        // return ratio of how much of the image the tag takes up 
        return 1-(area_tag/area_img); 
    }

    std::pair<float, float> Perception::getCenterFromTagCorners(std::vector<cv::Point2f> const& tagCorners) {
        // Add up all the x and y coordinates and find their average (the middle)
        int x_sum = 0;
        int y_sum = 0;
        for(auto c : tagCorners){
            x_sum += c.x;
            y_sum += c.y;
        }
        x_sum /= 4;
        y_sum /= 4;

        std::pair <float, float> result (x_sum, y_sum);

        return result;
    }

} // namespace mrover