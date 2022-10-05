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

        for (size_t i = 0; i < mTagCorners.size(); i++) {
            StarterProjectTag tag;
            tag.tagId = mTagIds[i];
            tag.clossnessMetric = getClosenessMetricFromTagCorners(image->image, mTagCorners[i]);
            std::pair<float, float> center = getCenterFromTagCorners(mTagCorners[i]);

            // Normalize center - (0,0) is top left, we want to make the center (0,0)
            // Also, we want to return xTagCenterPixel & yTagCenterPixel as a proportion from the center of the image
            // Between -1 & 1
            tag.xTagCenterPixel = (center.first - image->image.cols/2) / image->image.cols;
            tag.yTagCenterPixel = (center.second - image->image.rows/2) / image->image.rows;

            tags.push_back(tag);
        }
    }

    float getSquaredDistanceFromZero(const StarterProjectTag& tag) {
        return (tag.xTagCenterPixel * tag.xTagCenterPixel) + (tag.yTagCenterPixel * tag.yTagCenterPixel);
    }

    StarterProjectTag Perception::selectTag(std::vector<StarterProjectTag> const& tags) {
        int indexOfClosestTag = 0;
        float minDistance = getSquaredDistanceFromZero(tags[0]);

        for (size_t i = 1; i < tags.size(); i++) {
            if (getSquaredDistanceFromZero(tags[i]) < minDistance) {
                minDistance = getSquaredDistanceFromZero(tags[i]);
                indexOfClosestTag = i;
            }
        }

        return tags[indexOfClosestTag];
    }

    void Perception::publishTag(StarterProjectTag const& tag) {
        mTagPublisher.publish(tag);
    }

    float Perception::getClosenessMetricFromTagCorners(cv::Mat const& image, std::vector<cv::Point2f> const& tagCorners) {
        // 0 means tag is right there, 1 means tag is far away
        // hint: think about how you can use the "image" parameter
        // hint: this will be used later by navigation to stop "close enough" to a tag. units are your choice!
        // hint: do not overcomplicate, this metric does not have to be perfectly accurate, it just has to be correlated with distance away

        float maxX = tagCorners[0].x;
        float minX = tagCorners[0].x;

        for (size_t i = 1; i < tagCorners.size(); i++) {
            if (tagCorners[i].x > maxX) maxX = tagCorners[i].x;
            else if (tagCorners[i].x < minX) minX = tagCorners[i].x;
        }

        float widthOfTag = maxX - minX;

        return 1 - (widthOfTag / (image.cols * 1.0));
    }

    std::pair<float, float> Perception::getCenterFromTagCorners(std::vector<cv::Point2f> const& tagCorners) {
        float xCenter = 0;
        float yCenter = 0;

        // Accumulate x & y of corners
        for (auto c : tagCorners) {
            xCenter += c.x;
            yCenter += c.y;
        }

        // Get average of x & y to get center coords
        xCenter /= 4;
        yCenter /= 4;

        return std::make_pair(xCenter, yCenter);
    }

} // namespace mrover