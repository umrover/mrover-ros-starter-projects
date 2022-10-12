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

        // TODO: implement me!
        cv::aruco::detectMarkers(image->image, mTagDictionary, mTagCorners, mTagIds, mTagDetectorParams);
        for (int i = 0; i < (int)mTagIds.size(); ++i) {
            StarterProjectTag t;
            for (int j = 0; j < (int)mTagCorners.size(); ++j) {
                std::pair<float, float> center = getCenterFromTagCorners(image->image, mTagCorners.at(i));
                t.x = center.first;
                t.y = center.second;

            }
            t.dist = getClosenessMetricFromTagCorners(image->image, mTagCorners.at(i));
            t.id = mTagIds.at(i);
            mTags.push_back(t);
        }
    }

    StarterProjectTag Perception::selectTag(std::vector<StarterProjectTag> const& tags) {
        // TODO: implement me!
        if (tags.size() > 0) {
            ROS_INFO("Tag found!");
            if (tags.size() == 1) {
                return tags.at(0);
            }
            int minTag = 0;
            double minTagWeight = INT_MAX;
            for (int i = 0; i < (int)tags.size(); ++i) {
                double currentTagWeight = sqrt(pow(tags.at(i).x, 2) + pow(tags.at(i).y, 2));
                if (currentTagWeight < minTagWeight) {
                    minTagWeight = currentTagWeight;
                    minTag = i;
                }
            }
            return tags.at(minTag);

        }
        return {};
    }

    void Perception::publishTag(const StarterProjectTag& tag) {
        // TODO: implement me!
        mTagPublisher.publish(tag);
    }

    float Perception::getClosenessMetricFromTagCorners(cv::Mat const& image, std::vector<cv::Point2f> const& tagCorners) {
        // hint: think about how you can use the "image" parameter
        float imageSize = image.cols * image.rows;
        cv::Point2f topLeft = tagCorners.at(0);
        cv::Point2f topRight = tagCorners.at(1);
        cv::Point2f bottomLeft = tagCorners.at(2);
        float width = topRight.x - topLeft.x;
        float height = bottomLeft.y - topLeft.y;
        float metric = (width * height)/imageSize;
        return metric;
    }

    std::pair<float, float> Perception::getCenterFromTagCorners(cv::Mat const& image, const std::vector<cv::Point2f>& tagCorners) {
        // TODO: implement me!
        if (tagCorners.size() <= 4) {
            cv::Point2f topLeft = tagCorners.at(0);
            cv::Point2f topRight = tagCorners.at(1);
            cv::Point2f bottomLeft = tagCorners.at(2);
            cv::Point2f bottomRight = tagCorners.at(3);
            std::pair<float, float> center; 
            center.first = (topLeft.x + topRight.x + bottomLeft.x + bottomRight.x) / 4.0 - (image.cols / 2);
            center.second = (image.rows / 2) - (topLeft.y + topRight.y + bottomLeft.y + bottomRight.y) / 4.0;
            return center;
        } else {
            exit(1);
        }
        


        return {};
    }

} // namespace mrover