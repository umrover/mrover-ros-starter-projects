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
        findTagsInImage(cvImage->image, mTags);
        // Select the tag that is closest to the middle of the screen
        StarterProjectTag tag = selectTag(cvImage->image, mTags);
        // Publish the message to our topic so navigation or others can receive it
        publishTag(tag);
    }

    void Perception::findTagsInImage(cv::Mat const& image, std::vector<StarterProjectTag>& tags) {
        // hint: you have mTagDictionary, mTagCorners, mTagIds, and mTagDetectorParams member variables already defined!
        // hint: write and use the "getCenterFromTagCorners" and "getClosenessMetricFromTagCorners" functions

        tags.clear(); // Clear old tags in output vector

        // TODO: remove below & implement me!
        // extract aruco tags from image and place them into the tags vector

        // detect markers and save corners and ids to mTagCorners and mTagIds
        cv::aruco::detectMarkers(image, mTagDictionary, mTagCorners, mTagIds, mTagDetectorParams);
        // for debugging purposes, remove these when fixed
        int sz = mTagCorners.size();
        int tsz = mTagIds.size();

        for (size_t i = 0; i < mTagCorners.size(); ++i) {
            std::pair<float, float> center = Perception::getCenterFromTagCorners(mTagCorners[i]);
            float closeness = Perception::getClosenessMetricFromTagCorners(image, mTagCorners[i]);
            StarterProjectTag tag{};
            tag.tagId = mTagIds.at(i);
            tag.closenessMetric = closeness;
            tag.xTagCenterPixel = center.first;
            tag.yTagCenterPixel = center.second;
            
            tags.push_back(tag);
        }
    }

    StarterProjectTag Perception::selectTag(cv::Mat const& image, std::vector<StarterProjectTag> const& tags) {
        int w = image.size().width;
        int h = image.size().height;

        float wDeltaMin = MAXFLOAT;
        float hDeltaMin = MAXFLOAT;
        StarterProjectTag closestTagToCenter{};
        

        for (auto tag : tags) {
            float wDelta = abs(w/2 - tag.xTagCenterPixel);
            float hDelta = abs(h/2 - tag.yTagCenterPixel);

            if (wDelta < wDeltaMin || hDelta < hDeltaMin) {
                wDeltaMin = wDelta;
                hDeltaMin = hDelta;
                closestTagToCenter = tag;
            }
        }

        return closestTagToCenter;
    }

    void Perception::publishTag(StarterProjectTag const& tag) {
        mTagPublisher.publish(tag);
    }

    float Perception::getClosenessMetricFromTagCorners(cv::Mat const& image, std::vector<cv::Point2f> const& tagCorners) {
        // hint: think about how you can use the "image" parameter
        // hint: this will be used later by navigation to stop "close enough" to a tag. units are your choice!
        // hint: do not overcomplicate, this metric does not have to be perfectly accurate, it just has to be correlated with distance away

        // return the distance between image and rover
        // use the formula f = pd/w, where f=focal length, p=apparent width in pixels,
        // d=distance from image, and w=known width
        // so d = fw/p
        // ZED camera has a focal length of 12cm
        // aruco tags are 20x20cm, per URC guidelines

        float distance = 12 * 20 / (tagCorners.at(1).x - tagCorners.at(0).x);

        return distance;
    }

    std::pair<float, float> Perception::getCenterFromTagCorners(std::vector<cv::Point2f> const& tagCorners) {
        std::pair<float, float> center;

        // given that the corners are in default order (start from top left, go counterclockwise)
        center.first = (tagCorners.at(0).x + tagCorners.at(1).x) / 2.0;
        center.second = (tagCorners.at(1).y + tagCorners.at(2).y) / 2.0;

        return center;
    }


} // namespace mrover