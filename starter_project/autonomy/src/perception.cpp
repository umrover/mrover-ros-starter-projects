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
        cv::aruco::detectMarkers(image,mTagDictionary,mTagCorners,mTagIds,mTagDetectorParams);

        int count = 0;

        for(auto & corners: mTagCorners){
            std::pair<float, float> tagavg = getCenterFromTagCorners(corners);
            float closeness = getClosenessMetricFromTagCorners(image,corners);
            mrover::StarterProjectTag tag;
            tag.tagId = mTagIds[count];
            tag.xTagCenterPixel = tagavg.first;
            tag.yTagCenterPixel = tagavg.second;
            tag.closenessMetric = closeness;
            tags.push_back({tag});
            count++;
        }
    }

    StarterProjectTag Perception::selectTag(cv::Mat const& image, std::vector<StarterProjectTag> const& tags) {
        // TODO: remove below & implement me!

        if(tags.size() == 0){
            mrover::StarterProjectTag tag;
            tag.tagId = 0;
            tag.xTagCenterPixel = 0.0;
            tag.yTagCenterPixel = 0.0;
            tag.closenessMetric = 0.0;

            return tag;
        }



        float widthcenter = image.cols/2;
        float heightcenter = image.rows/2;
        cv::Point2f center(widthcenter,heightcenter);

        int currindex = 0;
        float mintagindex= 0;
        float mintagdist= INT32_MAX;


        for(auto & tag: tags){
            cv::Point2f taglocation(tag.xTagCenterPixel,tag.yTagCenterPixel);
            float dist = cv::norm(taglocation-center);

            if(dist < mintagdist){
                mintagindex = currindex;
                mintagdist = dist;
            }

            currindex++;
        }




        return tags[mintagindex];
    }

    void Perception::publishTag(StarterProjectTag const& tag) {
        // TODO: remove below & implement me!
        mTagPublisher.publish(tag);
        (void) tag;
    }

    float Perception::getClosenessMetricFromTagCorners(cv::Mat const& image, std::vector<cv::Point2f> const& tagCorners) {
        // hint: think about how you can use the "image" parameter
        // hint: this will be used later by navigation to stop "close enough" to a tag. units are your choice!
        // hint: do not overcomplicate, this metric does not have to be perfectly accurate, it just has to be correlated with distance away

        // TODO: remove below & implement me!
        float tagarea = contourArea(tagCorners);
        float imagearea = (image.cols * image.rows);

        return (tagarea/imagearea);
    }

    std::pair<float, float> Perception::getCenterFromTagCorners(std::vector<cv::Point2f> const& tagCorners) {
        // TODO: remove below & implement me!
        float xsum = 0;
        float ysum = 0;
        size_t tags = tagCorners.size();


        for(auto &point :tagCorners){
            xsum+=point.x;
            ysum+=point.y;
        }

        if(tags == 0){
            tags == 1;
        }

        return {xsum/tags,ysum/tags};
    }

} // namespace mrover