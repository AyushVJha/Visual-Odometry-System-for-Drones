#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>
#include <ceres/ceres.h>
#include <Eigen/Core>

// Visual Odometry class definition and implementation
class VisualOdometry {
public:
    VisualOdometry() {
        orb = cv::ORB::create();
        matcher = cv::BFMatcher::create(cv::NORM_HAMMING);
    }

    void processFrame(const cv::Mat& img) {
        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;
        orb->detectAndCompute(img, cv::noArray(), keypoints, descriptors);

        if (!prevDescriptors.empty()) {
            std::vector<std::vector<cv::DMatch>> knnMatches;
            matcher->knnMatch(prevDescriptors, descriptors, knnMatches, 2);

            // Filter matches using Lowe's ratio test
            std::vector<cv::DMatch> goodMatches;
            for (auto& m : knnMatches) {
                if (m[0].distance < 0.75 * m[1].distance) {
                    goodMatches.push_back(m[0]);
                }
            }

            // Extract matched points
            std::vector<cv::Point2f> pts1, pts2;
            for (auto& m : goodMatches) {
                pts1.push_back(prevKeypoints[m.queryIdx].pt);
                pts2.push_back(keypoints[m.trainIdx].pt);
            }

            if (pts1.size() >= 8) {
                // Estimate essential matrix using RANSAC
                cv::Mat mask;
                cv::Mat E = cv::findEssentialMat(pts1, pts2, focalLength, principalPoint, cv::RANSAC, 0.999, 1.0, mask);

                // Recover pose
                cv::Mat R, t;
                int inliers = cv::recoverPose(E, pts1, pts2, R, t, focalLength, principalPoint, mask);

                std::cout << "Inliers: " << inliers << std::endl;
                std::cout << "Rotation:\n" << R << std::endl;
                std::cout << "Translation:\n" << t << std::endl;

                // TODO: Bundle adjustment with Ceres Solver
            }
        }

        prevKeypoints = keypoints;
        prevDescriptors = descriptors.clone();
    }

    void setCameraParameters(double focal, cv::Point2d pp) {
        focalLength = focal;
        principalPoint = pp;
    }

private:
    cv::Ptr<cv::ORB> orb;
    cv::Ptr<cv::BFMatcher> matcher;
    std::vector<cv::KeyPoint> prevKeypoints;
    cv::Mat prevDescriptors;
    double focalLength = 718.8560; // example value from KITTI dataset
    cv::Point2d principalPoint = cv::Point2d(607.1928, 185.2157); // example value from KITTI dataset
};

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: ./visual_odometry <video_file>" << std::endl;
        return -1;
    }

    cv::VideoCapture cap(argv[1]);
    if (!cap.isOpened()) {
        std::cerr << "Error opening video file." << std::endl;
        return -1;
    }

    VisualOdometry vo;
    vo.setCameraParameters(718.8560, cv::Point2d(607.1928, 185.2157));

    cv::Mat frame;
    while (cap.read(frame)) {
        vo.processFrame(frame);
        cv::imshow("Frame", frame);
        if (cv::waitKey(30) == 27) break; // ESC to quit
    }

    return 0;
}
