
#include <memory>

#include "charuco_math.hpp"
#include "charuco_ros2_context.hpp"

#include "cv_bridge/cv_bridge.h"
#include "opencv2/aruco.hpp"
#include "opencv2/aruco/charuco.hpp"
#include "rclcpp/logging.hpp"

namespace charuco_ros2
{

// ==============================================================================
// CvCharucoMath class
// ==============================================================================

  class CharucoMath::CvCharucoMath
  {
    rclcpp::Logger &logger_;
    const CharucoRos2Context &cxt_;

    cv::Ptr<cv::aruco::DetectorParameters> detectorParams_ = cv::aruco::DetectorParameters::create();

    cv::Ptr<cv::aruco::Dictionary> dictionary_ =
      cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(cxt_.aruco_dictionary_id_));

    // create charuco board object
    cv::Ptr<cv::aruco::CharucoBoard> charucoboard_ =
      cv::aruco::CharucoBoard::create(
        cxt_.squares_x_, cxt_.squares_y_,
        cxt_.square_length_, cxt_.marker_length_,
        dictionary_);
    cv::Ptr<cv::aruco::Board> board_ = charucoboard_.staticCast<cv::aruco::Board>();


    struct FrameData
    {
      std::vector<int> ids_;
      std::vector<std::vector<cv::Point2f> > corners_;
      cv::Mat currentCharucoCorners_;
      cv::Mat currentCharucoIds_;

      explicit FrameData(CharucoMath::CvCharucoMath &cvcm, cv::Mat &image)
      {
        std::vector<std::vector<cv::Point2f> > rejected;

        // detect markers
        cv::aruco::detectMarkers(image, cvcm.dictionary_, corners_, ids_, cvcm.detectorParams_, rejected);

        // refind strategy to detect more markers
        if (cvcm.cxt_.refind_strategy_) {
          cv::aruco::refineDetectedMarkers(image, cvcm.board_, corners_, ids_, rejected);
        }

        // interpolate charuco corners
        if (!ids_.empty())
          cv::aruco::interpolateCornersCharuco(corners_, ids_,
                                               image, cvcm.charucoboard_,
                                               currentCharucoCorners_, currentCharucoIds_);
      }
    };


    struct AllFramesData
    {
      // collect data from each frame
      std::vector<std::vector<std::vector<cv::Point2f>>> allCorners_;
      std::vector<std::vector<int>> allIds_;
      std::vector<cv::Mat> allImgs_;
      cv::Size imgSize_;

      explicit AllFramesData(CharucoMath::CvCharucoMath &cvcm,
                             const std::vector<std::shared_ptr<cv_bridge::CvImage>> &captured_images)
      {
        for (auto &color : captured_images) {
          FrameData fd{cvcm, color->image};
          allCorners_.emplace_back(fd.corners_);
          allIds_.emplace_back(fd.ids_);
          allImgs_.emplace_back(color->image);
          imgSize_ = color->image.size();
        }
      }
    };


  public:
    explicit CvCharucoMath(rclcpp::Logger &logger, const CharucoRos2Context &cxt)
      : logger_{logger}, cxt_(cxt)
    {}

    void annotate_image(std::shared_ptr<cv_bridge::CvImage> &color)
    {
      FrameData fd{*this, color->image};

      // draw results
      if (!fd.ids_.empty()) {
        cv::aruco::drawDetectedMarkers(color->image, fd.corners_);
      }

      if (fd.currentCharucoCorners_.total() > 0) {
        cv::aruco::drawDetectedCornersCharuco(color->image, fd.currentCharucoCorners_, fd.currentCharucoIds_);
      }
    }

    charuco_ros2_msgs::srv::Calibrate::Response::_rc_type calculate_calibration(
      const std::vector<std::shared_ptr<cv_bridge::CvImage>> &captured_images)
    {
      int calibrationFlags = 0;

      AllFramesData afd{*this, captured_images};

      if (afd.allIds_.empty()) {
        RCLCPP_INFO(logger_, "Not enough captures for calibration");
        return charuco_ros2_msgs::srv::Calibrate_Response::NOT_ENOUGH_IMAGES;
      }

      cv::Mat cameraMatrix, distCoeffs;
      std::vector<cv::Mat> rvecs, tvecs;
      double repError;

      // prepare data for calibration
      std::vector<std::vector<cv::Point2f> > allCornersConcatenated;
      std::vector<int> allIdsConcatenated;
      std::vector<int> markerCounterPerFrame;
      markerCounterPerFrame.reserve(afd.allCorners_.size());
      for (unsigned int i = 0; i < afd.allCorners_.size(); i++) {
        markerCounterPerFrame.push_back((int) afd.allCorners_[i].size());
        for (unsigned int j = 0; j < afd.allCorners_[i].size(); j++) {
          allCornersConcatenated.push_back(afd.allCorners_[i][j]);
          allIdsConcatenated.push_back(afd.allIds_[i][j]);
        }
      }

      // calibrate camera using aruco markers
      double arucoRepErr;
      arucoRepErr = cv::aruco::calibrateCameraAruco(allCornersConcatenated, allIdsConcatenated,
                                                    markerCounterPerFrame, board_, afd.imgSize_, cameraMatrix,
                                                    distCoeffs, cv::noArray(), cv::noArray(), calibrationFlags);

      // prepare data for charuco calibration
      int nFrames = (int) afd.allCorners_.size();
      std::vector<cv::Mat> allCharucoCorners;
      std::vector<cv::Mat> allCharucoIds;
      std::vector<cv::Mat> filteredImages;
      allCharucoCorners.reserve(nFrames);
      allCharucoIds.reserve(nFrames);

      for (int i = 0; i < nFrames; i++) {
        // interpolate using camera parameters
        cv::Mat currentCharucoCorners, currentCharucoIds;
        cv::aruco::interpolateCornersCharuco(afd.allCorners_[i], afd.allIds_[i], afd.allImgs_[i], charucoboard_,
                                             currentCharucoCorners, currentCharucoIds, cameraMatrix,
                                             distCoeffs);

        allCharucoCorners.push_back(currentCharucoCorners);
        allCharucoIds.push_back(currentCharucoIds);
        filteredImages.push_back(afd.allImgs_[i]);
      }

      if (allCharucoCorners.size() < 2) {
        RCLCPP_INFO(logger_, "Not enough corners for calibration");
        return charuco_ros2_msgs::srv::Calibrate_Response::NOT_ENOUGH_CORNERS;
      }

      // calibrate camera using charuco
      repError =
        cv::aruco::calibrateCameraCharuco(allCharucoCorners, allCharucoIds, charucoboard_, afd.imgSize_,
                                          cameraMatrix, distCoeffs, rvecs, tvecs, calibrationFlags);

//      bool saveOk =  saveCameraParams(outputFile, imgSize, aspectRatio, calibrationFlags,
//                                      cameraMatrix, distCoeffs, repError);
//      if(!saveOk) {
//        cerr << "Cannot save output file" << endl;
//        return 0;
//      }

//      cout << "Rep Error: " << repError << endl;
//      cout << "Rep Error Aruco: " << arucoRepErr << endl;
//      cout << "Calibration saved to " << outputFile << endl;


      return charuco_ros2_msgs::srv::Calibrate_Response::OK;
    }
  };

// ==============================================================================
// CharucoMath class
// ==============================================================================

  CharucoMath::CharucoMath(rclcpp::Logger &logger, const CharucoRos2Context &cxt)
    : cv_{std::make_unique<CharucoMath::CvCharucoMath>(logger, cxt)}
  {}

  CharucoMath::~CharucoMath() = default;

  void CharucoMath::annotate_image(std::shared_ptr<cv_bridge::CvImage> &color)
  {
    cv_->annotate_image(color);
  }

  charuco_ros2_msgs::srv::Calibrate::Response::_rc_type CharucoMath::calculate_calibration(
    const std::vector<std::shared_ptr<cv_bridge::CvImage>> &captured_images)
  {
    return cv_->calculate_calibration(captured_images);
  }

}
