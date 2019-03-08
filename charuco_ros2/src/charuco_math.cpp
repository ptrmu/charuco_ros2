
#include <memory>
#include <charuco_ros2_context.hpp>

#include "charuco_math.hpp"

#include "cv_bridge/cv_bridge.h"
#include "opencv2/aruco.hpp"
#include "opencv2/aruco/charuco.hpp"

namespace charuco_ros2
{

// ==============================================================================
// CvCharucoMath class
// ==============================================================================

  class CharucoMath::CvCharucoMath
  {
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


    struct MarkerDetector
    {
      std::vector<int> ids_;
      std::vector<std::vector<cv::Point2f> > corners_;
      cv::Mat currentCharucoCorners_;
      cv::Mat currentCharucoIds_;

      MarkerDetector(CharucoMath::CvCharucoMath &cvcm, cv::InputArray image)
      {
        std::vector<std::vector<cv::Point2f> > rejected;

        // detect markers
        cv::aruco::detectMarkers(image, cvcm.dictionary_, corners_, ids_, cvcm.detectorParams_, rejected);

        // refind strategy to detect more markers
        if (cvcm.cxt_.refind_strategy_) {
          cv::aruco::refineDetectedMarkers(image, cvcm.board_, corners_, ids_, rejected);
        }

        // interpolate charuco corners
        cv::Mat currentCharucoCorners, currentCharucoIds;
        if (ids_.size() > 0)
          cv::aruco::interpolateCornersCharuco(corners_, ids_,
                                               image, cvcm.charucoboard_,
                                               currentCharucoCorners, currentCharucoIds);
      }
    };

  public:
    explicit CvCharucoMath(const CharucoRos2Context &cxt)
      : cxt_(cxt)
    {}

    void annotate_image(std::shared_ptr<cv_bridge::CvImage> color)
    {
      MarkerDetector md{*this, color->image};

      // draw results
      if (md.ids_.size() > 0) {
        cv::aruco::drawDetectedMarkers(color->image, md.corners_);
      }

      if (md.currentCharucoCorners_.total() > 0) {
        cv::aruco::drawDetectedCornersCharuco(color->image, md.currentCharucoCorners_, md.currentCharucoIds_);
      }
    }

    void calculate_calibration(const std::vector<std::shared_ptr<cv_bridge::CvImage>> &captured_images)
    {

    }
  };

// ==============================================================================
// CharucoMath class
// ==============================================================================

  CharucoMath::CharucoMath() = default;

  CharucoMath::~CharucoMath() = default;

  void CharucoMath::init(const CharucoRos2Context &cxt)
  {
    cv_ = std::make_unique<CharucoMath::CvCharucoMath>(cxt);
  }

  void CharucoMath::annotate_image(std::shared_ptr<cv_bridge::CvImage> color)
  {
    cv_->annotate_image(color);
  }

  void CharucoMath::calculate_calibration(const std::vector<std::shared_ptr<cv_bridge::CvImage>> &captured_images)
  {
    cv_->calculate_calibration(captured_images);
  }

}
