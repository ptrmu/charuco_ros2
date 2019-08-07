
#include <memory>

#include "charuco_math.hpp"
#include "charuco_ros2_context.hpp"

#include "cv_bridge/cv_bridge.h"
#include "opencv2/aruco.hpp"
#include "opencv2/aruco/charuco.hpp"
#include "opencv2/calib3d.hpp"
#include "rclcpp/logging.hpp"

namespace charuco_ros2
{

// ==============================================================================
// CharucoBoardModel class
// ==============================================================================

  class CharucoBoardModel
  {
    // Hold the board so the text reads properly and is at the lower left. The
    // origin of the board's coordinate system is the black corner at the top left.
    // When looking at the board with the origin at the upper left, the x axis is
    // to the right and the y axis is down. The checker board square intersections
    // are addressed by their ix and iy indices starting with zero at the origin.
    // The physical location of each intersection is at (ix*square_width,
    // iy*square_height). The address of a square is the same as its upper left
    // corner. Every other square is filled with an aruco marker. Squares
    // where (ix + iy) is odd contain an aruco marker ((0,1), (1,0), (0,3) ...).
    //
    // Every intersection where two black squares intersect has a label.
    // Given ix, iy
    //  no label if ix < 1 or ix > 11
    //  no label if iy < 1 or iy > 8
    //    label = (iy - 1) * 11 + (ix - 1)
    // Given a label
    //  ix = label % 11 + 1
    //  iy = label / 11 + 1
    //
    // Each aruco marker has a tag that is the same as its id
    // Given ix, iy
    //  no tag if ix < 0 or ix > 11
    //  no tag if iy < 0 or iy > 8
    //  no tag if (ix + iy) is even
    // Given tag
    //  iy = tag / 6
    //  ix = (tag % 6) * 2 + (iy is even ? 0 : 1)
    //
    // The corners of an aruco marker are always stored moving clockwise (looking at
    // the marker) around the marker. If looking at the board with the origin at the
    // upper-left, the marker coordinates are stored upper-left, upper-right, lower-right
    // and lower-left.
    // Given the (ix, iy) of a square that contains a marker, the location of the marker
    // corners is:
    //  0: (ix*square_width + (square_width - marker_width) / 2, iy*square_height + (square_height - marker_height) / 2
    //  1: (ix*square_width + (square_width + marker_width) / 2, iy*square_height + (square_height - marker_height) / 2
    //  2: (ix*square_width + (square_width + marker_width) / 2, iy*square_height + (square_height + marker_height) / 2
    //  3: (ix*square_width + (square_width - marker_width) / 2, iy*square_height + (square_height + marker_height) / 2

    int squares_x_;
    int squares_y_;
    float square_length_;
    float marker_length_;
    float square_add_marker_length_;
    float square_sub_marker_length_;

    cv::Point2i tag_to_indices(int tag)
    {
      int iy = tag / 6;
      int ix = (tag % 6) * 2;
      ix += (iy & 1) ? 0 : 1;

      return cv::Point2i{ix, iy};
    }

  public:
    CharucoBoardModel(int squares_x, int squares_y, float square_length, float marker_length) :
      squares_x_{squares_x}, squares_y_{squares_y}, square_length_{square_length}, marker_length_{marker_length}
    {
      square_add_marker_length_ = (square_length + marker_length) / 2.0;
      square_sub_marker_length_ = (square_length - marker_length) / 2.0;
    }

    std::vector<cv::Point2f> marker_corners2D_f_board(int tag)
    {
      // convert from tag to square indices
      auto indices = tag_to_indices(tag);
      auto origin_x = indices.x * square_length_;
      auto origin_y = indices.y * square_length_;
      auto square_origin = cv::Point2f{indices.x * square_length_, indices.y * square_length_};
      return std::vector<cv::Point2f>{
        cv::Point2f{origin_x + square_sub_marker_length_, origin_y + square_sub_marker_length_},
        cv::Point2f{origin_x + square_add_marker_length_, origin_y + square_sub_marker_length_},
        cv::Point2f{origin_x + square_add_marker_length_, origin_y + square_add_marker_length_},
        cv::Point2f{origin_x + square_sub_marker_length_, origin_y + square_add_marker_length_},
      };
    }

    std::vector<cv::Point2f> board_corners2D_f_board()
    {
      return std::vector<cv::Point2f>{
        cv::Point2f{0.0, 0.0},
        cv::Point2f{squares_x_ * square_length_, 0.0},
        cv::Point2f{squares_x_ * square_length_, squares_y_ * square_length_},
        cv::Point2f{0.0, squares_y_ * square_length_},
      };
    }

    float width_per_height()
    {
      return static_cast<float>(squares_x_) / squares_y_;
    }
  };

// ==============================================================================
// BoardProjection struct
// ==============================================================================

  struct BoardProjection
  {
    // Hold the image coordinates of the four corners of the board location.
    std::vector<cv::Point2f> ordered_board_corners_;

    BoardProjection(std::vector<cv::Point2f> &&ordered_board_corners) :
      ordered_board_corners_{ordered_board_corners}
    {}

    float difference(BoardProjection &other)
    {
      auto diff = (
                    cv::norm(ordered_board_corners_[0] - other.ordered_board_corners_[0]) +
                    cv::norm(ordered_board_corners_[1] - other.ordered_board_corners_[1]) +
                    cv::norm(ordered_board_corners_[2] - other.ordered_board_corners_[2]) +
                    cv::norm(ordered_board_corners_[3] - other.ordered_board_corners_[3])
                  ) / 4;
      return diff;
    }
  };

  struct ImageHolder
  {
    std::shared_ptr<cv_bridge::CvImage> gray_;

    std::vector<int> aruco_ids_;
    std::vector<std::vector<cv::Point2f> > aruco_corners_;

    cv::Mat charuco_ids_;
    cv::Mat charuco_corners_;

    cv::Mat homo_;
    BoardProjection board_projection_;

    ImageHolder(std::shared_ptr<cv_bridge::CvImage> &gray,
                std::vector<int> &&aruco_ids, std::vector<std::vector<cv::Point2f> > &&aruco_corners,
                cv::Mat &&charuco_ids, cv::Mat &&charuco_corners,
                cv::Mat &&homo, BoardProjection &&board_projection) :
      gray_{gray},
      aruco_ids_{aruco_ids}, aruco_corners_{aruco_corners},
      charuco_ids_{charuco_ids}, charuco_corners_{charuco_corners},
      homo_{homo}, board_projection_{board_projection}
    {}
  };

  struct CalibrationImage
  {
    BoardProjection target_;
    float difference_{};
    std::shared_ptr<ImageHolder> image_{};

    CalibrationImage(BoardProjection &&target) :
      target_{target}
    {}
  };

  class BoardTargets
  {
    rclcpp::Logger &logger_;
    const cv::Size image_size;
    std::vector<CalibrationImage> best_images_;

    static BoardProjection new_target(float width_per_height, cv::Size &image_size,
                                      int x_alignment, float x_normalized, float width_normalized,
                                      int y_alignment, float y_normalized)
    {
      float x_max = image_size.width - 1;
      float y_max = image_size.height - 1;

      float width = width_normalized * x_max;
      float height = width / width_per_height;

      float left = x_normalized * x_max;
      switch (x_alignment) {
        case 0:
          left -= width / 2;
          break;
        case 1:
          left -= width;
          break;
      }

      float top = y_normalized * y_max;
      switch (y_alignment) {
        case 0:
          top -= height / 2;
          break;
        case 1:
          top -= height;
          break;
      }

      return BoardProjection(std::vector<cv::Point2f>{
        cv::Point2f{left, top},
        cv::Point2f(left + width, top),
        cv::Point2f(left + width, top + height),
        cv::Point2f(left, top + height),
      });
    }

    static std::vector<CalibrationImage> new_best_images(float width_per_height, cv::Size &image_size)
    {
      return std::vector<CalibrationImage>{
        CalibrationImage(new_target(width_per_height, image_size, -1, 0., 0.25, -1, 0.)),
        CalibrationImage(new_target(width_per_height, image_size, +0, .5, 0.25, -1, 0.)),
        CalibrationImage(new_target(width_per_height, image_size, +1, 1., 0.25, -1, 0.)),
        CalibrationImage(new_target(width_per_height, image_size, -1, 0., 0.25, +1, 1.)),
        CalibrationImage(new_target(width_per_height, image_size, +0, .5, 0.25, +1, 1.)),
        CalibrationImage(new_target(width_per_height, image_size, +1, 1., 0.25, +1, 1.)),
      };
    }

  public:
    explicit BoardTargets(rclcpp::Logger &logger, float width_per_height, cv::Size &image_size) :
      logger_{logger}, image_size{image_size}, best_images_{new_best_images(width_per_height, image_size)}
    {
    }

    std::vector<CalibrationImage> &get_best_images()
    {
      return best_images_;
    }

    void compare_to_targets(std::shared_ptr<ImageHolder> &image_holder)
    {
      for (auto &best_image : best_images_) {
        auto difference = best_image.target_.difference(image_holder->board_projection_);
        if (difference < 300.) {
          if (!best_image.image_ ||
              difference < best_image.difference_) {
            best_image.image_ = image_holder;
            best_image.difference_ = difference;
          }
        }
      }
    }
  };


// ==============================================================================
// Drawing functions copied from opencv
// ==============================================================================

  static void drawDetectedMarkers(cv::InputOutputArray _image, cv::InputArrayOfArrays _corners,
                                  cv::InputArray _ids = cv::noArray(),
                                  cv::Scalar borderColor = cv::Scalar(0, 255, 0))
  {
    CV_Assert(_image.getMat().total() != 0 &&
              (_image.getMat().channels() == 1 || _image.getMat().channels() == 3));
    CV_Assert((_corners.total() == _ids.total()) || _ids.total() == 0);

    // calculate colors
    cv::Scalar textColor, cornerColor;
    textColor = cornerColor = borderColor;
    cv::swap(textColor.val[0], textColor.val[1]);     // text color just sawp G and R
    cv::swap(cornerColor.val[1], cornerColor.val[2]); // corner color just sawp G and B

    int nMarkers = (int) _corners.total();
    for (int i = 0; i < nMarkers; i++) {
      cv::Mat currentMarker = _corners.getMat(i);
      CV_Assert(currentMarker.total() == 4 && currentMarker.type() == CV_32FC2);

      // draw marker sides
      for (int j = 0; j < 4; j++) {
        cv::Point2f p0, p1;
        p0 = currentMarker.ptr<cv::Point2f>(0)[j];
        p1 = currentMarker.ptr<cv::Point2f>(0)[(j + 1) % 4];
        line(_image, p0, p1, borderColor, 1);
      }
      // draw first corner mark
      rectangle(_image, currentMarker.ptr<cv::Point2f>(0)[0] - cv::Point2f(3, 3),
                currentMarker.ptr<cv::Point2f>(0)[0] + cv::Point2f(3, 3), cornerColor, 1, cv::LINE_AA);

      // draw ID
//      if (_ids.total() != 0) {
//        cv::Point2f cent(0, 0);
//        for (int p = 0; p < 4; p++)
//          cent += currentMarker.ptr<cv::Point2f>(0)[p];
//        cent = cent / 4.;
//        std::stringstream s;
//        s << "id=" << _ids.getMat().ptr<int>(0)[i];
//        putText(_image, s.str(), cent, cv::FONT_HERSHEY_SIMPLEX, 0.5, textColor, 2);
//      }
    }
  }

  static void drawDetectedCornersCharuco(cv::InputOutputArray _image, cv::InputArray _charucoCorners,
                                         cv::InputArray _charucoIds = cv::noArray(),
                                         cv::Scalar cornerColor = cv::Scalar(255, 0, 0))
  {
    CV_Assert(_image.getMat().total() != 0 &&
              (_image.getMat().channels() == 1 || _image.getMat().channels() == 3));
    CV_Assert((_charucoCorners.getMat().total() == _charucoIds.getMat().total()) ||
              _charucoIds.getMat().total() == 0);

    unsigned int nCorners = (unsigned int) _charucoCorners.getMat().total();
    for (unsigned int i = 0; i < nCorners; i++) {
      cv::Point2f corner = _charucoCorners.getMat().at<cv::Point2f>(i);

      // draw first corner mark
      rectangle(_image, corner - cv::Point2f(3, 3), corner + cv::Point2f(3, 3), cornerColor, 1, cv::LINE_AA);

      // draw ID
//      if (_charucoIds.total() != 0) {
//        int id = _charucoIds.getMat().at<int>(i);
//        std::stringstream s;
//        s << "id=" << id;
//        putText(_image, s.str(), corner + cv::Point2f(5, -5), cv::FONT_HERSHEY_SIMPLEX, 0.5,
//                cornerColor, 2);
//      }
    }
  }

  static void drawBoardCorners(cv::InputOutputArray image, std::vector<cv::Point2f> &board_corners,
                               cv::Scalar borderColor = cv::Scalar(0, 0, 255))
  {
    for (int j = 0; j < 4; j++) {
      cv::Point2f p0, p1;
      p0 = board_corners[j];
      p1 = board_corners[(j + 1) % 4];
      line(image, p0, p1, borderColor, 1);
    }
  }

  static void drawPolygonAtCenter(std::shared_ptr<cv_bridge::CvImage> &color, std::vector<cv::Point2f> &board_corners,
                                  cv::Scalar borderColor = cv::Scalar(0, 0, 255))
  {
    cv::Point2f avg;
    for (int i = 0; i < board_corners.size(); i += 1) {
      avg = avg + board_corners[i];
    }
    avg = avg / double(board_corners.size());
    avg.x -= color->image.cols / 2;
    avg.y -= color->image.rows / 2;
    std::vector<cv::Point> bc = {
      cv::Point{int(round(board_corners[0].x - avg.x)), int(round(board_corners[0].y - avg.y))},
      cv::Point{int(round(board_corners[1].x - avg.x)), int(round(board_corners[1].y - avg.y))},
      cv::Point{int(round(board_corners[2].x - avg.x)), int(round(board_corners[2].y - avg.y))},
      cv::Point{int(round(board_corners[3].x - avg.x)), int(round(board_corners[3].y - avg.y))},
    };
    cv::fillConvexPoly(color->image, bc, borderColor);
//    for (int j = 0; j < 4; j++) {
//      cv::Point2f p0, p1;
//      p0 = board_corners[j];
//      p1 = board_corners[(j + 1) % 4];
//      line(image, p0, p1, borderColor, 1);
//    }
  }

  /*int main()
{
    // Read image
    Mat3b img = imread("path_to_image");

    // Convert to hsv
    Mat3b hsv;
    cvtColor(img, hsv, COLOR_BGR2HSV);

    // Threshold on yellow color (in hsv space)
    Mat1b maskOnYellow;
    inRange(hsv, Scalar(20, 100, 100), Scalar(40, 255, 255), maskOnYellow);

    // Find contours of yellow item
    vector<vector<Point>> contours;
    findContours(maskOnYellow.clone(), contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    // Create a mask as a filled contour
    Mat1b mask(img.rows, img.cols, uchar(0));
    drawContours(mask, contours, 0, Scalar(255), CV_FILLED);

    // Get the bounding box of the item
    Rect box = boundingRect(contours[0]);

    // Get the roi in the input image according to the mask
    Mat3b item(img(box));

    // Create a black image (same size as the yellow item and same background bolor as result image)
    // to copy the result of the segmentation
    Vec3b backgroundColor(0,0,0); // black
    Mat3b segmentedItem(item.rows, item.cols, backgroundColor);

    // Copy only the masked part
    item.copyTo(segmentedItem, mask(box));

    // Compute the center of the image
    Point center(img.cols / 2, img.rows / 2);

    // Create a result image
    Mat3b res(img.rows, img.cols, backgroundColor);

    // Compute the rectangle centered in the image, same size as box
    Rect centerBox(center.x - box.width/2, center.y - box.height/2, box.width, box.height);

    // Put the segmented item in the center of the result image
    segmentedItem.copyTo(res(centerBox));

    imshow("Result", res);
    waitKey();

    return 0;
}

   #include opencv2/opencv.hpp

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{

    // Read the images
    Mat foreground = imread("puppets.png");
    Mat background = imread("ocean.png");
    Mat alpha = imread("puppets_alpha.png");

    // Convert Mat to float data type
    foreground.convertTo(foreground, CV_32FC3);
    background.convertTo(background, CV_32FC3);

    // Normalize the alpha mask to keep intensity between 0 and 1
    alpha.convertTo(alpha, CV_32FC3, 1.0/255); //

    // Storage for output image
    Mat ouImage = Mat::zeros(foreground.size(), foreground.type());

    // Multiply the foreground with the alpha matte
    multiply(alpha, foreground, foreground);

    // Multiply the background with ( 1 - alpha )
    multiply(Scalar::all(1.0)-alpha, background, background);

    // Add the masked foreground and background.
    add(foreground, background, ouImage);

    // Display image
    imshow("alpha blended image", ouImage/255);
    waitKey(0);

    return 0;
}
   */


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

    cv::Size image_size_{};
    std::unique_ptr<BoardTargets> board_targets_{};

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

    std::shared_ptr<ImageHolder> new_image_holder(std::shared_ptr<cv_bridge::CvImage> &gray)
    {
      std::vector<std::vector<cv::Point2f> > rejected;

      // detect markers
      std::vector<int> aruco_ids;
      std::vector<std::vector<cv::Point2f> > aruco_corners;
      cv::aruco::detectMarkers(gray->image, dictionary_, aruco_corners, aruco_ids, detectorParams_, rejected);

      // refind strategy to detect more markers
      if (cxt_.refind_strategy_) {
        cv::aruco::refineDetectedMarkers(gray->image, board_, aruco_corners, aruco_ids, rejected);
      }

      // interpolate charuco corners
      cv::Mat charuco_ids;
      cv::Mat charuco_corners;
      if (!aruco_ids.empty()) {
        cv::aruco::interpolateCornersCharuco(aruco_corners, aruco_ids,
                                             gray->image, charucoboard_,
                                             charuco_corners, charuco_ids);
      }

      // Calculate Homography
      cv::Mat homo;
      std::vector<cv::Point2f> board_corners;
      if (!aruco_ids.empty()) {
        CharucoBoardModel cbm{cxt_.squares_x_, cxt_.squares_y_,
                              cxt_.square_length_, cxt_.marker_length_};

        std::vector<cv::Vec2f> op{};
        std::vector<cv::Vec2f> ip{};

        for (int i = 0; i < aruco_ids.size(); i += 1) {
          auto id = aruco_ids[i];
          auto object_points = cbm.marker_corners2D_f_board(id);
          auto image_points = aruco_corners[i];
          for (int j = 0; j < 4; j += 1) {
            op.emplace_back(cv::Vec2f{float(object_points[j].x), float(object_points[j].y)});
            ip.emplace_back(cv::Vec2f{float(image_points[j].x), float(image_points[j].y)});
          }
        }

        homo = cv::findHomography(op, ip);

        // Figure out the projection of the board corners in the image
        auto board_corners_f_board = cbm.board_corners2D_f_board();
        cv::perspectiveTransform(board_corners_f_board, board_corners, homo);
      }

      return std::make_shared<ImageHolder>(
        gray,
        std::move(aruco_ids), std::move(aruco_corners),
        std::move(charuco_ids), std::move(charuco_corners),
        std::move(homo), BoardProjection{std::move(board_corners)});
    }

  public:
    explicit CvCharucoMath(rclcpp::Logger &logger, const CharucoRos2Context &cxt)
      : logger_{logger}, cxt_(cxt)
    {
//      detectorParams_->doCornerRefinement = true;
//      detectorParams_->cornerRefinementWinSize = 3;
    }

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

    void DumpVec2f(std::string s, cv::Vec2f v)
    {
      RCLCPP_INFO(logger_, "V2 %s - %9.4f, %9.4f", s.c_str(), v(0), v(1));
    }

    void DumpVec3f(std::string s, cv::Vec3f v)
    {
      RCLCPP_INFO(logger_, "V3 %s - %9.4f, %9.4f, %9.4f", s.c_str(), v(0), v(1), v(2));
    }

    void evaluate_image(std::shared_ptr<cv_bridge::CvImage> &marked, std::shared_ptr<cv_bridge::CvImage> &gray)
    {
      // The first time this is called, we have to initialize the targets with the size
      // of the image passed in.
      if (board_targets_ == nullptr) {
        image_size_ = cv::Size{gray->image.cols, gray->image.rows};

        CharucoBoardModel cbm{cxt_.squares_x_, cxt_.squares_y_,
                              cxt_.square_length_, cxt_.marker_length_};

        board_targets_ = std::make_unique<BoardTargets>(logger_, cbm.width_per_height(), image_size_);
      }

      // Don't process images that happen to be a different size.
      if (image_size_.width != gray->image.cols || image_size_.height != gray->image.rows) {
        return;
      }

      auto image_holder = new_image_holder(gray);

      if (!image_holder->aruco_ids_.empty()) {
        board_targets_->compare_to_targets(image_holder);
      }

      // Annotate the image with info we have collected so far.
      if (!image_holder->aruco_ids_.empty()) {
        drawDetectedMarkers(marked->image, image_holder->aruco_corners_);
      }

      if (!image_holder->charuco_ids_.empty()) {
        drawDetectedCornersCharuco(marked->image,
                                   image_holder->charuco_corners_,
                                   image_holder->charuco_ids_);
      }

      if (!image_holder->board_projection_.ordered_board_corners_.empty()) {
        drawBoardCorners(marked->image, image_holder->board_projection_.ordered_board_corners_);
      }

      for (auto &best_image : board_targets_->get_best_images()) {
        drawBoardCorners(marked->image, best_image.target_.ordered_board_corners_);
        if (best_image.image_) {
          drawBoardCorners(marked->image, best_image.image_->board_projection_.ordered_board_corners_,
                           cv::Scalar(255, 0, 0));
        }
      }
    }

    void annotate_image_debug(std::shared_ptr<cv_bridge::CvImage> &color)
    {
      FrameData fd{*this, color->image};

      CharucoBoardModel cbm{cxt_.squares_x_, cxt_.squares_y_,
                            cxt_.square_length_, cxt_.marker_length_};

      /*
      for (int i = 0; i < fd.ids_.size(); i += 1) {
        auto id = fd.ids_[i];
        auto object_points = cbm.marker_corners2D_f_board(id);

        std::vector<cv::Vec2f> op{};
        for (int j = 0; j < 4; j += 1) {
          op.emplace_back(cv::Vec2f{float(object_points[j].x), float(object_points[j].y)});
//          DumpVec2f("op", op[j]);
        }
        auto image_points = fd.corners_[i];
        std::vector<cv::Vec2f> ip{};
        for (int j = 0; j < 4; j += 1) {
          ip.emplace_back(cv::Vec2f{float(image_points[j].x), float(image_points[j].y)});
//          DumpVec2f("ip", ip[j]);
        }

        auto homo = cv::findHomography(op, ip);

//        for (int j = 0; j < 3; j += 1) {
//          cv::Vec3f hr;
//          auto homo_row = cv::Mat{homo.row(j).t()};
//          homo_row.copyTo(hr);
//          DumpVec3f("homo", hr);
//        }

        std::vector<cv::Point2f> obj_p{cv::Point2f{0.00, 0.0}};
        std::vector<cv::Point2f> img_p{};
        cv::perspectiveTransform(obj_p, img_p, homo);
//        cv::Vec2d origin{0., 0.};
//        //auto product = homo.t()*cv::Mat{origin};
//        cv::Vec2d product{};
//        cv::perspectiveTransform(origin, product, homo);
//        cv::Vec3d mapping{cv::Mat{product}};
        RCLCPP_INFO(logger_, "%d, xform x:%9.4f. y:%9.4f", i, img_p[0].x, img_p[0].y);
      }
*/

      // Use all the matches to find the homography
      if (!fd.ids_.empty()) {
        std::vector<cv::Vec2f> op{};
        std::vector<cv::Vec2f> ip{};

        for (int i = 0; i < fd.ids_.size(); i += 1) {
          auto id = fd.ids_[i];
          auto object_points = cbm.marker_corners2D_f_board(id);
          auto image_points = fd.corners_[i];
          for (int j = 0; j < 4; j += 1) {
            op.emplace_back(cv::Vec2f{float(object_points[j].x), float(object_points[j].y)});
            ip.emplace_back(cv::Vec2f{float(image_points[j].x), float(image_points[j].y)});
          }
        }

        auto homo = cv::findHomography(op, ip);

        std::vector<cv::Point2f> obj_p{cv::Point2f{0.0, 0.0}};
        std::vector<cv::Point2f> img_p{};
        cv::perspectiveTransform(obj_p, img_p, homo);

        RCLCPP_INFO(logger_, "Origin using all correspondences, xform x:%9.4f. y:%9.4f", img_p[0].x, img_p[0].y);

        auto board_corners_f_board = cbm.board_corners2D_f_board();
        std::vector<cv::Point2f> board_corners2D_f_image{};
        cv::perspectiveTransform(board_corners_f_board, board_corners2D_f_image, homo);

        drawBoardCorners(color->image, board_corners2D_f_image);

        drawPolygonAtCenter(color, board_corners2D_f_image);
      }

      // draw results
      if (!fd.ids_.empty()) {
        drawDetectedMarkers(color->image, fd.corners_);
      }

      if (fd.currentCharucoCorners_.total() > 0) {
        drawDetectedCornersCharuco(color->image, fd.currentCharucoCorners_, fd.currentCharucoIds_);
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

  void
  CharucoMath::evaluate_image(std::shared_ptr<cv_bridge::CvImage> &marked, std::shared_ptr<cv_bridge::CvImage> &gray)
  {
    cv_->evaluate_image(marked, gray);
  }

  void CharucoMath::annotate_image_debug(std::shared_ptr<cv_bridge::CvImage> &color)
  {
    cv_->annotate_image_debug(color);
  }

  charuco_ros2_msgs::srv::Calibrate::Response::_rc_type CharucoMath::calculate_calibration(
    const std::vector<std::shared_ptr<cv_bridge::CvImage>> &captured_images)
  {
    return cv_->calculate_calibration(captured_images);
  }

}
