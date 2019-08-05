#include <opencv2/core/core.hpp>
#include <opencv/cv.hpp>
#include <opencv2/ximgproc.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <boost/filesystem.hpp>
#include <matcherJrsgm.h>

#include <string>

using namespace cv;

int CV_StereoBM = 0;
int CV_StereoSGBM = 1;
int JR_StereoSGBM = 2;

int _stereo_algorithm, _min_disparity, _disparity_range, _correlation_window_size, _uniqueness_ratio, _texture_threshold;
int _speckle_size, _speckle_range, _disp12MaxDiff;
float _p1, _p2;
bool _interp;
std::string _jr_config_file = "";

typedef pcl::PointCloud<pcl::PointXYZRGBNormal> PointCloudRGBNormal;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

class CameraInfo
{
public:
  Mat K, D, R, P;

public:
  CameraInfo()
  {
  }

  CameraInfo(std::string camera_info_yaml_path)
  {
    FileStorage camera_info_fs;
    camera_info_fs.open(camera_info_yaml_path, FileStorage::READ);
    camera_info_fs["camera_matrix"] >> K;
    camera_info_fs["distortion_coefficients"] >> D;
    camera_info_fs["rectification_matrix"] >> R;
    camera_info_fs["projection_matrix"] >> P;
    camera_info_fs.release();
  }
};

class CameraInfoPair
{
public:
  CameraInfo left, right;

public:
  CameraInfoPair(std::string left_camera_info_yaml_path, std::string right_camera_info_yaml_path)
  {
    left = CameraInfo(left_camera_info_yaml_path);
    right = CameraInfo(right_camera_info_yaml_path);
  }
};

//Calculate disparity using left and right images
Mat stereo_match(Mat left_image, Mat right_image, int algorithm, int min_disparity, int disparity_range, int correlation_window_size, int uniqueness_ratio, int texture_threshold, int speckleSize, int speckelRange, int disp12MaxDiff, float p1, float p2, bool interp)
{
  bool backwardMatch = interp;
  cv::Mat disp, disparity_rl, disparity_filter;
  cv::Size image_size = cv::Size(left_image.size().width, left_image.size().height);
  // Setup for 16-bit disparity
  cv::Mat(image_size, CV_16S).copyTo(disp);
  cv::Mat(image_size, CV_16S).copyTo(disparity_rl);
  cv::Mat(image_size, CV_16S).copyTo(disparity_filter);

  if ((disparity_range < 1 || disparity_range % 16 != 0) && (algorithm == CV_StereoBM || algorithm == CV_StereoSGBM))
  {
    return disp;
  }

  //disparity_range = disparity_range > 0 ? disparity_range : ((left_image.size().width / 8) + 15) & -16;

  if (algorithm == CV_StereoBM)
  {
    cv::Ptr<cv::StereoBM> bm = cv::StereoBM::create(64, 9);

    bm->setPreFilterCap(31);
    bm->setPreFilterSize(15);
    bm->setPreFilterType(1);
    bm->setBlockSize(correlation_window_size > 0 ? correlation_window_size : 9);
    bm->setMinDisparity(min_disparity);
    bm->setNumDisparities(disparity_range);
    bm->setTextureThreshold(texture_threshold);
    bm->setUniquenessRatio(uniqueness_ratio);
    bm->setSpeckleWindowSize(speckleSize);
    bm->setSpeckleRange(speckelRange);
    bm->setDisp12MaxDiff(disp12MaxDiff);

    bm->compute(left_image, right_image, disp);

    if (backwardMatch)
    {
      auto right_matcher = cv::ximgproc::createRightMatcher(bm);
      right_matcher->compute(right_image, left_image, disparity_rl);
      double wls_lambda = 8000;
      double wls_sigma = 1.5;
      cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter = cv::ximgproc::createDisparityWLSFilter(bm);
      wls_filter->setLambda(wls_lambda);
      wls_filter->setSigmaColor(wls_sigma);
      wls_filter->filter(disp, left_image, disparity_filter, disparity_rl);
      disp = disparity_filter;
    }
  }
  else if (algorithm == CV_StereoSGBM)
  {
    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(64, 9);

    sgbm->setPreFilterCap(31);
    sgbm->setBlockSize(correlation_window_size > 0 ? correlation_window_size : 9);
    sgbm->setMinDisparity(min_disparity);
    sgbm->setNumDisparities(disparity_range);
    sgbm->setUniquenessRatio(uniqueness_ratio);
    sgbm->setSpeckleWindowSize(speckleSize);
    sgbm->setSpeckleRange(speckelRange);
    sgbm->setDisp12MaxDiff(disp12MaxDiff);
    sgbm->setP1(p1);
    sgbm->setP2(p2);

    sgbm->compute(left_image, right_image, disp);

    if (backwardMatch)
    {
      auto right_matcher = cv::ximgproc::createRightMatcher(sgbm);
      right_matcher->compute(right_image, left_image, disparity_rl);
      double wls_lambda = 8000;
      double wls_sigma = 1.5;
      cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter = cv::ximgproc::createDisparityWLSFilter(sgbm);
      wls_filter->setLambda(wls_lambda);
      wls_filter->setSigmaColor(wls_sigma);
      wls_filter->filter(disp, left_image, disparity_filter, disparity_rl);
      disp = disparity_filter;
    }
  }
  else if (algorithm == JR_StereoSGBM)
  {
    MatcherJrSGM *matcher = new MatcherJrSGM(_jr_config_file);
    //matcher->setDisparityRange(disparity_range);
    //matcher->setDisparityShift(min_disparity);
    //matcher->setMatchCosts(p1, p2);
    //matcher->setWindowSize(correlation_window_size);
    //matcher->enableInterpolation(interp);

    //Jr matcher works with 32 bit float disparity
    cv::Mat( image_size, CV_32FC1 ).copyTo( disp );
    matcher->compute(left_image, right_image, disp);

    if (backwardMatch)
    {
      matcher->backwardMatch(left_image, right_image, disparity_rl);
      // TODO impliment backward matching filter for JR
    }
  }
  return disp;
}

cv::Mat_<uint8_t> rectify(cv::Mat image, CameraInfo cameraInfo)
{
  cv::Size resol = cv::Size(image.size().width, image.size().height);

  cv::Mat full_map1, full_map2;

  cv::initUndistortRectifyMap(cameraInfo.K, cameraInfo.D, cameraInfo.R, cameraInfo.P, resol,
                              CV_32FC1, full_map1, full_map2);

  cv::Mat_<uint8_t> image_rect;
  cv::remap(image, image_rect, full_map1, full_map2, cv::INTER_CUBIC);

  return (image_rect);
}

cv::Mat processDisparity(const cv::Mat &left_rect, const cv::Mat &right_rect)
{
  cv::Mat disparity16 = stereo_match(left_rect, right_rect, _stereo_algorithm, _min_disparity, _disparity_range, _correlation_window_size, _uniqueness_ratio, _texture_threshold, _speckle_size, _speckle_range, _disp12MaxDiff, _p1, _p2, _interp);
  cv::Mat disparity32f;
  disparity16.convertTo(disparity32f, CV_32F);
  return disparity32f;
}

void processImages(cv::Mat image_left, cv::Mat image_right, CameraInfoPair cameraInfoPair)
{
  cv::Mat frame_joint;
  cv::Mat_<uint8_t> left_rect = rectify(image_left, cameraInfoPair.left);
  cv::Mat_<uint8_t> right_rect = rectify(image_right, cameraInfoPair.right);

  cv::Mat disp = processDisparity(left_rect, right_rect);

  hconcat(left_rect, right_rect, frame_joint);

  Mat raw_disp_vis;
  int vis_mult = 10;
  cv::ximgproc::getDisparityVis(disp, raw_disp_vis, vis_mult);

  imshow("Camera", frame_joint);
  imshow("Disparity", raw_disp_vis);
}

void loadDisparityConfig(std::string disparity_config)
{
  iniReader *settings = new iniReader(disparity_config);

  _stereo_algorithm = settings->value("Disparity", "Stereo Algorithm", 0);
  _min_disparity = settings->value("Disparity", "Min Disparity", -25);
  _disparity_range = settings->value("Disparity", "Disparity Range", 17);
  _correlation_window_size = settings->value("Disparity", "Correlation Window Size", 3);
  _uniqueness_ratio = settings->value("Disparity", "Uniqueness Ratio", 10);
  _texture_threshold = settings->value("Disparity", "Texture Threshold", 10);
  _speckle_size = settings->value("Disparity", "Speckle Size", 1000);
  _speckle_range = settings->value("Disparity", "Speckle Range", 4);
  _disp12MaxDiff = settings->value("Disparity", "Disp12MaxDiff", 0);
  _p1 = settings->value("Disparity", "P1", 1.19);
  _p2 = settings->value("Disparity", "P2", 1.21);
  _interp = settings->value("Disparity", "Interpolation", false);
  _jr_config_file = settings->value("Disparity", "JR Config File", (std::string)"/home/i3dr/JRIntegration/example/JR_config/JR_matchingparam_without_interpolation.cfg");
  std::cout << "JR Config File: " << _jr_config_file << std::endl;
  if (_jr_config_file != "")
  {
    if (!boost::filesystem::exists(_jr_config_file))
    {
      std::cerr << "Invalid filename for jr config file" << std::endl;
      exit(EXIT_FAILURE);
    }
  }
}

int fromImages(std::string left_camera_info_yaml_path, std::string right_camera_info_yaml_path, std::string left_image_fn, std::string right_image_fn)
{
  Mat frame_split[3];
  Mat frame, image_left, image_right;

  CameraInfoPair cameraInfoPair = CameraInfoPair(left_camera_info_yaml_path, right_camera_info_yaml_path);

  image_left = imread(left_image_fn, CV_LOAD_IMAGE_UNCHANGED);
  image_right = imread(right_image_fn, CV_LOAD_IMAGE_UNCHANGED);

  processImages(image_left, image_right, cameraInfoPair);

  waitKey(0);

  return 0;
}

int liveCapture(std::string left_camera_info_yaml_path, std::string right_camera_info_yaml_path)
{
  Mat frame_split[3];
  Mat frame, image_left, image_right, frame_joint, frame_rect_joint, window_joint;

  CameraInfoPair cameraInfoPair = CameraInfoPair(left_camera_info_yaml_path, right_camera_info_yaml_path);

  VideoCapture cap(0); // open the default camera
  if (!cap.isOpened()) // check if we succeeded
    return -1;

  for (;;)
  {
    cap >> frame; // get a new frame from camera
    split(frame, frame_split);
    image_left = frame_split[1];
    image_right = frame_split[2];

    processImages(image_left, image_right, cameraInfoPair);

    if (waitKey(30) >= 0)
      break;
  }

  return 0;
}

int main(int argc, char **argv)
{
  namedWindow("Camera", 1);
  namedWindow("Disparity", 1);

  //get arguments from command line
  if (argc == 4 || argc == 6)
  {
    std::string disparity_config = argv[1];
    std::string left_camera_info_yaml_path = argv[2];
    std::string right_camera_info_yaml_path = argv[3];
    std::cout << disparity_config << std::endl;
    std::cout << left_camera_info_yaml_path << std::endl;
    std::cout << right_camera_info_yaml_path << std::endl;

    if (!boost::filesystem::exists(disparity_config))
    {
      std::cerr << "Invalid filename for disparity config" << std::endl;
    }
    else if (!boost::filesystem::exists(left_camera_info_yaml_path))
    {
      std::cerr << "Invalid filename for left camera info yaml" << std::endl;
      return -1;
    }
    else if (!boost::filesystem::exists(right_camera_info_yaml_path))
    {
      std::cerr << "Invalid filename for right camera info yaml" << std::endl;
      return -1;
    }
    else
    {
      loadDisparityConfig(disparity_config);
      if (argc == 4)
      {
        return liveCapture(left_camera_info_yaml_path, right_camera_info_yaml_path);
      }
      else if (argc == 6)
      {
        std::string left_image_fn = argv[4];
        std::string right_image_fn = argv[5];
        std::cout << left_image_fn << std::endl;
        std::cout << right_image_fn << std::endl;
        if (!boost::filesystem::exists(left_image_fn))
        {
          std::cerr << "Invalid filename for left image filepath" << std::endl;
        }
        else if (!boost::filesystem::exists(right_image_fn))
        {
          std::cerr << "Invalid filename for right image filepath" << std::endl;
        }
        else
        {
          return fromImages(left_camera_info_yaml_path, right_camera_info_yaml_path, left_image_fn, right_image_fn);
        }
        return -1;
      }
      else
      {
        std::cerr << "Invalid number of arguments. (MUST be 3 if running live or 5 if loading from file)" << std::endl;
        std::cerr << "Format: ./test_JR DISPARITY_CONFIG_FILE LEFT_CAMERA_YAML RIGHT_CAMERA_YAML [optional]LEFT_IMAGE_FILE [optional]RIGHT_IMAGE_FILE" << std::endl;
      }
    }
  }
  else
  {
    std::cerr << "Invalid number of arguments. (MUST be 3 if running live or 5 if loading from file)" << std::endl;
    std::cerr << "Format: ./test_JR DISPARITY_CONFIG_FILE LEFT_CAMERA_YAML RIGHT_CAMERA_YAML [optional]LEFT_IMAGE_FILE [optional]RIGHT_IMAGE_FILE" << std::endl;
  }
  return 0;
}
