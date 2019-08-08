#include <matcherJrsgm.h>

//Initialise matcher
void MatcherJrSGM::init(std::string &sConfigFile)
{
  std::cout << sConfigFile << std::endl;
  //readConfig(sConfigFile);
  JR::Phobos::ReadIniFile( params, sConfigFile );

  //set default values
  //enableInterpolation(false);
  //enableOcclusionDetection(false);
  //setWindowSize(9);
  //enableSubpixel(false);

  min_disparity = 0;
  disparity_range = 65;

  //setDisparityRange(21);
  //setMatchCosts(0.5, 1.5);
  matcher_handle = JR::Phobos::CreateMatchStereoHandle(params);
}

int MatcherJrSGM::getErrorDisparity(void)
{
  return -10000;
}

//compute disparity
void MatcherJrSGM::compute(cv::Mat left_image, cv::Mat right_image, cv::Mat &disp)
{
  cv::Mat left_joint, right_joint;
  std::string sgm_log = "/home/i3dr/Desktop/sgm_log.txt";
  try
  {
    JR::Phobos::MatchStereo(matcher_handle,
                            left_image,
                            right_image,
                            left_joint,
                            right_joint,
                            disp,
                            sgm_log,
                            JR::Phobos::e_logLog);
  }
  catch (const std::exception &ex)
  {
    std::cout << ex.what() << std::endl;
  }
}

//backward match disparity
void MatcherJrSGM::backwardMatch(cv::Mat left_image, cv::Mat right_image, cv::Mat &disp)
{
  cv::Mat left_joint, right_joint;
  std::string sgm_log = "./sgm_log.txt";

  JR::Phobos::MatchStereo(matcher_handle,
                          right_image,
                          left_image,
                          left_joint,
                          right_joint,
                          disp,
                          sgm_log,
                          JR::Phobos::e_logLog);
}

void MatcherJrSGM::setMatchCosts(float P1, float P2)
{
  for (auto &pyramid : params.oPyramidParams)
  {
    pyramid.oSGMParams.fP1_E_W = P1;
    pyramid.oSGMParams.fP1_SE_NW = P1;
    pyramid.oSGMParams.fP1_SW_NE = P1;
    pyramid.oSGMParams.fP1_S_N = P1;

    pyramid.oSGMParams.fP2_E_W = P2;
    pyramid.oSGMParams.fP2_SE_NW = P2;
    pyramid.oSGMParams.fP2_SW_NE = P2;
    pyramid.oSGMParams.fP2_S_N = P2;
  }

  setConfig();
}

void MatcherJrSGM::setWindowSize(int census_size)
{
  for (auto &pyramid : params.oPyramidParams)
  {
    pyramid.oMetricParams.nWindowSizeX = census_size;
    pyramid.oMetricParams.nWindowSizeY = census_size;
  }

  setConfig();
}

void MatcherJrSGM::setDisparityShift(int shift)
{
  params.fTopPredictionShift = shift / pow(2, params.nNumberOfPyramids - 1);
  min_disparity = shift;

  setConfig();
}

void MatcherJrSGM::enableSubpixel(bool enable)
{
  params.oFinalSubPixelParameters.bCompute = enable;

  setConfig();
}

void MatcherJrSGM::setDisparityRange(int n)
{
  /* Set disparity range for all pyramids */
  if (n % 2)
  {
    disparity_range = n;
    params.oPyramidParams[0].nMaximumNumberOfDisparities = n;
    params.oPyramidParams[1].nMaximumNumberOfDisparities = n;
    params.oPyramidParams[2].nMaximumNumberOfDisparities = n;
    params.oPyramidParams[3].nMaximumNumberOfDisparities = n;
    params.oPyramidParams[4].nMaximumNumberOfDisparities = n;

    params.oFinalSubPixelParameters.nMaximumNumberOfDisparities = n;
  }

  setConfig();
}

void MatcherJrSGM::enableInterpolation(bool enable)
{
  /* Toggle interpolation */
  for (auto &pyramid : params.oPyramidParams)
  {
    pyramid.bInterpol = enable;
  }

  params.oFinalSubPixelParameters.bInterpol = enable;
  setConfig();
}

void MatcherJrSGM::enableOcclusionDetection(bool enable)
{
  /* Toggle occlusion detection */
  for (auto &pyramid : params.oPyramidParams)
  {
    pyramid.bOcclusionDetection = enable;
  }

  params.oFinalSubPixelParameters.bOcclusionDetection = enable;

  setConfig();
}

void MatcherJrSGM::setConfig()
{
  /* Call to update the current configuration */
  if (matcher_handle != nullptr)
  {
    JR::Phobos::DestroyMatchStereoHandle(matcher_handle);
  }
  matcher_handle = JR::Phobos::CreateMatchStereoHandle(params);
}