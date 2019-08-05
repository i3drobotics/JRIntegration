#include <matcherJrsgm.h>

//Initialise matcher
void MatcherJrSGM::init(std::string &sConfigFile)
{
  std::cout << sConfigFile << std::endl;
  readConfig(sConfigFile);

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

void MatcherJrSGM::readConfig(std::string &sConfigFile)
{
  /* Load a configuration file for the matcher */
  iniReader *settings = new iniReader(sConfigFile);

  params.oGPUs = std::vector<int>(1, 0);
  params.bHasNodata = settings->value("Input", "Input Has Nodata", false);
  params.fNodataValue = settings->value("Input", "Nodata Value", -99999.000000);

  params.bWriteBackDisp = settings->value("Output", "Write Back Disparity", false);
  params.bWriteRevBackMatchDistance = settings->value("Output", "Write Reverse Back Match Distance", false);
  params.bWriteBackMatchDistance = settings->value("Output", "Write Back Match Distance", false);

  params.nNumberOfPyramids = settings->value("Parameter", "Pyramid Levels", 5);
  params.strDispPredictionFile = settings->value("Parameter", "Back Disparity Prediction Input File Pattern", "");
  params.strBackDispPredictionFile = settings->value("Parameter", "Disparity Prediction Input File Pattern", "");
  params.fTopPredictionShift = settings->value("Parameter", "Top Prediction Shift", 0);
  params.nNumberOfMorphologicalIter = settings->value("Parameter", "Morphological Iterations", 0);
  params.nDebugLevel = settings->value("Parameter", "Debug Output Mode", 0);
  params.strDebugOutput = settings->value("Parameter", "Debug Output Path", "");

  params.oPyramidParams.resize(static_cast<size_t>(params.nNumberOfPyramids));
  params.nJointMode = 0;

  for (int i = 0; i < params.nNumberOfPyramids; i++)
  {
    std::ostringstream oss;
    oss << "Pyramid " << i;
    std::string group = oss.str();
    params.oPyramidParams.at(i) = parsePyramid(settings, group);
  }

  std::ostringstream oss;
  oss << "Pyramid " << params.nNumberOfPyramids;
  std::string group = oss.str();
  params.oFinalSubPixelParameters = parsePyramid(settings, group);
}

JR::Phobos::SMatchingParameters MatcherJrSGM::parsePyramid(iniReader *settings, std::string group)
{
  /* Load in matching parameters for a particular pyramid */

  JR::Phobos::SMatchingParameters pyramid;

  pyramid.bCompute = settings->value(group, "Process This Pyramid", true);
  pyramid.nMetric = 0; // Census cost
  pyramid.nMaximumNumberOfDisparities = settings->value(group, "Number Of Disparities", 31);
  pyramid.fDisparityStepSize = settings->value(group, "Disparity Step Size", 0.5);
  pyramid.nSubpixelFnc = 2;
  pyramid.fMaxCost = settings->value(group, "Maximum Feature Distance", 6);
  pyramid.bDoSubpixelNormalization = settings->value(group, "Do Subpixel Normalization", true);
  pyramid.bBackMatch = settings->value(group, "Compute Backmatching", false);
  pyramid.bComputeBackDisparity = settings->value(group, "Compute Back Disparity", false);

  pyramid.fMaxBackMatchingDist = settings->value(group, "Maximum Backmatching Distance", 1.5);
  pyramid.bKeepValueOnNodataInBackmatch = settings->value(group, "Maximum Backmatching Distance - Keep Value On Nodata In Backmatch", false);
  pyramid.bKeepValueOnOutsideImage = settings->value(group, "Maximum Backmatching Distance - Keep Value On Outside Image In Backmatch", false);
  pyramid.bMultiGPU_AfterDSI = settings->value(group, "MultiGPU Implementation After DSI", false);

  pyramid.nRegionGrowingMode = 1;
  pyramid.nRegionGrowingIterations = 1;

  // manually added from GUI
  pyramid.bMedian = settings->value(group, "Disparity Median Optimizer", true); // use median optimizer
  pyramid.iMedX = settings->value(group, "Disparity Median Optimizer Kernel Size X", 3);
  pyramid.iMedY = settings->value(group, "Disparity Median Optimizer Kernel Size Y", 3);
  pyramid.iNodataPol = 1; //"Disparity Median Optimizer Nodata Policy"
  pyramid.iBorderPol = 2; //"Disparity Median Optimizer Border Policy"
  pyramid.fMedianThresh = settings->value(group, "Disparity Median Optimizer Threshold", -1);
  pyramid.bSplittedMedian = settings->value(group, "Disparity Median Optimizer Splitted Version", false); //"Disparity Median Optimizer Splitted Version"

  pyramid.bSpeckle = settings->value(group, "Disparity Speckle Filter Optimizer", true);             // use speckle optimizer
  pyramid.iSpeckleMaxSize = settings->value(group, "Disparity Speckle Filter Max Region Size", 200);  //"Disparity Speckle Filter Max Region Size"
  pyramid.fSpeckleMaxDiff = settings->value(group, "Disparity Speckle Filter Max Difference", 0.5); //"Disparity Speckle Filter Max Difference"

  pyramid.bInterpol = settings->value(group, "Interpolate Disparity", true);     // "Interpolate Disparity"
  pyramid.nDir = settings->value(group, "Interpolator Number Of Directions", 32); // "Interpolator Number Of Directions"
  //pyramid.nNth = settings->value(group,"Interpolator Number Of Nth Elements", 32);            // "Interpolator Number Nth Elements"
  pyramid.nNth =settings->value(group, "Interpolator Number Nth Elements", 32);          // "Interpolator Number Nth Elements"
  pyramid.nMinNth = settings->value(group, "Interpolator Minimum Number Of Elements", 0); // "Interpolator Minimum Number Of Elements"
  //pyramid.fRadius = settings->value(group,"Match Radius", -1);         // "Interpolator Match Radius"
  pyramid.fRadius = settings->value(group, "Interpolator Match Radius", -1); // "Interpolator Match Radius"
  pyramid.nMode = 2;                                                                       // "Interpolator Mode" (Gauss);

  pyramid.bOcclusionDetection = settings->value(group, "Occlusion Detection", true);
  pyramid.fOcclusionDispGapSize = settings->value(group, "Occlusion Detector Step Size", 1.1);
  pyramid.bOccInterpol = settings->value(group, "Interpolate Occlusions", true);              // "Interpolate Occlusions"
  pyramid.nOccDir = settings->value(group, "Occlusion Interpolator Number Of Directions", 32); // "Occlusion Interpolator Number Of Directions"
  pyramid.nOccNth = settings->value(group, "Occlusion Interpolator Number Nth Elements", 32);  // "Occlusion Interpolator Number Nth Elements"
  pyramid.nOccMinNth = settings->value(group, "Occlusion Interpolator Minimum Number Of Elements", 0);
  pyramid.fOccRadius = settings->value(group, "Occlusion Interpolator Match Radius", -1); // "Occlusion Interpolator Match Radius"
  pyramid.nOccMode = 2;                                                                                   // "Occlusion Interpolator Mode" (Percentile 10)

  pyramid.nRegionGrowingMode = 1; //0 no, 1 iterations, 2//full
  pyramid.nRegionGrowingIterations = settings->value(group, "Occlusion Region Growing Iterations", 5);

  pyramid.oMetricParams.nWindowSizeX = settings->value(group, "Feature Set Size X", 9);
  pyramid.oMetricParams.nWindowSizeY = settings->value(group, "Feature Set Size Y", 9);
  pyramid.oMetricParams.fMetricDeltaX = settings->value(group, "Feature Set Delta Step Size X", 1);
  pyramid.oMetricParams.fMetricDeltaY = settings->value(group, "Feature Set Delta Step Size Y", 1);
  pyramid.oMetricParams.nCensusCenterFunction = 1; // Census Center Function
  pyramid.oMetricParams.nNumberOfScales = settings->value(group, "Feature Set Slanted Window Number Of Scales", 0);
  //pyramid.oMetricParams.fScaleStepSize = str_to_float(settings->value(group,"Feature Set Slanted Window Shift Step Size", 0.250000);
  pyramid.oMetricParams.fScaleStepSize = settings->value(group, "Feature Set Slanted Window Scale Step Size", 0.250000);
  pyramid.oMetricParams.nNumberOfSlants = settings->value(group, "Feature Set Slanted Window Number Of Shifts", 0);
  //pyramid.oMetricParams.fSlantStepSize = str_to_float(settings->value(group,"Feature Set Slanted Window Slant Step Size", 0.250000);
  pyramid.oMetricParams.fSlantStepSize = settings->value(group, "Feature Set Slanted Window Shift Step Size", 0.250000);
  pyramid.oMetricParams.bUseProximityWeight = 0;
  pyramid.oMetricParams.fProximityGamma = settings->value(group, "Feature Set Proximity Weight", 2);
  pyramid.oMetricParams.bUseSimilarityWeight = 0;
  pyramid.oMetricParams.fSimilarityGamma = settings->value(group, "Feature Set Similarity Weight", 2);

  JR::Phobos::SSGMParameters oSGMParams;

  oSGMParams.nAllowedOutsideImageMatchCount = settings->value(group, "DSI Allowed Outside Image Disparities", 2147483647);
  oSGMParams.bDSICountCorrection = settings->value(group, "DSI Count Correction", true);
  oSGMParams.nDSICountCorrectionNoDataCheckMode = 1;

  // Up/Down
  oSGMParams.bMatch_S_N = settings->value(group, "SGM South-North Optimization", true);
  oSGMParams.fP1_S_N = settings->value(group, "SN Penalty 1", 0.100000);
  oSGMParams.fP2_S_N = settings->value(group, "SN Penalty 2", 0.800000);
  oSGMParams.bP2EqualP1_S_N = 0;
  oSGMParams.nAdaptiveP1Fnc_S_N = 0;
  oSGMParams.nAdaptiveP2Fnc_S_N = 0;
  oSGMParams.bAdditive_S_N = settings->value(group, "SN Additive", true);

  // Left/Right
  oSGMParams.bMatch_E_W = settings->value(group, "SGM West-East Optimization", true);
  oSGMParams.fP1_E_W = settings->value(group, "WE Penalty 1", 0.1);
  oSGMParams.fP2_E_W = settings->value(group, "WE Penalty 1", 0.8);
  oSGMParams.bP2EqualP1_E_W = 0;
  oSGMParams.nAdaptiveP1Fnc_E_W = 0; // None
  oSGMParams.nAdaptiveP2Fnc_E_W = 0; // Inverse gradient
  oSGMParams.bAdditive_E_W = settings->value(group, "WE Additive", true);

  // Diagonal
  oSGMParams.bMatch_SW_NE = settings->value(group, "SGM SouthWest-NorthEast Optimization", true);
  oSGMParams.fP1_SW_NE = settings->value(group, "SW-NE Penalty 1", 0.1);
  oSGMParams.fP2_SW_NE = settings->value(group, "SW-NE Penalty 2", 0.8);
  oSGMParams.bP2EqualP1_SW_NE = 0;
  oSGMParams.nAdaptiveP1Fnc_SW_NE = 0;
  oSGMParams.nAdaptiveP2Fnc_SW_NE = 0;
  oSGMParams.bAdditive_SW_NE = settings->value(group, "SW-NE Additive", true);

  // Diagonal
  oSGMParams.bMatch_SE_NW = settings->value(group, "SGM SouthEast-NorthWest Optimization", true);
  oSGMParams.fP1_SE_NW = settings->value(group, "SE-NW Penalty 1", 0.1);
  oSGMParams.fP2_SE_NW = settings->value(group, "SE-NW Penalty 2", 0.8);
  oSGMParams.bP2EqualP1_SE_NW = 0;
  oSGMParams.nAdaptiveP1Fnc_SE_NW = 0;
  oSGMParams.nAdaptiveP2Fnc_SE_NW = 0;
  oSGMParams.bAdditive_SE_NW = settings->value(group, "SE-NW Additive", true);

  //oSGMParams.bUseDSITexture = str_to_bool(settings->value(group,"Use DSI Texture Memory", false);
  oSGMParams.bUseDSITexture = settings->value(group, "Use DSI  Texture Memory", false);
  oSGMParams.bUseAggregatedCostTexture = settings->value(group, "Use Aggregated Cost Texture Memory", true);

  pyramid.oSGMParams = oSGMParams;

  return pyramid;
}

void MatcherJrSGM::writeConfig(const std::string &sConfigFile)
{
  //TODO
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