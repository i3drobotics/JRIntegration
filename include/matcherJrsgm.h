#ifndef MATCHERJRSGM_H
#define MATCHERJRSGM_H

#include <PhobosIntegration/PhobosIntegration.hpp>
#include <iniReader.h>

class MatcherJrSGM{
    public:
        MatcherJrSGM(std::string &sConfigFile){
            init(sConfigFile);
        }
    
        ~MatcherJrSGM(void){
            if (matcher_handle != nullptr){
                JR::Phobos::DestroyMatchStereoHandle(matcher_handle);
            }
        }

        void parseConfig(std::string input_file);
        int getErrorDisparity();

        void writeConfig(const std::string &sConfigFile);
        void readConfig(std::string &sConfigFile);
        void setConfig();

        void setDisparityShift(int shift);
        void setDisparityRange(int n);
        void enableSubpixel(bool enable);
        void setMatchCosts(float P1, float P2);
        void setWindowSize(int census_size);
        void enableInterpolation(bool enable);
        void enableOcclusionDetection(bool enable);

        float getP1(void){ return params.oPyramidParams[0].oSGMParams.fP1_E_W; }
        float getP2(void){ return params.oPyramidParams[0].oSGMParams.fP2_E_W; }
        int getDisparityRange(void){ return params.oPyramidParams[0].nMaximumNumberOfDisparities; }
        int getCensusSize(void){ return params.oPyramidParams[0].oMetricParams.nWindowSizeX; }
        bool getInterpolate(void){return params.oFinalSubPixelParameters.bInterpol; }
        bool getOcclusionDetect(void){ return params.oFinalSubPixelParameters.bOcclusionDetection; }
        bool getSubpixel(void){return params.oFinalSubPixelParameters.bCompute; }
        int getDisparityShift(void){return params.fTopPredictionShift * pow(2, params.nNumberOfPyramids-1) ; }

        void compute(cv::Mat left_image, cv::Mat right_image, cv::Mat disp);
        void backwardMatch(cv::Mat left_image, cv::Mat right_image, cv::Mat disp);

    private:
        JR::Phobos::TSTEREOHANDLE matcher_handle = nullptr;
        JR::Phobos::SMatchingParametersInput params;
        int min_disparity, disparity_range;

        void init(std::string &sConfigFile);
        void initParameters(int nPyramids, JR::Phobos::SSGMParameters sgm_params);
        JR::Phobos::SMatchingParameters parsePyramid(iniReader *settings, std::string group);
        JR::Phobos::SSGMParameters initSGMParameters();
};

#endif // MATCHERJRSGM_H