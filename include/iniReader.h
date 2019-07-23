#ifndef INIREADER_H
#define INIREADER_H

#include <string>
#include <vector>
#include <algorithm>
#include <sstream>
#include <iostream>
#include <fstream>

class iniReader {

    public:

        struct settingsVar {
            std::string var;
            std::string val;
        };
        struct settingsGroup {
            std::string group;
            std::vector<settingsVar> vars;
        };
        struct settings {
            std::vector<settingsGroup> groups;
        };
    
        iniReader(std::string ini_file){
            init(ini_file);
        }
        bool value(std::string groupName, std::string varName, bool returnValIfEmpty);
        std::string value(std::string groupName, std::string varName, std::string returnValIfEmpty);
        int value(std::string groupName, std::string varName, int returnValIfEmpty);
        float value(std::string groupName, std::string varName, float returnValIfEmpty);
        double value(std::string groupName, std::string varName, double returnValIfEmpty);

    private:
        std::string processValue(std::string groupName, std::string varName, std::string returnValIfEmpty);

        std::string ini_filepath = "multigpusgm_param_test.param";
        iniReader::settings ini_settings;
        void init(std::string ini_file);
        std::vector<std::string> extract_lines(std::string ini_file);
        iniReader::settings extract_settings(std::string ini_file);

        bool str_to_bool(std::string str);
        float str_to_float(std::string str);
        double str_to_double(std::string str);
        int str_to_int(std::string str);
};

#endif // INIREADER_H