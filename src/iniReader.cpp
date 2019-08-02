#include "iniReader.h"

//convert string to boolean
bool iniReader::str_to_bool(std::string str)
{
    std::transform(str.begin(), str.end(), str.begin(), ::tolower);
    std::istringstream is(str);
    bool b;
    is >> std::boolalpha >> b;
    return b;
}

//convert string to float
float iniReader::str_to_float(std::string str)
{
    float f = std::atof(str.c_str());
    return f;
}

//convert string to double
double iniReader::str_to_double(std::string str)
{
    double d = std::atof(str.c_str());
    return d;
}

//convert string to integer
int iniReader::str_to_int(std::string str)
{
    int i = std::atoi(str.c_str());
    return i;
}

void iniReader::init(std::string ini_file)
{
    ini_settings = extract_settings(ini_file);
}

std::vector<std::string> iniReader::extract_lines(std::string ini_file)
{
    std::vector<std::string> lines;
    std::string line;
    std::ifstream file(ini_file);
    while (std::getline(file, line))
    {
        if (!(line == "" || line == "\n" || line == "\r"))
        {
            line.erase(std::remove(line.begin(), line.end(), '\n'), line.end());
            line.erase(std::remove(line.begin(), line.end(), '\r'), line.end());
            lines.push_back(line);
        }
    }
    return (lines);
}

iniReader::settings iniReader::extract_settings(std::string ini_file)
{
    iniReader::settings set;
    iniReader::settingsGroup setGroup;
    std::vector<std::string> ini_file_lines = extract_lines(ini_file);
    int i = 0;
    for (std::vector<std::string>::iterator it = ini_file_lines.begin(); it != ini_file_lines.end(); ++it)
    {
        std::string line = *it;
        if (line.at(0) == '[' && line.back() == ']')
        {
            //new group found
            if (i != 0)
            {
                //store all vars to this point in previous group
                set.groups.push_back(setGroup);
            }
            //new group
            std::string groupName = line.substr(1, line.size() - 2);
            iniReader::settingsGroup newGroup;
            setGroup = newGroup;
            setGroup.group = groupName;
        }
        else
        {
            iniReader::settingsVar setVar;
            int delimLoc = line.find("=");
            std::string varName = line.substr(0, delimLoc - 1);
            std::string varVal = "";
            if (delimLoc + 2 < line.size())
            {
                varVal = line.substr(delimLoc + 2);
            }
            setVar.val = varVal;
            setVar.var = varName;
            setGroup.vars.push_back(setVar);
        }
        if (i == ini_file_lines.size() - 1)
        {
            iniReader::settingsVar setVar;
            std::string varName = line;
            std::string varVal = line;
            setVar.val = varVal;
            setVar.var = varName;
            setGroup.vars.push_back(setVar);
            set.groups.push_back(setGroup);
        }
        i++;
    }
    return (set);
}

std::string iniReader::processValue(std::string groupName, std::string varName, std::string returnValIfEmpty)
{
    std::vector<iniReader::settingsGroup> groups = ini_settings.groups;
    for (std::vector<iniReader::settingsGroup>::iterator it_g = groups.begin(); it_g != groups.end(); ++it_g)
    {
        iniReader::settingsGroup group = *it_g;
        if (group.group == groupName)
        {
            std::vector<iniReader::settingsVar> vars = group.vars;
            for (std::vector<iniReader::settingsVar>::iterator it_v = vars.begin(); it_v != vars.end(); ++it_v)
            {
                iniReader::settingsVar var = *it_v;
                if (var.var == varName)
                {
                    if (var.val == "" || var.val == " ")
                    {
                        std::cout << "variable '" << varName << "' in group '" << groupName << "' is empty ";
                        std::cout << "using default " << returnValIfEmpty << std::endl;
                        return returnValIfEmpty;
                    }
                    else if (var.var == "None"){
                        return nullptr;
                    }
                    else
                    {
                        return var.val;
                    }
                }
            }
            std::cout << "variable '" << varName << "' not found in group '" << groupName << "' ";
            std::cout << "using default " << returnValIfEmpty << std::endl;
            return returnValIfEmpty;
        }
    }
    std::cerr << "group '" << groupName << "' not found" << std::endl;
    return "";
}

std::string iniReader::value(std::string groupName, std::string varName, std::string returnValIfEmpty)
{
    return processValue(groupName, varName, returnValIfEmpty);
}

bool iniReader::value(std::string groupName, std::string varName, bool returnValIfEmpty)
{
    std::string strReturnValIfEmpty;
    if (returnValIfEmpty){
        strReturnValIfEmpty = "true";
    } else {
        strReturnValIfEmpty = "false";
    }
    std::string value = processValue(groupName, varName, strReturnValIfEmpty);
    bool r_value = str_to_bool(value);
    return r_value;
}

int iniReader::value(std::string groupName, std::string varName, int returnValIfEmpty)
{
    std::string value = processValue(groupName, varName, std::to_string(returnValIfEmpty));
    int r_value = str_to_int(value);
    return r_value;
}

float iniReader::value(std::string groupName, std::string varName, float returnValIfEmpty)
{
    std::string value = processValue(groupName, varName, std::to_string(returnValIfEmpty));
    float r_value = str_to_int(value);
    return r_value;
}

double iniReader::value(std::string groupName, std::string varName, double returnValIfEmpty)
{
    std::string value = processValue(groupName, varName, std::to_string(returnValIfEmpty));
    double r_value = str_to_int(value);
    return r_value;
}

// namespace iniReader