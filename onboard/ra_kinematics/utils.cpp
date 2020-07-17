#include "json.hpp"
#include "utils.hpp"
#include <fstream>
#include <iostream>

using namespace nlohmann;
using namespace std;

json read_json_from_file(string filepath) {
    ifstream file(filepath);

    json config;

    file >> config;
    return config;
}