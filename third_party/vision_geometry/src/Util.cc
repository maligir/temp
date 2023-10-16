#include <vision_geometry/Util.h>

#include <dirent.h>

#include <cstdlib>
#include <iostream>

using namespace std;

double randZeroToOne() {
    return (double)rand()/(double)RAND_MAX;
}

vector<string> listFiles(string directory) {
    vector<string> retVal;

    DIR *dir;
    struct dirent *ent;
    if ((dir = opendir (directory.c_str())) != NULL) {
        /* print all the files and directories within directory */
        while ((ent = readdir (dir)) != NULL) {
            string n = ent->d_name;
            if(!((n.compare(".") == 0) || (n.compare("..") == 0)))
                retVal.push_back(directory + n);
        }
        closedir (dir);
    }

    return retVal;
}

void printStrings(std::string prefix, std::vector<std::string> s) {
    for(size_t i = 0; i < s.size(); i++) cout << prefix << s[i] << endl;
}
