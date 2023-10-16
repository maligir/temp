#ifndef PLACARD_DATABASE_H
#define PLACARD_DATABASE_H

#include "placard.h"
#include <map>
#include <vector>
#include <string>

class PlacardDatabase {
public:
    PlacardDatabase(float dist_thr, float angl_thr, std::string fappend="");

    void AddPlacard(Placard& placard);
    void write(std::string& fname);
    void write_all(std::string& fname);

protected:
    std::vector<std::vector<Placard> > _placards;
    std::vector<Placard> _avg;
    std::vector<size_t> _lastUpdated;
    std::map<std::string, size_t> _label2idx;
    const float _dist_thr;
    const float _angl_thr;
    const bool _append;
    const std::string _fappend;

    void UpdateAvg(size_t idx);
    size_t ManualSearch(Placard& placard);
    void write_append(Placard& placard);

};

#endif