#include "occupation_map_2d.h"

#include <iostream>
#include <fstream>
#include <math.h>

OccupationMap2D::OccupationMap2D(size_t length,
                                size_t width):
                                _length(length)
                                , _width(width)
                                , _resolution(0.05) {
    _data = new uint8_t[length*width];
    for (size_t i = 0; i < length*width; i++) {
        _data[i] = UNKNOWN;
    }

    _offsetx = -length*_resolution/2.0;
    _offsety = -width*_resolution/2.0;
}

OccupationMap2D::~OccupationMap2D() {
    delete _data;
}

size_t OccupationMap2D::GetIdx(float x, float y) {
    x -= _offsetx;
    y -= _offsety;

    long int xidx = floor(x/_resolution + 0.5);
    long int yidx = floor(y/_resolution + 0.5);

    xidx = xidx < 0 ? 0 : xidx;
    yidx = yidx < 0 ? 0 : yidx;

    size_t xi = xidx >= _length ? _length-1 : xidx;
    size_t yi = yidx >= _width  ? _width-1  : yidx;

    return xi*_length + yi;
}

void OccupationMap2D::GetBinMidpoint(size_t idx, float& x, float& y) {
    size_t row = idx / _length;
    size_t col = idx % _length;

    x = row * _resolution + _offsetx;
    y = col * _resolution + _offsety;
}

bool OccupationMap2D::isInBounds(float x, float y) {
    x -= _offsetx;
    y -= _offsety;

    long int xidx = floor(x/_resolution + 0.5);
    long int yidx = floor(y/_resolution + 0.5);

    return !(xidx < 0 ||
             yidx < 0 ||
             xidx >= _length ||
             yidx >= _width);
}

void OccupationMap2D::write(std::string& fname) {
    std::ofstream fmap;
    fmap.open(fname);
    for (size_t i = 0; i < _length*_width; i++) {
        size_t col = i % _length;
        if (i > 0 && col == 0) {
            fmap << "\n";
        }
        fmap << std::to_string(_data[i]);
        if (col != _length -1) {
            fmap << ",";
        }
    }
    fmap.close();
}