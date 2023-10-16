#ifndef _OCCUPATION_MAP_2D_H
#define _OCCUPATION_MAP_2D_H

#include <stdint.h>
#include <stddef.h>
#include <string>

#define OCCUPIED 255
#define UNKNOWN 127
#define FREE 0

class OccupationMap2D {
public:
    OccupationMap2D(size_t length, size_t width);
    ~OccupationMap2D();

    bool isInBounds(float x, float y);
    size_t GetIdx(float x, float y);
    inline uint8_t GetValue(size_t idx) {return _data[idx];}
    inline uint8_t GetValue(float x, float y) {return GetValue(GetIdx(x, y));}
    void GetBinMidpoint(size_t idx, float& x, float& y);

    inline void SetResolution(float resolution) {_resolution = resolution;}
    inline void SetOffset(float x, float y) {_offsetx = x; _offsety = y;}
    inline void SetValue(size_t idx, uint8_t value) {_data[idx] = value;}
    inline void SetValue(float x, float y, uint8_t value) {SetValue(GetIdx(x, y), value);}
    inline void CenterMap() {
        _offsetx = -(_length * _resolution / 2.0);
        _offsety = -(_width * _resolution / 2.0);
    }

    inline uint8_t* GetBuffer() {return _data;}

    void write(std::string& fname);

    float _resolution;
    const size_t _length;
    const size_t _width;

protected:
    uint8_t* _data;
    float _offsetx;
    float _offsety;
};

#endif