#ifndef DO_ONCE_H
#define DO_ONCE_H

class DoOnce {
public:
    virtual void start() = 0;
    virtual void doOnce() = 0;
};

#endif