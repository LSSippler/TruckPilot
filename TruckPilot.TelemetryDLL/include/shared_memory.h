#pragma once

#include <windows.h>

class SharedMemory
{
public:
    SharedMemory();
    ~SharedMemory();

    bool Create();
    void* Data() const;
    HANDLE EventHandle() const;

private:
    HANDLE mapping_;
    void* view_;
    HANDLE event_;
};
