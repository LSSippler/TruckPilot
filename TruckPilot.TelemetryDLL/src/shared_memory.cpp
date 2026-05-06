#include "shared_memory.h"

SharedMemory::SharedMemory() : mapping_(nullptr), view_(nullptr), event_(nullptr) {}

SharedMemory::~SharedMemory()
{
    if (view_)
        UnmapViewOfFile(view_);
    if (mapping_)
        CloseHandle(mapping_);
    if (event_)
        CloseHandle(event_);
}

bool SharedMemory::Create()
{
    mapping_ = CreateFileMappingA(INVALID_HANDLE_VALUE, nullptr, PAGE_READWRITE, 0, 512, "Local\\TruckPilotTelemetry");
    if (!mapping_)
        return false;

    view_ = MapViewOfFile(mapping_, FILE_MAP_WRITE, 0, 0, 512);
    if (!view_) {
        CloseHandle(mapping_);
        mapping_ = nullptr;
        return false;
    }

    event_ = CreateEventA(nullptr, TRUE, FALSE, "Local\\TruckPilotTelemetryReady");
    if (!event_) {
        UnmapViewOfFile(view_);
        view_ = nullptr;
        CloseHandle(mapping_);
        mapping_ = nullptr;
        return false;
    }

    return true;
}

void* SharedMemory::Data() const
{
    return view_;
}

HANDLE SharedMemory::EventHandle() const
{
    return event_;
}
