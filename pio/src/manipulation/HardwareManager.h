#pragma once
#include "BlockManager.h"

class HardwareManager {
private:
    BlockManager* BlockList;

public:
    HardwareManager(BlockManager* BlockList);
    void init();
    void pickUp(int* cups);
    void dropOff();
};