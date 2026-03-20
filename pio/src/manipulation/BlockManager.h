#pragma once
#include <stdint.h>

class BlockManager { 
private:
    int queue_head = 0;
    int queue_tail = 0;

    // block color enum, if needed
    // enum BlockColor : uint8_t {
    // COLOR_UNKNOWN = 0,
    // COLOR_BLUE    = 1,
    // COLOR_YELLOW  = 2,
    // };

    struct BlockEntry {
        bool BlockColor;
        uint8_t cup_id;  // 0..3
    };

    static constexpr int QUEUE_SIZE = 12;

    BlockEntry block_queue[QUEUE_SIZE];

public:
    BlockManager();

    bool queue_empty();
    bool queue_full();
    bool queue_push(BlockEntry e);
    bool queue_pop(BlockEntry &out);
};