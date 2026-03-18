#include "BlockManager.h"

BlockManager::BlockManager() {}

bool BlockManager::queue_empty() {
  return queue_head == queue_tail;
}

bool BlockManager::queue_full() {
  return ((queue_tail + 1) % QUEUE_SIZE) == queue_head;
}

bool BlockManager::queue_push(BlockEntry e) {
  if (queue_full()) return false;
  block_queue[queue_tail] = e;
  queue_tail = (queue_tail + 1) % QUEUE_SIZE;
  return true;
}

bool BlockManager::queue_pop(BlockEntry &out) {
  if (queue_empty()) return false;
  out = block_queue[queue_head];
  queue_head = (queue_head + 1) % QUEUE_SIZE;
  return true;
}