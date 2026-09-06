#include "AsyncLogging.h"

#include <cstdio>
#include <functional>

#include "LogFile.h"
#include "Timestamp.h"

namespace mhost {

AsyncLogging::AsyncLogging(std::string basename, size_t rollSize, int flushInterval)
    : flushInterval_(flushInterval),
      running_(false),
      basename_(std::move(basename)),
      rollSize_(rollSize),
      thread_(),
      mutex_(),
      cond_(),
      currentBuffer_(new Buffer),
      nextBuffer_(new Buffer),
      buffers_() {
  buffers_.reserve(16);
}

AsyncLogging::~AsyncLogging() {
  if (running_.load()) {
    stop();
  }
}

void AsyncLogging::append(const char* logline, int len) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (currentBuffer_->avail() > static_cast<int>(len)) {
    currentBuffer_->append(logline, static_cast<size_t>(len));
  } else {
    // 当前缓冲已满：移入待写队列，换上备用缓冲（无备用则临时分配，罕见）
    buffers_.push_back(std::move(currentBuffer_));
    if (nextBuffer_) {
      currentBuffer_ = std::move(nextBuffer_);
    } else {
      currentBuffer_.reset(new Buffer);  // 极端积压：后台缓冲尚未归还
    }
    currentBuffer_->append(logline, static_cast<size_t>(len));
    cond_.notify_one();
  }
}

void AsyncLogging::start() {
  running_ = true;
  thread_ = std::thread(std::bind(&AsyncLogging::threadFunc, this));
}

void AsyncLogging::stop() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    running_ = false;
  }
  cond_.notify_one();
  if (thread_.joinable()) {
    thread_.join();
  }
}

void AsyncLogging::threadFunc() {
  LogFile output(basename_, rollSize_, false);
  BufferPtr newBuffer1(new Buffer);
  BufferPtr newBuffer2(new Buffer);
  newBuffer1->bzero();
  newBuffer2->bzero();
  BufferVector buffersToWrite;
  buffersToWrite.reserve(16);

  while (running_.load()) {
    assert(newBuffer1 && newBuffer2);
    assert(buffersToWrite.empty());
    {
      // 等待：被前端换缓冲唤醒，或 flushInterval_ 兜底到时（避免低速率日志
      // 长时间滞留在前台缓冲里）
      std::unique_lock<std::mutex> lock(mutex_);
      if (buffers_.empty()) {
        cond_.wait_for(lock, std::chrono::seconds(flushInterval_));
      }
      buffers_.push_back(std::move(currentBuffer_));
      currentBuffer_ = std::move(newBuffer1);
      buffersToWrite.swap(buffers_);
      if (!nextBuffer_) {
        nextBuffer_ = std::move(newBuffer2);
      }
    }

    assert(!buffersToWrite.empty());

    // 日志风暴保护：待写超过 25 块时只保留最新 2 块，其余丢弃
    if (buffersToWrite.size() > 25) {
      char buf[96];
      ::snprintf(buf, sizeof buf,
                 "Dropped log messages at %s: %zu larger buffers\n",
                 Timestamp::now().toFormattedString().c_str(),
                 buffersToWrite.size() - 2);
      ::fputs(buf, stderr);
      output.append(buf, static_cast<int>(::strlen(buf)));
      buffersToWrite.erase(buffersToWrite.begin() + 2, buffersToWrite.end());
    }

    for (const auto& buffer : buffersToWrite) {
      output.append(buffer->data(), buffer->length());
    }

    // 缓冲回收：写完的缓冲还池，稳态下 append 路径零动态分配
    if (buffersToWrite.size() > 2) {
      buffersToWrite.resize(2);
    } else if (!newBuffer1) {
      newBuffer1 = std::move(buffersToWrite.back());
      buffersToWrite.pop_back();
      newBuffer1->reset();
    }
    if (!newBuffer2) {
      newBuffer2 = std::move(buffersToWrite.back());
      buffersToWrite.pop_back();
      newBuffer2->reset();
    }
    buffersToWrite.clear();
    output.flush();
  }

  output.flush();
}

}  // namespace mhost
