/*
 * Authors: Neil Weidinger
 * Organisation: HYPED
 * Date: April 2020
 * Description:
 *
 *    Copyright 2020 HYPED
 *    Licensed under the Apache License, Version 2.0 (the "License");
 *    you may not use this file except in compliance with the License.
 *    You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 *    Unless required by applicable law or agreed to in writing, software
 *    distributed under the License is distributed on an "AS IS" BASIS,
 *    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *    See the License for the specific language governing permissions and
 *    limitations under the License.
 */

#ifndef UTILS_RINGBUFFER_HPP_
#define UTILS_RINGBUFFER_HPP_

#include <array>
#include <string>
#include <cstdio>

#include "utils/concurrent/lock.hpp"

namespace hyped {
namespace utils {

using concurrent::Lock;
using concurrent::ScopedLock;

namespace {
Lock stdout_lock;

template <std::size_t N>
class RingBuffer {
    public:
        void enqueue(int thread_num, int iteration, int random_num) {
            char* buf = new char[100];  // wow i hate this, very dangerous
            std::snprintf(buf, 100, "LOGGER BENCHMARK: Thread %d Iteration %d: random value %d\n", \
                         thread_num, iteration, random_num);

            if (!this->spaceAvailable(std::strlen(buf))) {
                this->flush();
            }

            for (int i = 0; buf[i] != '\0'; i++) {
                enqueue(buf[i]);
            }

            delete[] buf;
        }

        void flush() {
            ScopedLock scoped_lock(&stdout_lock);

            if (tail_ > head_) {
                std::fwrite((buffer_.data() + head_), sizeof(char), (tail_ - head_), stdout);
            } else {
                std::fwrite((buffer_.data() + head_), sizeof(char), (capacity_ - head_), stdout);
                std::fwrite(buffer_.data(), sizeof(char), (tail_), stdout);
            }

            head_ = tail_;
            size_ = 0;
        }

        bool spaceAvailable(std::size_t len) const {
            return (capacity_ - size_) > len;
        }

    private:
        std::array<char, N> buffer_;
        std::size_t head_ = 0;  // read from head
        std::size_t tail_ = 0;  // write into tail
        std::size_t size_ = 0;
        const std::size_t capacity_ = N;

        void enqueue(char c) {
            buffer_[tail_] = c;
            tail_ = (tail_ + 1) % capacity_;
            size_++;
        }
};

}  // unnamed namespace

}  // namespace utils
}  // namespace hyped

#endif // UTILS_RINGBUFFER_HPP_
