//
// Created by Keijo Länsikunnas on 4.2.2024.
//

#include "RingBuffer.h"

RingBuffer::RingBuffer(int size): head{0}, tail{0}, buffer(size) {
}

bool RingBuffer::empty() const {
    return head == tail;
}

bool RingBuffer::full() const {
    return (head + 1) % buffer.size() == tail;
}

bool RingBuffer::put(uint8_t data) {
    // calculate new head (position where to store the value)
    uint32_t nh = (head + 1) % buffer.size();
    // return false if buffer would be full
    if(nh == tail) return false;

    buffer[head] = data;
    head = nh;
    return true;
}

uint8_t RingBuffer::get() {
    uint8_t value = buffer[tail];
    if(head != tail) {
        tail = (tail + 1) % buffer.size();
    }
    return value;
}
