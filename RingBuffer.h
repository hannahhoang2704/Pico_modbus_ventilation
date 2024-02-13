//
// Created by Keijo Länsikunnas on 4.2.2024.
//

#ifndef UART_IRQ_RINGBUFFER_H
#define UART_IRQ_RINGBUFFER_H
#include <vector>
#include <cstdint>

class RingBuffer {
public:
    explicit RingBuffer(int size);
    bool empty() const;
    bool full() const;
    bool put(uint8_t data);
    uint8_t get();
private:
    uint32_t head;
    uint32_t tail;
    std::vector<uint8_t> buffer;
};


#endif //UART_IRQ_RINGBUFFER_H
