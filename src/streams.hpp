/** @file streams.hpp
 * @author Irreq
 * @brief Manages ring buffers used in streaming data.
 */

#ifndef BEAMFORMER_STREAMS_H
#define BEAMFORMER_STREAMS_H


#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/memfd.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/syscall.h>
#include <sys/types.h>
#include <unistd.h>

#include <iostream>
#include <unordered_map>

#include "config.h"

#define PAGE_SIZE getpagesize()

#define N_ITEMS_BUFFER PAGE_SIZE / sizeof(float)

#define BUFFER_BYTES N_SAMPLES * sizeof(float)

/**
 * @class Streams
 * @brief Manages multiple ring buffers for streaming data.
 */
/**
 * Multiple ring-buffers can be thought of as:
 * 
 * id       ring-buffer
 * 
 * 0: [0 1 2 3 ... 0 1 2 3]
 * 1: [0 1 2 3 ... 0 1 2 3]
 * 2: [0 1 2 3 ... 0 1 2 3]
 *              ·
 *              ·
 *              ·
 * x: [0 1 2 3 ... 0 1 2 3]
 * 
 */
class Streams {
public:
    /// Map to store ring buffers, where the key is the buffer index and the value is a pointer to the buffer.
    std::unordered_map<unsigned, float *> buffers;

    /// Reader start position in ring-buffer
    unsigned position;
    int _position;

    /// Default constructor.
    Streams() : position(0), _position(0) {};

    /**
     * @brief Create a ring-buffer for a specific index.
     * @param index The index of the ring buffer to create.
     * @return true if success, false otherwise.
     */
    bool create_stream(unsigned index) {
        if (this->buffers.find(index) != this->buffers.end()) {
            std::cout << "Stream: " << index << " already exists" << std::endl;
            return false;
        }

        this->buffers[index] = allocate_buffer();

        if (this->buffers[index] == nullptr) {
            std::cout << "Unable to allocate virtual memory for stream" << std::endl;
            this->buffers.erase(index);
            return false;
        }

        return true;
    }

    /**
     * @brief Gets a pointer to a specific location in the ring buffer with an offset. 
     * @param index The index of the buffer.
     * @param offset The offset from the current position.
     * @return Pointer to the buffer location.
     */
    float *get_signal(unsigned index, int offset) {
        return (float *) ((char *) this->buffers[index] + this->position + offset * sizeof(float));
    }

    /**
     * @brief Fast write to buffer using wrap-around memcpy
     * @param index The index of the buffer.
     * @param data Pointer to the data to write.
     */
    inline void write_stream(unsigned index, float *data) {
        memcpy((char *) this->buffers[index] + this->position, data, BUFFER_BYTES);
    }

    /**
     * @brief Fast read from buffer using wrap-around memcpy
     * @param index The index of the buffer.
     * @param data Pointer to where the data will be read into.
     * @param offset The offset from the current position.
     */
    inline void read_stream(unsigned index, float *data, unsigned offset = 0) {
        //memcpy(data, &buffers[index][_position + offset], N_ITEMS_BUFFER);
        memcpy(data, (float *) ((char *) this->buffers[index] + this->position + offset * sizeof(float)), PAGE_SIZE);
    }

#if 1// Only debugging
    void dump_stream(unsigned index, float *data) {
        memcpy(data, (char *) this->buffers[index], PAGE_SIZE);
    }
#endif
    /**
     * @brief Destructor, cleanups of resources.
     */
    ~Streams() {
        for (auto itr = this->buffers.begin(); itr != this->buffers.end(); ++itr) {
            // Free memory
            munmap(itr->second, PAGE_SIZE * 2);
        }
    }

    /**
     * @brief Reserved only for producer, advances the reader position in the buffer.
     */
    void forward() {
        this->position = (this->position + BUFFER_BYTES) % PAGE_SIZE;
        //_position = (_position + N_SAMPLES) % N_ITEMS_BUFFER;
    }

private:
    /**
     * @brief Allocate virtual memory ring buffers.
     * 
     * We use two pages of contigious memory mapped to the same region
     * meaning that when data overflows from region1 over to region2
     * it is mapped to the beginning of the "actual" buffer. As long 
     * as reading and writing is less than the buffer size, no other checks 
     * must be done making it a very optimized buffer.
     * @return Pointer to the allocated buffer, or `nullptr` if allocation fails.
     */
    float *allocate_buffer() {
        int fd = memfd_create("float", 0);
        if (fd == -1) {
            printf("Unable to create fd\n");
            return nullptr;
        }

        if (ftruncate(fd, 2 * PAGE_SIZE) == -1) {
            printf("Unable to resize fd\n");
            return nullptr;
        }

        // Create first region
        float *pages = (float *) mmap(0, 2 * PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
        if (pages == MAP_FAILED) {
            printf("Unable to create first region\n");
            close(fd);
            return nullptr;
        }

        if (mmap((char *) pages + PAGE_SIZE, PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_FIXED, fd, 0) == MAP_FAILED) {
            printf("Unable to create reg1\n");
            close(fd);
            munmap(pages, PAGE_SIZE);
            return nullptr;
        }

        close(fd);

        return pages;
    }
};

#endif //BEAMFORMER_STREAMS_H