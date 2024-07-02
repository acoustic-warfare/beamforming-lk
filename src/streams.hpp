#ifndef STREAMS_H
#define STREAMS_H


#include "config.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <sys/syscall.h>
#include <linux/memfd.h>
#include <fcntl.h>

#include <unordered_map>
#include <iostream>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdarg.h>
#include <fcntl.h>
#include <errno.h>

#include <iostream>

#include <linux/memfd.h>
#include <sys/syscall.h>
#include <sys/mman.h>
#include <sys/types.h>



#include <assert.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>

#define PAGE_SIZE getpagesize()

#define N_ITEMS_BUFFER PAGE_SIZE / sizeof(float)

#define BUFFER_BYTES N_SAMPLES * sizeof(float)

class Streams {
public:
    std::unordered_map<unsigned, float*> buffers;
    unsigned position;
    
    
    Streams() : position(0) {

    };

    bool create_stream(unsigned index) {
        if (this->buffers.find(index) != this->buffers.end())
        {
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

    float* get_signal(unsigned index, int offset) {
        return (float *)((char *)this->buffers[index] + this->position + offset * sizeof(float));
    }

    inline void write_stream(unsigned index, float *data) {
        memcpy((char*)this->buffers[index] + this->position, data, BUFFER_BYTES);
    }

    inline void read_stream(unsigned index, float *data, unsigned offset = 0) {
        memcpy(data, (char*)this->buffers[index] + this->position + offset * sizeof(float), BUFFER_BYTES);
    }

#if 1 // Only debugging
    void dump_stream(unsigned index, float *data) {
        memcpy(data, (char*)this->buffers[index], PAGE_SIZE);
    }
#endif

    ~Streams() {
        for (auto itr = this->buffers.begin(); itr != this->buffers.end(); ++itr) {
            // Free memory
            munmap(itr->second, PAGE_SIZE * 2);
        }
    }

    /**
    Reserved only for producer
    
     */
    void forward() {
        this->position = (this->position + BUFFER_BYTES) % PAGE_SIZE;
    }



    
private:
    
    
    float* allocate_buffer() {
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
        float *pages = (float*)mmap(0, 2 * PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd,  0);
        if (pages == MAP_FAILED) {
            printf("Unable to create first region\n");
            close(fd);
            return nullptr;
        }

        if (mmap((char*)pages + PAGE_SIZE, PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_FIXED, fd, 0) == MAP_FAILED) {
            printf("Unable to create reg1\n");
            close(fd);
            munmap(pages, PAGE_SIZE);
            return nullptr;
        }

        close(fd);

        return pages;
    }
    

    
};





#endif