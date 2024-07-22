// https://github.com/eruffaldi

#include "Mp3encoder.h"
#include <iostream>

Mp3encoder::Mp3encoder(int samplerate, int channels, int bitrate, const std::string &name)
    : onf(name.c_str(), std::ios::binary) {
    buf.resize(1024 * 64);
    p = lame_init();
    if (!p) {
        std::cerr << "Failed to initialize LAME encoder" << std::endl;
        exit(EXIT_FAILURE);
    }
    lame_set_in_samplerate(p, samplerate);
    lame_set_num_channels(p, channels);
    lame_set_brate(p, bitrate);
    lame_init_params(p);
}

void Mp3encoder::encode_inter(const short *s, int samples) {
    int n = lame_encode_buffer_interleaved(p, (short int*)s, samples, (unsigned char*)&buf[0], buf.size());
    if (n > 0) {
        onf.write(&buf[0], n);
    }
}

void Mp3encoder::flush() {
    int n = lame_encode_flush(p, (unsigned char*)&buf[0], buf.size());
    if (n > 0) {
        onf.write(&buf[0], n);
    }
}

Mp3encoder::~Mp3encoder() {
    if (p) {
        lame_close(p);
    }
}
