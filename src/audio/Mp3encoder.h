#include <lame/lame.h>
#include <portaudio.h>

#include <atomic>
#include <fstream>
#include <string>
#include <vector>

class Mp3encoder {
public:
    Mp3encoder(int samplerate, int channels, int bitrate, const std::string &name);
    ~Mp3encoder();

    void encode_inter(const short *s, int samples);
    void flush();

private:
    std::vector<char> buf;
    std::ofstream onf;
    lame_global_flags *p;
};
