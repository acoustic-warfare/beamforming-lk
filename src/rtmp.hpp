#include <gst/app/gstappsrc.h>
#include <gst/gst.h>

#include <mutex>
#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>

class RtmpStreamer {
public:
    GstBus *bus;

    RtmpStreamer();
    RtmpStreamer(uint width, uint height);
    RtmpStreamer(const RtmpStreamer &) = delete;
    RtmpStreamer &operator=(const RtmpStreamer &) = delete;
    ~RtmpStreamer();

    bool send_frame(cv::Mat frame);

    void start_stream();
    void stop_stream();

    void start_rtmp_stream();
    void stop_rtmp_stream();
    void start_local_stream();
    void stop_local_stream();

    [[nodiscard]] bool check_error() const;
    void async_streamer_control_unit();
    void debug_info();

private:
    static void cb_need_data(GstAppSrc *appsrc, guint size, gpointer user_data);
    static void cb_enough_data(GstAppSrc *appsrc, gpointer user_data);

    bool connect_sink_bin_to_source_bin(GstElement *source_bin,
                                        GstElement *sink_bin, GstPad **request_pad,
                                        const char *tee_element_name,
                                        const char *tee_ghost_pad_name);
    bool disconnect_sink_bin_from_source_bin(
            GstElement *source_bin, GstElement *sink_bin, GstPad *tee_pad,
            const char *tee_ghost_pad_src_name);


    gboolean check_links();
    void initialize_streamer();

    GstElement *pipeline, *source_bin, *rtmp_bin, *local_video_bin;
    gchar *source_bin_name, *rtmp_bin_name, *local_video_bin_name;
    GstElement *appsrc;
    GstPad *src_rtmp_tee_pad, *src_local_tee_pad;


    bool want_data;
    uint screen_width, screen_height;
    static std::mutex want_data_muxex;
    static std::mutex handling_pipeline;
};
