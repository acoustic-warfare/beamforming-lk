#include "rtmp.hpp"

#include <iostream>
#include <mutex>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <string>

#include "gst/gstelement.h"


#define RGB_BYTES 3
std::mutex RtmpStreamer::want_data_muxex = std::mutex();
std::mutex RtmpStreamer::handling_pipeline = std::mutex();


void RtmpStreamer::async_streamer_control_unit() {
    std::string command;
    std::getline(std::cin, command);

    do {
        if (std::strcmp(command.c_str(), "stop_rtmp_stream") == 0) {
            stop_rtmp_stream();
        } else if (std::strcmp(command.c_str(), "stop_local_stream") == 0) {
            stop_local_stream();
        } else if (std::strcmp(command.c_str(), "start_rtmp_stream") == 0) {
            start_rtmp_stream();
        } else if (std::strcmp(command.c_str(), "start_local_stream") == 0) {
            start_local_stream();
        } else if (std::strcmp(command.c_str(), "quit") == 0) {
            break;
        } else {
            gst_printerr("\nInvalid command.\n");
        }
    } while (std::getline(std::cin, command));
}


void RtmpStreamer::cb_need_data(GstAppSrc *appsrc, guint size,
                                gpointer user_data) {
    std::lock_guard<std::mutex> guard(want_data_muxex);
    bool *want_data = (bool *) user_data;
    *want_data = true;
}
void RtmpStreamer::cb_enough_data(GstAppSrc *appsrc, gpointer user_data) {
    std::lock_guard<std::mutex> guard(want_data_muxex);
    bool *want_data = (bool *) user_data;
    *want_data = false;
}

RtmpStreamer::RtmpStreamer()
    : screen_width(1024),
      screen_height(1024),
      want_data(true) {
    initialize_streamer();
}


RtmpStreamer::~RtmpStreamer() {
    g_print("set to null.\n");
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);
}


void RtmpStreamer::start_stream() {
    appsrc = gst_bin_get_by_name(GST_BIN(source_bin), "appsrc");
    if (!appsrc) {
        gst_printerr("error extracting appsrc\n");
        exit(1);
    }
    g_signal_connect(appsrc, "need-data", G_CALLBACK(cb_need_data), &want_data);
    g_signal_connect(appsrc, "enough-data", G_CALLBACK(cb_enough_data),
                     &want_data);
    gst_element_set_state(pipeline, GST_STATE_PLAYING);
    bus = gst_element_get_bus(pipeline);
}

void RtmpStreamer::stop_stream() {
    gst_element_set_state(pipeline, GST_STATE_PAUSED);
}

bool RtmpStreamer::send_frame(cv::Mat frame) {
    static guint64 count = 0;
    GstBuffer *buffer;
    guint size;
    GstFlowReturn ret;

    if (frame.empty()) {
        g_printerr("Captured frame is empty.\n");
        return FALSE;
    }

    std::lock_guard<std::mutex> guard(handling_pipeline);

    want_data_muxex.lock();
    if (!want_data) {
        //gst_printerr("appsrc does not require data right now.\n");
        want_data_muxex.unlock();
        return FALSE;
    }
    want_data_muxex.unlock();

    // Ensure the frame is in RGB format
    if (frame.channels() == 4) {
        cv::cvtColor(frame, frame, cv::COLOR_BGRA2RGB);
    } else if (frame.channels() == 3) {
        cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);
    } else {
        g_printerr("Captured frame is not in a supported format.\n");
        return FALSE;
    }

    // Get the size of the frame
    size = frame.total() * frame.elemSize();

    // Create a new buffer
    buffer = gst_buffer_new_allocate(nullptr, size, nullptr);

    GstClock *clock = gst_element_get_clock(appsrc);
    if (clock) {
        GstClockTime base_time = gst_element_get_base_time(appsrc);
        GstClockTime current_time = gst_clock_get_time(clock);
        GstClockTime running_time = current_time - base_time;
        GstClockTime timestamp = running_time;

        // Set the PTS (presentation timestamp) and DTS (decoding timestamp) of the buffer
        GST_BUFFER_PTS(buffer) = timestamp;
        GST_BUFFER_DTS(buffer) = timestamp;
        GST_BUFFER_DURATION(buffer) =
                (GstClockTime) gst_util_uint64_scale_int(GST_SECOND, 1, 30);
        gst_object_unref(clock);
    } else {
        gst_printerr("unable to open clock for appsrc!\n");
        exit(1);
    }

    if (!GST_BUFFER_DURATION_IS_VALID(buffer)) {
        gst_printerr("Invalid buffer duration.!\n");
        exit(1);
    }
    count += GST_SECOND / 30;

    // Copy the cv::Mat data into the GStreamer buffer
    GstMapInfo map;
    gst_buffer_map(buffer, &map, GST_MAP_WRITE);
    memcpy(map.data, frame.data, size);
    gst_buffer_unmap(buffer, &map);

    // Push the buffer to appsrc
    g_signal_emit_by_name(appsrc, "push-buffer", buffer, &ret);

    // Free the buffer
    gst_buffer_unref(buffer);

    if (ret != GST_FLOW_OK) {
        // We got some error, stop sending data
        g_print("error when sending :(.\n");
        return FALSE;
    }

    return TRUE;
}

void RtmpStreamer::initialize_streamer() {
    gst_init(nullptr, nullptr);

    pipeline = gst_pipeline_new("video pipeline");

    if (!pipeline) {
        gst_printerr("unable to create pipeline\n");
        exit(1);
    }

    source_bin = gst_parse_bin_from_description(
            "appsrc name=appsrc is-live=true block=false "
            "format=GST_FORMAT_TIME "
            "caps=video/x-raw,format=RGB,framerate=60/1,width=1024,height=1024 "
            "! videoconvert name=videoconvert ! videoscale name=videoscale ! "
            "videorate name=videorate ! "
            "video/x-raw,framerate=30/1,width=512,height=512 ! tee name=tee",
            false, nullptr);

    source_bin_name = gst_element_get_name(source_bin);

    rtmp_bin = gst_parse_bin_from_description(
            "x264enc name=x264_encoder tune=zerolatency speed-preset=superfast "
            "bitrate=2500 ! "
            "queue name=rtmp_queue ! flvmux name=flvmux streamable=true ! "
            "rtmpsink "
            "name=rtmp_sink"
            " location=rtmp://ome.waraps.org/app/beamforming",
            true, nullptr);
    rtmp_bin_name = gst_element_get_name(rtmp_bin);
    local_video_bin = gst_parse_bin_from_description(
            "queue name=local_video_queue ! autovideosink "
            "name=local_video_sink",
            true, nullptr);
    local_video_bin_name = gst_element_get_name(local_video_bin);

    if (!source_bin || !rtmp_bin || !local_video_bin) {
        gst_printerrln("Error setting up bins.");
        exit(1);
    }

    gst_bin_add_many(GST_BIN(pipeline), source_bin, rtmp_bin, local_video_bin,
                     NULL);

    GstElement *tee = gst_bin_get_by_name(GST_BIN(source_bin), "tee");
    if (!tee) {
        gst_printerrln("Unable to retreive tee element.");
        exit(1);
    }


    src_rtmp_tee_pad = gst_element_request_pad_simple(tee, "src_%u");
    src_local_tee_pad = gst_element_request_pad_simple(tee, "src_%u");

    gst_element_add_pad(source_bin,
                        gst_ghost_pad_new("tee_rtmp_src", src_rtmp_tee_pad));
    gst_element_add_pad(source_bin, gst_ghost_pad_new("local_video_src",
                                                      src_local_tee_pad));


    GstPad *source_bin_rtmp =
            gst_element_get_static_pad(source_bin, "tee_rtmp_src");
    GstPad *rtmp_sink_pad = gst_element_get_static_pad(rtmp_bin, "sink");
    GstPadLinkReturn rtmp_link_result =
            gst_pad_link(source_bin_rtmp, rtmp_sink_pad);
    if (rtmp_link_result != GST_PAD_LINK_OK) {
        gst_printerrln(
                "Unable to link request-pads to video and rtmp bins. rtmp "
                "result: %d.",
                rtmp_link_result);
        exit(1);
    }

    GstPad *source_bin_video =
            gst_element_get_static_pad(source_bin, "local_video_src");
    GstPad *local_video_sink_pad =
            gst_element_get_static_pad(local_video_bin, "sink");
    GstPadLinkReturn video_link_result =
            gst_pad_link(source_bin_video, local_video_sink_pad);

    if (video_link_result != GST_PAD_LINK_OK) {
        gst_printerrln(
                "Unable to link request-pads to video and rtmp bins.video "
                "result: %d.",
                video_link_result);
        exit(1);
    }

    gst_object_unref(source_bin_rtmp);
    gst_object_unref(source_bin_video);
    gst_object_unref(rtmp_sink_pad);
    gst_object_unref(local_video_sink_pad);
}


static void set_element_state_to_parent_state(GstElement *element) {
    GstElement *parent;
    GstState parent_state, parent_pending;
    GstStateChangeReturn ret;

    if (!element) {
        g_printerr("Element is NULL\n");
        return;
    }

    // Get the parent element
    parent = GST_ELEMENT(gst_element_get_parent(element));
    if (!parent) {
        g_printerr("Element has no parent\n");
        return;
    }

    // Get the current state and pending state of the parent
    ret = gst_element_get_state(parent, &parent_state, &parent_pending,
                                GST_CLOCK_TIME_NONE);
    if (ret != GST_STATE_CHANGE_SUCCESS) {
        g_printerr("Failed to get parent state\n");
        gst_object_unref(parent);
        return;
    }

    // Set the state of the element to the parent state
    ret = gst_element_set_state(element, parent_state);
    if (ret != GST_STATE_CHANGE_SUCCESS) {
        g_printerr("Failed to set element state\n");
    } else {
        g_print("Element state set to match parent state: %d\n", parent_state);
    }

    // Clean up
    gst_object_unref(parent);
}


void RtmpStreamer::start_rtmp_stream() {
    GstElement *bin = gst_bin_get_by_name(GST_BIN(pipeline), rtmp_bin_name);
    if (bin) {
        gst_print("rtmp bin already connected\n");
        g_object_unref(bin);
        return;
    }
    gst_print("hello");
    if (!connect_sink_bin_to_source_bin(source_bin, rtmp_bin, &src_rtmp_tee_pad, "tee",
                                        "tee_rtmp_src")) {

        exit(1);
    }
}

void RtmpStreamer::stop_rtmp_stream() {
    GstElement *bin = gst_bin_get_by_name(GST_BIN(pipeline), rtmp_bin_name);
    if (!bin) {
        gst_print("rtmp bin already disconnected\n");
        return;
    }
    g_object_unref(bin);
    if (!disconnect_sink_bin_from_source_bin(
                source_bin, rtmp_bin, src_rtmp_tee_pad, "tee_rtmp_src")) {
        exit(1);
    }
    src_rtmp_tee_pad = nullptr;
}


void RtmpStreamer::start_local_stream() {
    GstElement *bin =
            gst_bin_get_by_name(GST_BIN(pipeline), local_video_bin_name);
    if (bin) {
        gst_print("local bin already connected\n");
        g_object_unref(bin);
        return;
    }
    if (!connect_sink_bin_to_source_bin(source_bin, local_video_bin, &src_local_tee_pad, "tee",
                                        "local_video_src")) {
        exit(1);
    }
}

void RtmpStreamer::stop_local_stream() {
    GstElement *bin =
            gst_bin_get_by_name(GST_BIN(pipeline), local_video_bin_name);
    if (!bin) {
        gst_print("local bin already disconnected\n");
        return;
    }
    g_object_unref(bin);
    if (!disconnect_sink_bin_from_source_bin(source_bin, local_video_bin,
                                             src_local_tee_pad,
                                             "local_video_src")) {
        exit(1);
    }
    src_local_tee_pad = nullptr;
}

bool RtmpStreamer::disconnect_sink_bin_from_source_bin(
        GstElement *source_bin, GstElement *sink_bin, GstPad *tee_pad,
        const char *tee_ghost_pad_src_name) {
    GstPad *ghost_pad =
            gst_element_get_static_pad(source_bin, tee_ghost_pad_src_name);
    GstElement *tee = gst_bin_get_by_name(GST_BIN(source_bin), "tee");
    if (!source_bin) {
        g_printerr("Invalid bin\n");
        return false;
    }
    if (!tee) {
        g_printerr("Invalid tee element\n");
        return false;
    }
    if (!tee_pad) {
        g_printerr("Invalid tee pad\n");
        return false;
    }
    if (!ghost_pad) {
        g_printerr("Invalid source ghost pad\n");
        return false;
    }

    if (!source_bin || !tee || !tee_pad || !ghost_pad) {
        g_printerr("Invalid bin, tee, tee pad, or ghost pad\n");
        return false;
    }
    std::lock_guard<std::mutex> guard(handling_pipeline);

    // Set elements to NULL state
    //gst_element_set_state(pipeline, GST_STATE_NULL);

    // Lock the state of the elements before modifying the pipeline
    if (!gst_element_set_locked_state(GST_ELEMENT(pipeline), TRUE)) {
        gst_printerr("unable to lock pipeline state\n");
        return false;
    }

    // Unlink the ghost pad from its peer
    GstPad *peer_pad = gst_pad_get_peer(ghost_pad);
    if (peer_pad) {
        gst_pad_unlink(ghost_pad, peer_pad);
        gst_object_unref(peer_pad);
    }

    // Remove the ghost pad from the bin
    gst_element_remove_pad(GST_ELEMENT(source_bin), ghost_pad);
    gst_object_unref(ghost_pad);

    // Unlink the tee pad from its peer
    peer_pad = gst_pad_get_peer(tee_pad);
    if (peer_pad) {
        gst_pad_unlink(tee_pad, peer_pad);
        gst_object_unref(peer_pad);
    }

    // Release the request pad from the tee element
    gst_element_release_request_pad(tee, tee_pad);

    gst_object_unref(tee_pad);

    gchar *name = gst_element_get_name(sink_bin);
    sink_bin = gst_bin_get_by_name(GST_BIN(pipeline), name);
    g_free(name);


    gst_bin_remove(GST_BIN(pipeline), sink_bin);

    // Unlock the mutex after making changes
    if (!gst_element_set_locked_state(GST_ELEMENT(pipeline), FALSE)) {
        gst_printerr("unable to unlock locked pipeline state\n");
        return false;
    }

    // Set everyting but the disconnected Bin to PLAYING state
    gst_element_set_state(pipeline, GST_STATE_PLAYING);

    // Clean up
    gst_object_unref(tee);

    return true;
}

bool RtmpStreamer::connect_sink_bin_to_source_bin(
        GstElement *source_bin, GstElement *sink_bin, GstPad **request_pad,
        const char *tee_element_name, const char *tee_ghost_pad_name) {

    if (!source_bin || !sink_bin) {
        gst_printerr("Invalid source- or sink-bin, \n");
        return false;
    }

    std::lock_guard<std::mutex> guard(handling_pipeline);

    if (!gst_element_set_locked_state(GST_ELEMENT(pipeline), TRUE)) {
        gst_printerr("unable to lock pipeline state\n");
        return false;
    }

    if (!gst_bin_add(GST_BIN(pipeline), sink_bin)) {
        gst_printerr("error, unable to add sink bin to pipeline!!\n");
        exit(1);
    }

    // Request Tee pad
    GstElement *tee =
            gst_bin_get_by_name(GST_BIN(source_bin), tee_element_name);
    if (!tee) {
        gst_printerr("No tee element by that name \n");
        if (!gst_element_set_locked_state(GST_ELEMENT(pipeline), FALSE)) {
            gst_printerr("unable to unlock locked pipeline state\n");
            return false;
        }
        return false;
    }

    // Setup ghost pad for source bin
    *request_pad = gst_element_request_pad_simple(tee, "src_%u");
    gst_element_add_pad(source_bin, gst_ghost_pad_new(tee_ghost_pad_name,
                                                      *request_pad));

    // Link ghost pad of source_bin to sink_bin
    GstPad *src_ghost_pad =
            gst_element_get_static_pad(source_bin, tee_ghost_pad_name);
    GstPad *sink_ghost_pad = gst_element_get_static_pad(sink_bin, "sink");
    GstPadLinkReturn link_ok = gst_pad_link(src_ghost_pad, sink_ghost_pad);
    if (link_ok != GST_PAD_LINK_OK) {
        gst_printerr(
                "error linking source_bin ghost pad to sink_bin ghost pad. "
                "Error code: %d\n",
                link_ok);
        if (!gst_element_set_locked_state(GST_ELEMENT(pipeline), FALSE)) {
            gst_printerr("unable to unlock locked pipeline state\n");
            return false;
        }
        return false;
    }

    // Unlock the state mutex after making changes
    if (!gst_element_set_locked_state(GST_ELEMENT(pipeline), FALSE)) {
        gst_printerr("unable to unlock locked pipeline state\n");
        return false;
    }

    gst_element_set_state(sink_bin, GST_STATE_PLAYING);


    // Unref objects
    gst_object_unref(src_ghost_pad);
    gst_object_unref(sink_ghost_pad);
    gst_object_unref(tee);

    return true;
}

void RtmpStreamer::debug_info() {
    const char *possible_states[5] = {"Void Pending", "Null", "Ready", "Paused",
                                      "Playing"};
    gst_println(
            "\n----------------- START DEBUG INFO "
            "-----------------------\n");
    gst_println("pipeline state: %s (pending state: %s)\n",
                possible_states[pipeline->current_state],
                possible_states[pipeline->pending_state]);
    gchar *name;
    GST_OBJECT_LOCK(pipeline);
    GList *pipeline_children = GST_BIN(pipeline)->children;

    while (pipeline_children) {
        GstBin *bin = GST_BIN(pipeline_children->data);
        gst_println("###### BIN: %s ######", gst_pad_get_name(bin));
        gst_println("bin state: %s (pending state: %s)\n",
                    possible_states[GST_ELEMENT(pipeline_children->data)
                                            ->current_state],
                    possible_states[GST_ELEMENT(pipeline_children->data)
                                            ->pending_state]);

        GList *pads = GST_ELEMENT(pipeline_children->data)->pads;
        gst_println("--- Bin Pads ---");
        while (pads) {
            GstPad *pad = GST_PAD(pads->data);
            name = gst_pad_get_name(pad);
            gst_println("bin pad: %s (is linked: %s)", name,
                        gst_pad_is_linked(pad) ? "true" : "false");
            g_free(name);
            pads = pads->next;
        }

        gst_println("\n--- Elements ---");
        GList *elements = bin->children;
        while (elements) {
            GstElement *element = GST_ELEMENT(elements->data);
            name = gst_element_get_name(element);
            gst_println("element: %s", name);
            g_free(name);
            gst_println("- element state: %s (pending state: %s)",
                        possible_states[element->current_state],
                        possible_states[element->pending_state]);

            GList *pads = element->pads;
            gst_println("element pads:");
            while (pads) {
                GstPad *pad = GST_PAD(pads->data);
                name = gst_pad_get_name(pad);
                gst_println("- element pad: %s (is linked: %s)", name,
                            gst_pad_is_linked(pad) ? "true" : "false");
                g_free(name);

                pads = pads->next;
            }
            elements = elements->next;
            gst_print("\n");
        }
        pipeline_children = pipeline_children->next;
        gst_print("\n");
    }
    GST_OBJECT_UNLOCK(pipeline);
    gst_println("----------------- END DEBUG INFO -----------------------\n");
}

 bool RtmpStreamer::check_error() const {
        GstMessage *msg = gst_bus_timed_pop_filtered(bus, GST_CLOCK_TIME_NONE,
                                                     GST_MESSAGE_ERROR);
        if (msg != nullptr) {
            GError *err;
            gchar *debug_info;

            switch (GST_MESSAGE_TYPE(msg)) {
                case GST_MESSAGE_ERROR:
                    gst_message_parse_error(msg, &err, &debug_info);
                    g_printerr("Error received from element %s: %s\n",
                               GST_OBJECT_NAME(msg->src), err->message);
                    g_printerr("Debugging information: %s\n",
                               debug_info ? debug_info : "none");
                    g_clear_error(&err);
                    g_free(debug_info);
                    return true;
                case GST_MESSAGE_EOS:
                    std::cout << "end" << std::endl;
                    g_print("End-Of-Stream reached.\n");
                    return true;
                default:
                    g_print("Error: Unknown message type");
                    exit(1);
                    // We should not reach here because we only asked for ERRORs and EOS
                    break;
            }
            gst_message_unref(msg);
        }

        return false;
    }
