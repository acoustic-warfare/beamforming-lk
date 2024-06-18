#ifndef MQTT_CLIENT_H
#define MQTT_CLIENT_H

#include <mqtt/async_client.h>
#include <nlohmann/json.hpp>

#define DEFAULT_HEARTBEAT_INTERVAL 1000

class mqtt_client
{
private:
    const std::chrono::milliseconds heartbeat_interval =
        std::chrono::milliseconds(DEFAULT_HEARTBEAT_INTERVAL);
    const std::string UNIT_NAME, SERVER_ADDRESS;

    std::string static generate_uuid();
    std::string generate_full_topic(std::string topic) const;
    std::string generate_heartbeat_message() const;
    void pong(nlohmann::json msg_payload);

    std::string uuid = generate_uuid();
    std::thread heartbeat_thread;
    mqtt::async_client client;
    std::shared_ptr<bool> is_running = std::make_shared<bool>(false);

public:
    mqtt_client(std::string name, std::string server_address);
    mqtt_client(mqtt_client &&);
    mqtt_client(const mqtt_client &);
    mqtt_client &operator=(const mqtt_client &);
    mqtt_client &operator=(mqtt_client &&);
    ~mqtt_client();

    bool running() const;
    void start();
    void stop();
    bool handle_message(mqtt::const_message_ptr msg);
    bool publish_message_async(std::string topic, std::string payload);
};

#endif