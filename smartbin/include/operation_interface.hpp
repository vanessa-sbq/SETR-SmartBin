#pragma once
#include <string>

// Operation Interface
// Receives operator commands from an esp8266 WiFi remote over TCP and translates button presses into control events for the tasks.
// esp8266 connects as a TCP client and sends newline terminated lines of comma separated button names.
// Every call is nonblocking so the owning periodic task keeps its period.
class OperationInterface {
  public:
    /*
        toggleActive is triggered when the START button is pressed. This value will be used to start/stop the motors.
        quit is triggered upon pressing the B button. This will make all the tasks and the program stop.
    */
    struct Events {
        bool toggleActive = false;
        bool quit = false;
    };

    explicit OperationInterface(int port);
    ~OperationInterface();

    OperationInterface(const OperationInterface &) = delete;
    OperationInterface &operator=(const OperationInterface &) = delete;

    bool start();
    void stop();

    Events poll();

    bool listening() const { return listen_sock_fd >= 0; }
    bool clientConnected() const { return client_sock_fd >= 0; }

  private:
    void acceptClient();
    void drainClient(Events &ev);
    void handleLine(const std::string &line, Events &ev);
    void closeClient();

    int operation_interface_port;
    int listen_sock_fd = -1;
    int client_sock_fd = -1;
    std::string line_buffer;
};
