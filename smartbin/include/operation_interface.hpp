#pragma once
#include <string>

// Operation Interface
// receives operator commands from an ESP8266 WiFi remote over TCP and translates 
// button presses into control events for the tasks. 
// ESP8266 connects as a TCP client and sends newline-terminated lines of
// comma-separated, optionally double-quoted button names, e.g. "START","QUIT"
// Every call is non-blocking so the owning periodic task keeps its period.
class OperationInterface {
public:
    // Control events seen since the previous poll().
    struct Events {
        bool toggleActive = false; // START pressed: flip the start/stop flag
        bool quit         = false; // QUIT pressed: request shutdown
    };

    explicit OperationInterface(int port);
    ~OperationInterface();

    OperationInterface(const OperationInterface&)            = delete;
    OperationInterface& operator=(const OperationInterface&) = delete;

    // Open the (non-blocking) listening socket. Returns false on failure with
    // errno logged; the rest of the system still runs, just without the remote.
    bool start();

    // Accept a pending client, drain readable bytes and parse any complete
    // button lines. Never blocks; returns the events that occurred this call.
    Events poll();

    // Close the client and listening sockets.
    void stop();

    bool listening() const { return listen_fd_ >= 0; }
    bool clientConnected() const { return client_fd_ >= 0; }

private:
    void acceptClient();      // try to accept a waiting ESP8266 connection
    void drainClient(Events& ev);
    void handleLine(const std::string& line, Events& ev);
    void closeClient();

    int         port_;
    int         listen_fd_ = -1;
    int         client_fd_ = -1;
    std::string line_buffer_;
};
