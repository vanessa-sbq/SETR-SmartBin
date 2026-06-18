#include "operation_interface.hpp"
#include "config.hpp"

#include <arpa/inet.h>
#include <cctype>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <vector>

#include <sys/socket.h>
#include <unistd.h>

constexpr int kBacklog = 4;
constexpr int kRecvBufSize = 1024;

bool setReuseAddr(int fd) {
    int yes = 1;
    return ::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes)) == 0;
}

bool setNonBlocking(int fd) {
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags < 0)
        return false;
    return fcntl(fd, F_SETFL, flags | O_NONBLOCK) == 0;
}

/*
    Helper function
    Given the existing white space characters we try to create a substring by
    looking at the points where a whitespace is not found.
    Example: " SELECT" (value)
    Return: "SELECT"

*/
std::string trim(const std::string &value) {
    const char *kWhitespace = " \t\r\n";
    const std::string::size_type first = value.find_first_not_of(kWhitespace);
    if (first == std::string::npos)
        return "";
    const std::string::size_type last = value.find_last_not_of(kWhitespace);
    return value.substr(first, last - first + 1);
}

/*
    Helper function.
    Split a line on commas and strip optional surrounding double quotes.
    It calls the trim function which helps determine the actual button string.
    We iterate through every single character of a line. Since the buttons are
    separated by commas (e.g. "A, B, SELECT") each time we see one we call flush
    (lambda function) so that the button can be obtained and added to the button
    vector. It returns the buttons that are currently being pressed.

    Example: "A, B, SELECT"
    Return: vec<string>{"A", "B", "SELECT"}
*/
std::vector<std::string> parseButtons(const std::string &line) {
    std::vector<std::string> buttons;
    std::string token;
    auto flush = [&] {
        std::string t = trim(token);
        if (t.size() >= 2 && t.front() == '"' && t.back() == '"')
            t = t.substr(1, t.size() - 2);
        if (!t.empty())
            buttons.push_back(t);
        token.clear();
    };
    for (char ch : line) {
        if (ch == ',') {
            flush();
            continue;
        }
        token.push_back(ch);
    }
    flush();
    return buttons;
}

/*
    Helper function. Compares two strings. The second argument is a char*
    because it is being loaded from the config where it was defined (inside the
    Config namespace).
*/
bool equalsIgnoreCase(const std::string &a, const char *b) {
    std::size_t i = 0;
    for (; i < a.size() && b[i] != '\0'; ++i) {
        if (std::tolower(static_cast<unsigned char>(a[i])) != std::tolower(static_cast<unsigned char>(b[i])))
            return false;
    }
    return i == a.size() && b[i] == '\0';
}

/*
    Default constructor. The socket will listen in the port specified.
*/
OperationInterface::OperationInterface(int port) : operation_interface_port(port) {}

OperationInterface::~OperationInterface() { stop(); }

/*
    This function opens the socket to listen to the commands coming from the
    esp8266. This function also sets the address as reusable so that if the
    connection closes, upon restart it doesn't have to wait. Furthermore the
    socket will also be nonblocking which is important because preemption can
    happen. If an error happens it is logged by perror.

    @return true - If the socket is listening.
    @return false - In the case of an error.
*/
bool OperationInterface::start() {
    listen_sock_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (listen_sock_fd < 0) {
        std::perror("[OpIface] socket");
        return false;
    }
    if (!setReuseAddr(listen_sock_fd))
        std::perror("[OpIface] setsockopt");

    sockaddr_in addr = {};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(static_cast<uint16_t>(operation_interface_port));

    if (bind(listen_sock_fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
        std::perror("[OpIface] bind");
        closeClient();
        close(listen_sock_fd);
        listen_sock_fd = -1;
        return false;
    }
    if (listen(listen_sock_fd, kBacklog) < 0) {
        std::perror("[OpIface] listen");
        close(listen_sock_fd);
        listen_sock_fd = -1;
        return false;
    }
    if (!setNonBlocking(listen_sock_fd))
        std::perror("[OpIface] fcntl(listen)");

    std::cout << "[OpIface] WiFi remote listening on TCP port " << operation_interface_port << "\n";
    return true;
}

/*
    Helper function.
    If the client's fd is still less than 0 then we try to accept a client.
    If we have a client's fd then we call drainClient.
    This function always returns the events.

    These may end up not be changed.
*/
OperationInterface::Events OperationInterface::poll() {
    Events ev;
    if (listen_sock_fd < 0)
        return ev;

    if (client_sock_fd < 0)
        acceptClient();
    if (client_sock_fd >= 0)
        drainClient(ev);
    return ev;
}

/*
    Accepts the client and stores the socket fd on client_sock_fd.
*/
void OperationInterface::acceptClient() {
    sockaddr_in client_addr = {};
    socklen_t client_len = sizeof(client_addr);
    int fd = accept(listen_sock_fd, reinterpret_cast<sockaddr *>(&client_addr), &client_len);
    if (fd < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)
            std::perror("[OpIface] accept");
        return; // no client waiting
    }
    if (!setNonBlocking(fd))
        std::perror("[OpIface] fcntl(client)");

    char ip[INET_ADDRSTRLEN] = {0};
    if (inet_ntop(AF_INET, &client_addr.sin_addr, ip, sizeof(ip)) != nullptr)
        std::cout << "[OpIface] remote connected: " << ip << ":" << ntohs(client_addr.sin_port) << "\n";
    else
        std::cout << "[OpIface] remote connected: (unknown)\n";

    client_sock_fd = fd;
    line_buffer.clear();
}

/*
    This function helps receive the content that the client sends.
    How it works:

    We temporarily receive the data using recv and hold the data inside buf.
    The content that was just received then gets added to the line_buffer
    string. Then we try to search for a newline character. If we find it then it
    means that we have successfully received one line of events from the esp8266.
    This line will be processed by the handleLine function.
*/
void OperationInterface::drainClient(Events &ev) {
    char buf[kRecvBufSize];
    while (true) {
        const ssize_t n = recv(client_sock_fd, buf, sizeof(buf), 0);
        if (n == 0) { // peer closed
            std::cout << "[OpIface] remote disconnected\n";
            closeClient(); // TODO: FIXME: should we close everytime ?
            return;
        }
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK)
                break; // drained
            if (errno == EINTR)
                continue;
            std::perror("[OpIface] recv");
            closeClient();
            return;
        }

        line_buffer.append(buf, static_cast<std::size_t>(n));

        std::string::size_type pos;
        while ((pos = line_buffer.find('\n')) != std::string::npos) {
            std::string line = line_buffer.substr(0, pos);
            if (!line.empty() && line.back() == '\r')
                line.pop_back();
            handleLine(line, ev);
            line_buffer.erase(0, pos + 1);
        }
    }
}

/*
    Helper function.
    Receives both a line and event struct as a reference (to be modified).
    It uses two functions. parseButtons and equalsIgnoreCase
*/
void OperationInterface::handleLine(const std::string &line, Events &ev) {
    for (const std::string &button : parseButtons(line)) {
        if (equalsIgnoreCase(button, Config::OPIF_BTN_TOGGLE)) {
            ev.toggleActive = true;
            std::cout << "[OpIface] button: " << button << " (start/stop)\n";
        } else if (equalsIgnoreCase(button, Config::OPIF_BTN_QUIT)) {
            ev.quit = true;
            std::cout << "[OpIface] button: " << button << " (quit)\n";
        } else {
            std::cout << "[OpIface] button: " << button << " (ignored)\n";
        }
    }
}

/*
    Helper function. Closes the client's socket but also clears the line_buffer
    string so old buttons do not get parsed if the client reconnects.
*/
void OperationInterface::closeClient() {
    if (client_sock_fd >= 0) {
        close(client_sock_fd);
        client_sock_fd = -1;
    }
    line_buffer.clear();
}

/*
    Helper function. Closes the client's socket first and then closes ours.
*/
void OperationInterface::stop() {
    closeClient();
    if (listen_sock_fd >= 0) {
        close(listen_sock_fd);
        listen_sock_fd = -1;
    }
}
