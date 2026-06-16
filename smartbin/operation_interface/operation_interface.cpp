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

namespace {

constexpr int kBacklog     = 4;
constexpr int kRecvBufSize = 1024;

bool setReuseAddr(int fd) {
    int yes = 1;
    return ::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes)) == 0;
}

bool setNonBlocking(int fd) {
    int flags = ::fcntl(fd, F_GETFL, 0);
    if (flags < 0) return false;
    return ::fcntl(fd, F_SETFL, flags | O_NONBLOCK) == 0;
}

std::string trim(const std::string& value) {
    const char* kWhitespace = " \t\r\n";
    const std::string::size_type first = value.find_first_not_of(kWhitespace);
    if (first == std::string::npos) return "";
    const std::string::size_type last = value.find_last_not_of(kWhitespace);
    return value.substr(first, last - first + 1);
}

// Split a line on commas and strip optional surrounding double quotes,
// matching the ESP8266 wire format in wifi_raspberry/wifi.cpp.
std::vector<std::string> parseButtons(const std::string& line) {
    std::vector<std::string> buttons;
    std::string token;
    auto flush = [&] {
        std::string t = trim(token);
        if (t.size() >= 2 && t.front() == '"' && t.back() == '"')
            t = t.substr(1, t.size() - 2);
        if (!t.empty()) buttons.push_back(t);
        token.clear();
    };
    for (char ch : line) {
        if (ch == ',') { flush(); continue; }
        token.push_back(ch);
    }
    flush();
    return buttons;
}

bool equalsIgnoreCase(const std::string& a, const char* b) {
    std::size_t i = 0;
    for (; i < a.size() && b[i] != '\0'; ++i) {
        if (std::tolower(static_cast<unsigned char>(a[i])) !=
            std::tolower(static_cast<unsigned char>(b[i])))
            return false;
    }
    return i == a.size() && b[i] == '\0';
}

}  // namespace

OperationInterface::OperationInterface(int port) : port_(port) {}

OperationInterface::~OperationInterface() { stop(); }

bool OperationInterface::start() {
    listen_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd_ < 0) {
        std::perror("[OpIface] socket");
        return false;
    }
    if (!setReuseAddr(listen_fd_))
        std::perror("[OpIface] setsockopt");

    sockaddr_in addr = {};
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port        = htons(static_cast<uint16_t>(port_));

    if (::bind(listen_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        std::perror("[OpIface] bind");
        closeClient();
        ::close(listen_fd_);
        listen_fd_ = -1;
        return false;
    }
    if (::listen(listen_fd_, kBacklog) < 0) {
        std::perror("[OpIface] listen");
        ::close(listen_fd_);
        listen_fd_ = -1;
        return false;
    }
    if (!setNonBlocking(listen_fd_))
        std::perror("[OpIface] fcntl(listen)");

    std::cout << "[OpIface] WiFi remote listening on TCP port " << port_ << "\n";
    return true;
}

OperationInterface::Events OperationInterface::poll() {
    Events ev;
    if (listen_fd_ < 0) return ev;

    if (client_fd_ < 0)
        acceptClient();
    if (client_fd_ >= 0)
        drainClient(ev);
    return ev;
}

void OperationInterface::acceptClient() {
    sockaddr_in client_addr = {};
    socklen_t   client_len  = sizeof(client_addr);
    int fd = ::accept(listen_fd_, reinterpret_cast<sockaddr*>(&client_addr), &client_len);
    if (fd < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)
            std::perror("[OpIface] accept");
        return;  // no client waiting
    }
    if (!setNonBlocking(fd))
        std::perror("[OpIface] fcntl(client)");

    char ip[INET_ADDRSTRLEN] = {0};
    if (::inet_ntop(AF_INET, &client_addr.sin_addr, ip, sizeof(ip)) != nullptr)
        std::cout << "[OpIface] remote connected: " << ip << ":"
                  << ntohs(client_addr.sin_port) << "\n";
    else
        std::cout << "[OpIface] remote connected: (unknown)\n";

    client_fd_ = fd;
    line_buffer_.clear();
}

void OperationInterface::drainClient(Events& ev) {
    char buf[kRecvBufSize];
    for (;;) {
        const ssize_t n = ::recv(client_fd_, buf, sizeof(buf), 0);
        if (n == 0) {  // peer closed
            std::cout << "[OpIface] remote disconnected\n";
            closeClient();
            return;
        }
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) break;  // drained
            if (errno == EINTR) continue;
            std::perror("[OpIface] recv");
            closeClient();
            return;
        }

        line_buffer_.append(buf, static_cast<std::size_t>(n));

        std::string::size_type pos;
        while ((pos = line_buffer_.find('\n')) != std::string::npos) {
            std::string line = line_buffer_.substr(0, pos);
            if (!line.empty() && line.back() == '\r')
                line.pop_back();
            handleLine(line, ev);
            line_buffer_.erase(0, pos + 1);
        }
    }
}

void OperationInterface::handleLine(const std::string& line, Events& ev) {
    for (const std::string& button : parseButtons(line)) {
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

void OperationInterface::closeClient() {
    if (client_fd_ >= 0) {
        ::close(client_fd_);
        client_fd_ = -1;
    }
    line_buffer_.clear();
}

void OperationInterface::stop() {
    closeClient();
    if (listen_fd_ >= 0) {
        ::close(listen_fd_);
        listen_fd_ = -1;
    }
}
