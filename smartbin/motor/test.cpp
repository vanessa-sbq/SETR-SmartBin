/* 
#include <PiPCA9685/PCA9685.h>
#include "motor_controller.h"
#include "MotorPins.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <arpa/inet.h>
#include <cerrno>
#include <csignal>
#include <cstring>
#include <string>
#include <vector>
#include <sys/socket.h>
#include <unistd.h>
#include <math.h>
#include <limits.h>
#include <cmath>
#include <stdexcept>

namespace {
	constexpr int kListenPort = 6767;
	constexpr int kBacklog = 4;
	constexpr int kRecvBufSize = 1024;

	volatile std::sig_atomic_t g_stop = 0;

	void HandleSignal(int) {
		g_stop = 1;
	}

	enum class ServerState {
		kInit,
		kListen,
		kAccept,
		kRecv,
		kShutdown
	};

	bool SetReuseAddr(int fd) {
		int yes = 1;
		return ::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes)) == 0;
	}

	void PrintPeer(const sockaddr_in &addr) {
		char ip[INET_ADDRSTRLEN] = {0};
		if (::inet_ntop(AF_INET, &addr.sin_addr, ip, sizeof(ip)) != nullptr) {
			std::cout << "Client connected: " << ip << ":" << ntohs(addr.sin_port) << '\n';
		} else {
			std::cout << "Client connected: (unknown)" << '\n';
		}
	}

	std::string Trim(const std::string &value) {
		const char *kWhitespace = " \t\r\n";
		const std::string::size_type first = value.find_first_not_of(kWhitespace);
		if (first == std::string::npos) {
			return "";
		}
		const std::string::size_type last = value.find_last_not_of(kWhitespace);
		return value.substr(first, last - first + 1);
	}

	std::vector<std::string> ParseButtons(const std::string &line) {
		std::vector<std::string> buttons;
		std::string token;
		for (char ch : line) {
			if (ch == ',') {
				std::string trimmed = Trim(token);
				if (trimmed.size() >= 2 && trimmed.front() == '"' && trimmed.back() == '"') {
					trimmed = trimmed.substr(1, trimmed.size() - 2);
				}
				if (!trimmed.empty()) {
					buttons.push_back(trimmed);
				}
				token.clear();
				continue;
			}
			token.push_back(ch);
		}

		std::string trimmed = Trim(token);
		if (trimmed.size() >= 2 && trimmed.front() == '"' && trimmed.back() == '"') {
			trimmed = trimmed.substr(1, trimmed.size() - 2);
		}
		if (!trimmed.empty()) {
			buttons.push_back(trimmed);
		}

		return buttons;
	}
}  // namespace

#define MAX_PWM 4095.0

// This is in centimeters per second
#define MAX_SPEED 10.0

int16_t meters_per_sec_to_pwm(double speed) {
    //if (speed < 0) speed = -speed;

    // First convert cm/s to m/s
    double temp_conversion = MAX_SPEED / 100.0;
    
    double pwm_value = (speed * MAX_PWM) / temp_conversion;

    std::cout << "PWM Value:" << pwm_value << "\n";

    if ((pwm_value < INT_MIN) || (pwm_value > INT_MAX)) {
        return 0;
    }

    if (pwm_value > MAX_PWM) pwm_value = MAX_PWM;

    return (int16_t) pwm_value;
}

std::vector<double> calculateOmniWheelSpeeds(double vx, double vy, double omega, double radius) {
    std::vector<double> wheelAngles;
   
    wheelAngles = {45.0, 135.0, 225.0, 315.0};

    std::vector<double> wheelSpeeds;

    double v_fl = vx + vy;
    double v_fr = vx - vy;
    double v_rl = vx - vy;
    double v_rr = vx + vy;

    wheelSpeeds.push_back(v_fr);
    wheelSpeeds.push_back(v_fl);
    wheelSpeeds.push_back(v_rr);
    wheelSpeeds.push_back(v_rl);

   return wheelSpeeds;
}


int main() {

    try {


        PiPCA9685::PCA9685 pwm("/dev/i2c-1", 0x7f);
        pwm.set_pwm_freq(60.0);
        float vx = 0.02;
        float vy = 0.04;
        
        std::vector<double> wheel_speeds = calculateOmniWheelSpeeds(vx, vy, 0, 0);
        
        uint16_t pwmSpeed1 = meters_per_sec_to_pwm(abs(wheel_speeds[0]));
        uint16_t pwmSpeed2 = meters_per_sec_to_pwm(abs(wheel_speeds[1]));
        uint16_t pwmSpeed3 = meters_per_sec_to_pwm(abs(wheel_speeds[2]));
        uint16_t pwmSpeed4 = meters_per_sec_to_pwm(abs(wheel_speeds[3]));
        std::cout << "Wheel speeds: " << wheel_speeds[0] << " " << wheel_speeds[1] << " " << wheel_speeds[2] << " " << wheel_speeds[3] << "\n";
        

        pwm.set_pwm(0,0, pwmSpeed1); // Front Right
        pwm.set_pwm(1,0, pwmSpeed2); // Front Left
        pwm.set_pwm(2,0, pwmSpeed3); // Rear Right
        pwm.set_pwm(3,0, pwmSpeed4); // Rear Left

        MotorPins pins;
        MotorTranslation motors("/dev/gpiochip0", pins);

        uint16_t speed = 0xFFFF;

        std::signal(SIGINT, HandleSignal);
        std::signal(SIGTERM, HandleSignal);

        ServerState state = ServerState::kInit;
        int listen_fd = -1;
        int client_fd = -1;

        std::string line_buffer;

        while (!g_stop && state != ServerState::kShutdown) {
            switch (state) {
                case ServerState::kInit: {
                    listen_fd = ::socket(AF_INET, SOCK_STREAM, 0);
                    if (listen_fd < 0) {
                        std::perror("socket");
                        state = ServerState::kShutdown;
                        break;
                    }
                    if (!SetReuseAddr(listen_fd)) {
                        std::perror("setsockopt");
                    }

                    sockaddr_in addr = {};
                    addr.sin_family = AF_INET;
                    addr.sin_addr.s_addr = htonl(INADDR_ANY);
                    addr.sin_port = htons(kListenPort);

                    if (::bind(listen_fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
                        std::perror("bind");
                        state = ServerState::kShutdown;
                        break;
                    }

                    if (::listen(listen_fd, kBacklog) < 0) {
                        std::perror("listen");
                        state = ServerState::kShutdown;
                        break;
                    }

                    std::cout << "Listening on TCP port " << kListenPort << "..." << '\n';
                    state = ServerState::kAccept;
                    break;
                }

                case ServerState::kAccept: {
                    motors.move_individual(wheel_speeds);
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    motors.stop_car();
                    //motors.go_ahead(speed);

                    //sockaddr_in client_addr = {};
                    //socklen_t client_len = sizeof(client_addr);
                    //client_fd = ::accept(listen_fd, reinterpret_cast<sockaddr *>(&client_addr), &client_len);
                    //if (client_fd < 0) {
                    //    if (errno == EINTR) {
                    //        continue;
                    //    }
                    //    std::perror("accept");
                    //    state = ServerState::kShutdown;
                    //    break;
                    //}
                    //PrintPeer(client_addr);
                    //line_buffer.clear();
                    //state = ServerState::kRecv;
                    break;
                }

                case ServerState::kRecv: {
                    char buf[kRecvBufSize];
                    const ssize_t n = ::recv(client_fd, buf, sizeof(buf), 0);
                    if (n == 0) {
                        std::cout << "Client disconnected." << '\n';
                        ::close(client_fd);
                        client_fd = -1;
                        state = ServerState::kAccept;
                        break;
                    }
                    if (n < 0) {
                        if (errno == EINTR) {
                            continue;
                        }
                        std::perror("recv");
                        ::close(client_fd);
                        client_fd = -1;
                        state = ServerState::kAccept;
                        break;
                    }

                    line_buffer.append(buf, static_cast<size_t>(n));

                    size_t pos = 0;
                    while ((pos = line_buffer.find('\n')) != std::string::npos) {
                        std::string line = line_buffer.substr(0, pos);
                        if (!line.empty() && line.back() == '\r') {
                            line.pop_back();
                        }
                        std::cout << "ESP8266: " << line << '\n';
                        const std::vector<std::string> buttons = ParseButtons(line);
                        if (!buttons.empty()) {
                            std::cout << "Buttons: ";
                            for (size_t i = 0; i < buttons.size(); ++i) {

                                 if (buttons[i] == "A") {
                                     motors.go_ahead(speed);
                                     std::this_thread::sleep_for(std::chrono::milliseconds(10));
                                     motors.stop_car();
                                 } else if (buttons[i] == "B") {
                                     motors.go_ahead(speed);
                                     std::this_thread::sleep_for(std::chrono::milliseconds(10));
                                     motors.stop_car();
                                 } else if (buttons[i] == "Select") {
                                     motors.go_ahead(speed);
                                     std::this_thread::sleep_for(std::chrono::milliseconds(10));
                                     motors.stop_car();
                                 } else if (buttons[i] == "Start") {
                                     motors.go_ahead(speed);
                                     std::this_thread::sleep_for(std::chrono::milliseconds(10));
                                     motors.stop_car();
                                 } else if (buttons[i] == "Up") {
                                     motors.go_ahead(speed);
                                     std::this_thread::sleep_for(std::chrono::milliseconds(10));
                                     motors.stop_car();
                                 } else if (buttons[i] == "Down") {
                                     motors.go_back(speed);
                                     std::this_thread::sleep_for(std::chrono::milliseconds(10));
                                     motors.stop_car();
                                 } else if (buttons[i] == "Left") {
                                     motors.turn_left(speed);
                                     std::this_thread::sleep_for(std::chrono::milliseconds(10));
                                     motors.stop_car();
                                 } else if (buttons[i] == "Right") {
                                     motors.turn_right(speed);
                                     std::this_thread::sleep_for(std::chrono::milliseconds(10));
                                     motors.stop_car();
                                 } else {
                                     motors.go_ahead(speed);
                                     std::this_thread::sleep_for(std::chrono::milliseconds(10));
                                     motors.stop_car();
                                 }

                                if (i > 0) {
                                    std::cout << ", ";
                                }
                                std::cout << buttons[i];
                            }
                            std::cout << '\n';
                        }
                        line_buffer.erase(0, pos + 1);
                    }
                    break;
                }

                case ServerState::kListen:
                case ServerState::kShutdown:
                default:
                    state = ServerState::kShutdown;
                    break;
            }
        }

        if (client_fd >= 0) {
            ::close(client_fd);
        }
        if (listen_fd >= 0) {
            ::close(listen_fd);
        }

        std::cout << "Server stopped." << '\n';
        return 0;
    } catch (const std::exception &ex) {
        std::cerr << "Error: " << ex.what() << std::endl;
        return 1;
    }

    return 0;
}

 */