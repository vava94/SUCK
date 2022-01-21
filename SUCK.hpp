/**
 * Created by Vitaliy Kiselyov on 17.01.2022.
 * https://github.com/vava94/SUCK
 * vitkiselyov@gmail.com
 *
 * Simple Uart Communication Kernel.
 * Can be used only in Linux for now.
 *
 * Very simple usage:
 * 1) Construct
 * 2) Open
 * 3) Operate (write, read (once or in the loop))
 * 4) Close
 */

#ifndef SUCK_HPP
#define SUCK_HPP

#include <functional>
#include <string>
#include <thread>
#include <termios.h>

#ifndef __unix__
#error "Only Linux for now"
#endif

class SUCK {

public:
    /// Available baud rates for termios.
    enum BAUD_RATE {
        BR0         = B0,
        BR50        = B50,
        BR75        = B75,
        BR110       = B110,
        BR134       = B134,
        BR150       = B150,
        BR200       = B200,
        BR300       = B300,
        BR600       = B600,
        BR1200      = B1200,
        BR1800      = B1800,
        BR2400      = B2400,
        BR4800      = B4800,
        BR9600      = B9600,
        BR19200     = B19200,
        BR38400     = B38400,
        BR57600     = B57600,
        BR115200    = B115200,
        BR230400    = B230400,
        BR460800    = B460800,
        BR500000    = B500000,
        BR576000    = B576000,
        BR921600    = B921600,
        BR1000000   = B1000000,
        BR1152000   = B1152000,
        BR1500000   = B1500000,
        BR2000000   = B2000000,
        BR2500000   = B2500000,
        BR3000000   = B3000000,
        BR3500000   = B3500000,
        BR4000000   = B4000000
    };

    /// @brief Callback for received data.
    /// Could be binded with function which template is "void func(uint8_t* data, size_t size)".
    std::function<void(uint8_t*, size_t)> callback_dataReady = nullptr;

    /// @brief Logging variable. Enable to see some infos in console.
    bool log = false;

    /// @brief Receive buffer size in bytes.
    size_t readBuffSize = 4096;

    /// @brief Write buffer size in bytes.
    size_t writeBuffSize = 4096;

    explicit SUCK();

    /// @brief Closes the port.
    void close() const;

    /// @brief Checks port state.
    [[nodiscard]] bool isOpen() const;

    /// @brief Opens the port.
    /// @param ttyName - path to port, for example "/dev/ttyS0".
    /// @param baudRate - rate in built-in bauds.
    /// @return true in case of success.
    bool open(const std::string& ttyName, BAUD_RATE baudRate);

    /// @brief OPens the port.
    /// @param ttyName - path to port, for example "/dev/ttyS0".
    /// @param baudRate - rate in integer digits. Must be similar to built-in variants, else false.
    /// @return true in case of success.
    bool open(const std::string& ttyName, int baudRate);

    /// @brief Reads from port once. Returns data in callback.
    /// @param async If false then read blocks calling thread for a little.
    void read(bool async=true);

    /// @brief Set another baud rate to current port.
    bool setBaud(BAUD_RATE baudRate);

    /// @brief Starts a read loop.
    void startReadLoop();

    /// @brief Stops a read loop.
    void stopReadLoop();

    /// @brief Writes data to port.
    void write(uint8_t *data, size_t size) const;

private:
    int mDelay = 0;
    bool mOpen;
    bool mRunning;
    int mTtyFD = 0;
    struct termios mTty{};
    std::thread *mLoopThread;

    void mReadLoop(bool oneShot=false);
};


#endif //SUCK_HPP
