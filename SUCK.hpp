/**
 * Created by Vitaliy Kiselyov on 17.01.2022.
 * https://github.com/vava94/SUCK
 * vitkiselyov@gmail.com
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

    std::function<void(uint8_t*, size_t)> callback_dataReady = nullptr;
    bool log = true;
    size_t readBuffSize = 4096;
    size_t writeBuffSize = 4096;

    explicit SUCK();
    [[nodiscard]] bool isOpen() const;
    bool open(const std::string& ttyName, BAUD_RATE baudRate);
    void read(bool async=true);
    bool setBaud(BAUD_RATE baudRate);
    void startReadLoop();
    void stopReadLoop();
    void write(uint8_t *data, size_t size);

private:
    bool mOpen;
    bool mRunning;
    int mTtyFD = 0;
    struct termios mTty{};
    std::thread *mLoopThread;

    void mReadLoop(bool oneShot=false);

};


#endif //SUCK_HPP
