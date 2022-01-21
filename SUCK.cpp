/**
 * Created by Vitaliy Kiselyov on 17.01.2022.
 * https://github.com/vava94/SUCK
 * vitkiselyov@gmail.com
 */

#include "SUCK.hpp"

#include <fcntl.h>
#include <iostream>
#include <cstring>
#include <unistd.h>

SUCK::SUCK() {
    mOpen = false;
    mRunning = false;
    mLoopThread = nullptr;
}

void SUCK::close() const {
    if (mRunning) {
        if (log) {
            std::cerr << "SUCK: Attempt to close while running." << std::endl;
        }
        return;
    }
    ::close(mTtyFD);
}

bool SUCK::isOpen() const {
    return mOpen;
}

void SUCK::mReadLoop(bool oneShot) {
    auto buff = new uint8_t[4096];
    ssize_t n = 0;
    mRunning = true;
    while (mRunning) {
        n = ::read(mTtyFD, buff, readBuffSize);
        if (n >= 0) {
            callback_dataReady(buff, n);
            memset(buff, 0, n);
        } else {
            std:: this_thread::sleep_for(std::chrono::nanoseconds(mDelay));
        }
        if (oneShot) {
            break;
        }
    }
    delete[] buff;
    mRunning = false;
}

bool SUCK::open(const std::string& ttyName, BAUD_RATE baudRate) {

    if (mOpen) return false;

    if (baudRate == BR50) mDelay = 1000000000 / (2 * 50);
    else if (baudRate == BR75) mDelay = 1000000000 / (2 * 75);
    else if (baudRate == BR110) mDelay = 1000000000 / (2 * 110);
    else if (baudRate == BR134) mDelay = 1000000000 / (2 * 134);
    else if (baudRate == BR150) mDelay = 1000000000 / (2 * 150);
    else if (baudRate == BR200) mDelay = 1000000000 / (2 * 200);
    else if (baudRate == BR300) mDelay = 1000000000 / (2 * 300);
    else if (baudRate == BR600) mDelay = 1000000000 / (2 * 600);
    else if (baudRate == BR1200) mDelay = 1000000000 / (2 * 1200);
    else if (baudRate == BR1800) mDelay = 1000000000 / (2 * 1800);
    else if (baudRate == BR2400) mDelay = 1000000000 / (2 * 2400);
    else if (baudRate == BR4800) mDelay = 1000000000 / (2 * 4800);
    else if (baudRate == BR9600) mDelay = 1000000000 / (2 * 9600);
    else if (baudRate == BR19200) mDelay = 1000000000 / (2 * 19200);
    else if (baudRate == BR38400) mDelay = 1000000000 / (2 * 38400);
    else if (baudRate == BR57600) mDelay = 1000000000 / (2 * 57600);
    else if (baudRate == BR115200) mDelay = 1000000000 / (2 * 115200);
    else if (baudRate == BR230400) mDelay = 1000000000 / (2 * 230400);
    else if (baudRate == BR460800) mDelay = 1000000000 / (2 * 460800);
    else if (baudRate == BR500000) mDelay = 1000000000 / (2 * 500000);
    else if (baudRate == BR576000) mDelay = 1000000000 / (2 * 576000);
    else if (baudRate == BR921600) mDelay = 1000000000 / (2 * 921600);
    else if (baudRate == BR1000000) mDelay = 1000000000 / (2 * 1000000);
    else if (baudRate == BR1152000) mDelay = 1000000000 / (2 * 1152000);
    else if (baudRate == BR1500000) mDelay = 1000000000 / (2 * 1500000);
    else if (baudRate == BR2000000) mDelay = 1000000000 / (2 * 2000000);
    else if (baudRate == BR2500000) mDelay = 1000000000 / (2 * 2500000);
    else if (baudRate == BR3000000) mDelay = 1000000000 / (2 * 3000000);
    else if (baudRate == BR3500000) mDelay = 1000000000 / (2 * 3500000);
    else if (baudRate == BR4000000) mDelay = 1000000000 / (2 * 4000000);

    mTtyFD = ::open(ttyName.c_str(), O_RDWR | O_NONBLOCK|O_NOCTTY);
    if (mTtyFD < 0) {
        if (log) {
            std::cout << "Can't open " << ttyName << std::endl;
        }
        return false;
    }
    if (tcgetattr(mTtyFD, &mTty) !=0) {
        if (log) {
            std::cout << "Error " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
        }
        return false;
    }

    /// Set BaudRate
    cfsetospeed(&mTty, (speed_t)baudRate);
    cfsetispeed(&mTty, (speed_t)baudRate);
    /// Set other settings, 8n1
    mTty.c_cflag &= ~PARENB;
    mTty.c_cflag &= ~CSTOPB;
    mTty.c_cflag &= ~CSIZE;
    mTty.c_cflag |= CS8;
    /// No flow control
    mTty.c_cflag &= ~CRTSCTS;
    /// Read doesn't block
    mTty.c_cc[VMIN] = 1;
    /// Timeout for reading 0.5 seconds
    mTty.c_cc[VTIME] = 5;
    /// Turn on READ & ignore ctrl lines
    mTty.c_cflag |= CREAD | CLOCAL;
    /// Make raw
    cfmakeraw(&mTty);
    /// Flush and apply
    tcflush(mTtyFD, TCIFLUSH);
    if (tcsetattr(mTtyFD, TCSANOW, &mTty) != 0) {
        std::cout << "Error " << errno << " from tcsetattr" << std::endl;
        return false;
    }
    mOpen = true;
    return true;
}

bool SUCK::open(const std::string &ttyName, int baudRate) {
    BAUD_RATE br = BR0;

    if (baudRate == 50) br = BR50;
    else if (baudRate == 75) br = BR75;
    else if (baudRate == 110) br = BR110;
    else if (baudRate == 134) br = BR134;
    else if (baudRate == 150) br = BR150;
    else if (baudRate == 200) br = BR200;
    else if (baudRate == 300) br = BR300;
    else if (baudRate == 600) br = BR600;
    else if (baudRate == 1200) br = BR1200;
    else if (baudRate == 1800) br = BR1800;
    else if (baudRate == 2400) br = BR2400;
    else if (baudRate == 4800) br = BR4800;
    else if (baudRate == 9600) br = BR9600;
    else if (baudRate == 19200) br = BR19200;
    else if (baudRate == 38400) br = BR38400;
    else if (baudRate == 57600) br = BR57600;
    else if (baudRate == 115200) br = BR115200;
    else if (baudRate == 230400) br = BR230400;
    else if (baudRate == 460800) br = BR460800;
    else if (baudRate == 500000) br = BR500000;
    else if (baudRate == 576000) br = BR576000;
    else if (baudRate == 921600) br = BR921600;
    else if (baudRate == 1000000) br = BR1000000;
    else if (baudRate == 1152000) br = BR1152000;
    else if (baudRate == 1500000) br = BR1500000;
    else if (baudRate == 2000000) br = BR2000000;
    else if (baudRate == 2500000) br = BR2500000;
    else if (baudRate == 3000000) br = BR3000000;
    else if (baudRate == 3500000) br = BR3500000;
    else if (baudRate == 4000000) br = BR4000000;

    if (br == BR0) {
        if (log) {
            std::cerr << "Wrong baud rate: " << baudRate << std::endl;
        }
        return false;
    }
    else {
        return open(ttyName, br);
    }
}

void SUCK::read(bool async) {

    if (callback_dataReady == nullptr) {
        std::cerr << "SUCK: dataReady callback is NULL." << std::endl;
        return;
    }
    if (mRunning) {
        std::cerr << "SUCK: Can't use read command while reading in loop is active." << std::endl;
        return;
    }
    if (async) {
        while (mRunning) {
            if (log) {
                std::cout << "Read thread is busy. Waiting 10 msecs..." << std::endl;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        mLoopThread = new std::thread(
                [this](auto &&PH1) { mReadLoop(std::forward<decltype(PH1)>(PH1)); }, true
        );
    } else {
        mReadLoop(true);
    }
}

bool SUCK::setBaud(BAUD_RATE baudRate) {
    cfsetospeed(&mTty, (speed_t)baudRate);
    cfsetispeed(&mTty, (speed_t)baudRate);
    tcflush(mTtyFD, TCIFLUSH);
    if (tcsetattr(mTtyFD, TCSANOW, &mTty) != 0) {
        std::cout << "Error " << errno << " from tcsetattr" << std::endl;
        return false;
    }
    return true;
}

void SUCK::startReadLoop() {
    if (mRunning) {
        std::cerr << "SUCK: Reading in loop is active." << std::endl;
        return;
    }
    mLoopThread = new std::thread(
            [this](auto &&PH1) { mReadLoop(std::forward<decltype(PH1)>(PH1)); }, false
    );
}

void SUCK::stopReadLoop() {
    mRunning = false;
}

void SUCK::write(uint8_t *data, size_t size) const {
    size_t written = 0, pos = 0;
    do {
        written = ::write(mTtyFD, &data[pos], writeBuffSize);
        pos += written;
    } while (pos < size - 1);
}
