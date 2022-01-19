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

bool SUCK::isOpen() const {
    return mOpen;
}

void SUCK::mReadLoop(bool oneShot) {
    auto buff = new uint8_t[4096];
    size_t n = 0;
    mRunning = true;
    while (mRunning) {
        n = ::read(mTtyFD, buff, readBuffSize);
        callback_dataReady(buff, n);
        if (oneShot) {
            break;
        }
    }
    mRunning = false;
}

bool SUCK::open(const std::string& ttyName, BAUD_RATE baudRate) {

    if (mOpen) return false;

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
            [this](auto &&PH1) { mReadLoop(std::forward<decltype(PH1)>(PH1)); }, true
    );
}

void SUCK::stopReadLoop() {
    mRunning = false;
}

void SUCK::write(uint8_t *data, size_t size) {
    size_t written = 0, pos = 0;
    do {
        written = ::write(mTtyFD, &data[pos], writeBuffSize);
        pos += written;
    } while (pos < size - 1);
}
