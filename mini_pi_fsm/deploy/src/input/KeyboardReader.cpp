#include "input/KeyboardReader.h"

#include <ros/ros.h>

#include <sys/select.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>

namespace mini_pi
{

namespace
{
/// Poll period of the read thread. Also the release latency: with no byte in
/// this window the reader reports "no key held". Unitree uses 80 ms
/// (keyboard.h:107); 50 ms is the same idea slightly tightened, and it is still
/// far longer than one control cycle, so it costs nothing.
constexpr long kSelectUsec = 50000;
} // namespace

KeyboardReader::~KeyboardReader()
{
    stop();
}

bool KeyboardReader::start()
{
    if (running_) return true;

    if (!isatty(STDIN_FILENO))
    {
        // Piped, redirected, or </dev/null. Scripts that must not compete for
        // the operator's keyboard (scripts/run-rl.sh, scripts/test-joy.sh)
        // redirect stdin precisely to land here.
        reason_ = "stdin is not a terminal";
        return false;
    }

    // SIGTTIN is only raised when a BACKGROUND process reads from its own
    // CONTROLLING terminal. tcgetpgrp() failing means this terminal is not our
    // controlling one -- which is the normal case under roslaunch, because
    // roslaunch starts every node with setsid(), putting it in a fresh session
    // with no controlling terminal. Reading is safe there, and refusing would
    // disable the keyboard for the only way this controller is ever launched.
    const pid_t fg = tcgetpgrp(STDIN_FILENO);
    if (fg != -1 && fg != getpgrp())
    {
        reason_ = "this process is a background job on its controlling terminal";
        return false;
    }

    if (tcgetattr(STDIN_FILENO, &saved_) != 0)
    {
        reason_ = std::string("tcgetattr failed: ") + std::strerror(errno);
        return false;
    }
    termios raw = saved_;
    // Canonical mode off (deliver bytes without waiting for Enter) and echo off
    // (do not paint the operator's keys over the status dump). Same two flags
    // Unitree clears.
    raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0)
    {
        reason_ = std::string("tcsetattr failed: ") + std::strerror(errno);
        return false;
    }
    termios_saved_ = true;

    reason_.clear();
    running_ = true;
    thread_ = std::thread(&KeyboardReader::readLoop_, this);
    return true;
}

void KeyboardReader::stop()
{
    // The thread wakes at least every kSelectUsec, so clearing the flag is
    // enough to end it; no signal and no pipe-to-self is needed.
    const bool was = running_.exchange(false, std::memory_order_acq_rel);
    if (thread_.joinable()) thread_.join();

    if (termios_saved_)
    {
        tcsetattr(STDIN_FILENO, TCSANOW, &saved_);
        termios_saved_ = false;
    }
    if (was)
    {
        std::lock_guard<std::mutex> lk(mutex_);
        key_.clear();
    }
}

std::string KeyboardReader::key() const
{
    std::lock_guard<std::mutex> lk(mutex_);
    return key_;
}

void KeyboardReader::readLoop_()
{
    while (running_.load(std::memory_order_acquire))
    {
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(STDIN_FILENO, &fds);

        timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = kSelectUsec;

        const int n = select(STDIN_FILENO + 1, &fds, nullptr, nullptr, &tv);
        if (n < 0)
        {
            if (errno == EINTR) continue;   // a signal, not a failure
            break;
        }

        std::string k;
        if (n > 0)
        {
            char c = '\0';
            if (read(STDIN_FILENO, &c, 1) != 1) continue;

            if (c != '\033')
            {
                k.assign(1, c);
            }
            else
            {
                // Escape sequence: ESC '[' <final>. Decoded exactly as Unitree
                // does, so the arrow keys are usable as aliases.
                char b = '\0';
                if (read(STDIN_FILENO, &b, 1) == 1 && b == '[' &&
                    read(STDIN_FILENO, &b, 1) == 1)
                {
                    switch (b)
                    {
                        case 'A': k = "up";    break;
                        case 'B': k = "down";  break;
                        case 'C': k = "right"; break;
                        case 'D': k = "left";  break;
                        default:               break;
                    }
                }
            }
        }
        // n == 0: nothing arrived within the window, so no key is held and `k`
        // stays empty. That empty value is the release event.

        std::lock_guard<std::mutex> lk(mutex_);
        key_ = k;
    }

    running_.store(false, std::memory_order_release);
}

} // namespace mini_pi
