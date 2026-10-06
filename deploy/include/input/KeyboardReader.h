// KeyboardReader -- raw-mode stdin reader on its own thread.
//
// Adapted from unitree_rl_mjlab/deploy/include/isaaclab/devices/keyboard/
// keyboard.h (class Keyboard). Kept: termios canonical+echo disabled, a
// dedicated read thread, select() with a short timeout so the thread never
// blocks indefinitely, the escape-sequence decode for the arrow keys, and the
// "no key within the timeout means no key is held" semantics that gives a free
// release event.
//
// Differences, all deliberate:
//  - It refuses to start unless stdin is a TTY *and* this process owns the
//    terminal's foreground process group. A backgrounded reader would take
//    SIGTTIN and stop the process the first time it read; a launcher that
//    backgrounds the controller lands here, so this check is what makes the
//    feature safe to enable by default.
//  - Start/stop are explicit and restore the terminal, rather than happening in
//    the constructor/destructor.
//  - It knows nothing about the robot. It reports keys; InputManager owns the
//    mapping. Unitree instead reads FSMState::keyboard straight from an
//    observation term (robots/g1/src/State_RLBase.cpp:11-30), which would put a
//    second command path next to the RL observation layout -- exactly what this
//    port must not do.
#pragma once

#include <atomic>
#include <mutex>
#include <string>
#include <termios.h>
#include <thread>

namespace deploy
{

class KeyboardReader
{
public:
    ~KeyboardReader();

    KeyboardReader() = default;
    KeyboardReader(const KeyboardReader&) = delete;
    KeyboardReader& operator=(const KeyboardReader&) = delete;

    /// Puts stdin in raw mode and starts the read thread.
    ///
    /// Returns false, without touching the terminal, when stdin is not an
    /// interactive foreground TTY -- piped, redirected, or backgrounded. That
    /// is not an error: it is the normal case for a scripted run, and the
    /// caller simply continues with the ROS input paths.
    bool start();

    /// Restores the terminal and joins the thread. Safe to call twice, and
    /// called from the destructor, so a normal exit or a ROS SIGINT shutdown
    /// always leaves the terminal usable.
    void stop();

    bool running() const { return running_.load(std::memory_order_acquire); }

    /// The key currently held, or "" when none is. Arrow keys read as "up",
    /// "down", "left", "right". Cheap enough to call every control cycle.
    std::string key() const;

    /// Why start() declined, for the operator-facing log line.
    const std::string& unavailableReason() const { return reason_; }

private:
    void readLoop_();

    std::thread thread_;
    std::atomic<bool> running_{false};

    mutable std::mutex mutex_;
    std::string key_;

    termios saved_{};
    bool termios_saved_ = false;
    std::string reason_ = "not started";
};

} // namespace deploy
