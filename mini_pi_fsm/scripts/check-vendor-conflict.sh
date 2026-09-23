#!/usr/bin/env bash
# Run ON THE MINI-PI before `roslaunch mini_pi_fsm mini_pi_fsm.launch backend:=hightorque`.
#
# Reports -- and exits non-zero on -- anything that would fight mini_pi_fsm for
# the motor bus or the IMU. It only READS: it never kills a process and never
# edits an autostart entry or a service. What to do about a finding is printed.
#
# mini_pi_fsm_node runs the same process/serial scan itself and refuses the
# hightorque backend while a conflict exists (src/hardware/VendorConflict.cpp);
# this script additionally shows the autostart entries that will bring the
# vendor stack back at the next login.
set -u

conflict=0
warn=0
say() { printf '%s\n' "$*"; }

say "== vendor processes"
pat='sim2real_master_node|lr_control_node|rl_pd_controller|hightorque_hardware_sdk_node|livelybot_bringup/'
if pgrep -af "$pat" | grep -v "check-vendor-conflict" ; then
    conflict=1
else
    say "   none"
fi

say "== vendor roslaunch (respawns the controller and runs a second IMU driver)"
if pgrep -af 'roslaunch' | grep -E 'sim2real_master|joy_control_pi|sim2real\.launch' ; then
    conflict=1
else
    say "   none"
fi

say "== processes holding the motor serial (/dev/ttyACM*) or the IMU serial (/dev/ttyS7)"
holders=0
for dev in /dev/ttyACM* /dev/ttyS7; do
    [ -e "$dev" ] || continue
    if command -v fuser >/dev/null 2>&1; then
        out=$(fuser "$dev" 2>/dev/null)
        if [ -n "$out" ]; then
            say "   $dev held by pid(s):$out"
            for p in $out; do ps -o pid=,args= -p "$p" | sed 's/^/      /'; done
            holders=1
        fi
    fi
done
[ "$holders" -eq 0 ] && say "   none"
[ "$holders" -eq 1 ] && conflict=1

say "== XFCE/desktop autostart entries that launch the vendor stack"
for f in "$HOME"/.config/autostart/*.desktop; do
    [ -e "$f" ] || continue
    exec_line=$(grep -m1 '^Exec=' "$f" | cut -d= -f2-)
    hidden=$(grep -m1 '^Hidden=' "$f" | cut -d= -f2-)
    case "$exec_line" in
        *sim2real*|*hightorque_rl_control_box*)
            if [ "$hidden" = "true" ]; then
                say "   disabled: $f -> $exec_line"
            else
                say "   ENABLED : $f -> $exec_line"
                say "             WARNING: not running now, but it starts the vendor"
                say "             controller at the next login. To disable by hand:"
                say "             set Hidden=true in that file."
                warn=1
            fi ;;
    esac
done

say "== systemd units mentioning sim2real / livelybot"
if command -v systemctl >/dev/null 2>&1; then
    # Services only: device units are merely named after the Livelybot board.
    { systemctl list-units --type=service --all --no-legend 2>/dev/null
      systemctl --user list-units --type=service --all --no-legend 2>/dev/null; } \
        | grep -Ei 'sim2real|livelybot|hightorque' | sed 's/^/   /' || say "   none"
    say "   (listed for information; only the process/serial checks above are conflicts)"
fi

echo
if [ "$conflict" -ne 0 ]; then
    say "CONFLICT: stop the processes above before launching mini_pi_fsm with backend:=hightorque."
    say "  e.g. close the 'sim2real' terminal, or: rosnode kill -a ; pkill -f joy_control_pi.launch"
    say "  then re-run this script. Nothing was changed by this script."
    exit 1
fi
if [ "$warn" -ne 0 ]; then
    say "OK (with warning): nothing owns the motor bus now; an enabled autostart entry"
    say "will start the vendor controller at the next login -- re-run after any re-login."
else
    say "OK: no vendor controller, no foreign motor-serial owner, no enabled vendor autostart."
fi
exit 0
