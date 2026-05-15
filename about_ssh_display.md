# Virtual Display for Headless SSH Sessions

## Problem
When running GUI applications like RViz2 over SSH on a headless machine
(e.g., ASUS NUC robot PC with no monitor), Qt crashes immediately:

```
qt.qpa.xcb: could not connect to display
This application failed to start because no Qt platform plugin could be initialized.
```

This happens because there is no X display server running on the robot PC,
so `$DISPLAY` is empty and Qt has nowhere to render.

## Solution: Xvfb (X Virtual Framebuffer)

Xvfb creates a fake X display entirely in memory. GUI apps think they have
a real monitor and launch without crashing. No actual screen output is
produced — it only prevents the "no display" error.

### Add to `~/.bashrc` on the robot PC

```bash
# Virtual Monitor — activates only when no real display is detected
if [ -z "$DISPLAY" ] && [ -z "$WAYLAND_DISPLAY" ]; then
    export DISPLAY=:99
    pgrep Xvfb >/dev/null || Xvfb :99 -screen 0 1920x1080x24 &
fi
```

### How it works

1. `[ -z "$DISPLAY" ] && [ -z "$WAYLAND_DISPLAY" ]`
   Checks if both `DISPLAY` and `WAYLAND_DISPLAY` are empty (no real
   monitor session). This guard ensures the block is skipped when you
   log in locally with a real monitor — the desktop session sets these
   variables automatically.

2. `export DISPLAY=:99`
   Tells all GUI apps in this shell to use X display number 99.
   Using a high number avoids conflicts with real displays (`:0`, `:1`).

3. `pgrep Xvfb >/dev/null || Xvfb :99 -screen 0 1920x1080x24 &`
   Starts Xvfb only if it is not already running. The `&` runs it
   in the background. The virtual screen is 1920x1080 at 24-bit color.

### Wayland note

If `QT_QPA_PLATFORM` is set to `wayland` in your environment, Qt will
try the Wayland plugin instead of xcb and fail under Xvfb. Fix by also
adding to `~/.bashrc`:

```bash
export QT_QPA_PLATFORM=xcb
```

## When you have a real monitor

If you plug a monitor into the robot PC and log in at the desktop, the
desktop session sets `$DISPLAY=:0` (X11) or `$WAYLAND_DISPLAY=wayland-0`
(Wayland) before `~/.bashrc` runs. The `if` guard detects this and skips
the entire Xvfb block — no changes needed.

## Install Xvfb

```bash
sudo apt install xvfb
```