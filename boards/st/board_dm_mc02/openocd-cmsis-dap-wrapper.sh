#!/bin/bash
# CMSIS-DAP OpenOCD wrapper for Horco CMSIS-DAP debugger (VID:PID=faed:4873)
#
# Workarounds:
# 1. Kills stale OpenOCD processes left by cortex-debug or previous sessions
# 2. Resets USB device to recover from firmware lockup after cdc_acm binds
# 3. Unbinds cdc_acm before OpenOCD, rebinds after it exits
#
# Usage: Set OPENOCD=/path/to/this/script in cmake or environment
#        The script passes all arguments through to the real openocd.

REAL_OPENOCD="/usr/bin/openocd"
VID="faed"
# Some Horco probes enumerate as 4870, others as 4873; accept either.
PIDS="4870 4873"
USB_DEV=""

# Prefer the root-owned helper covered by the narrow NOPASSWD sudoers rule.
# Falling back to `sudo tee` keeps this wrapper usable on machines without it,
# but those machines will still ask for a password after sudo's cache expires.
PROBE_HELPER="${HPM_PROBE_HELPER:-/usr/local/sbin/hpm-probe-usb}"
[ -x "$PROBE_HELPER" ] || PROBE_HELPER=""

# Auto-detect the USB device path
auto_detect_device() {
    for dev in /sys/bus/usb/devices/*/; do
        [ "$(cat "$dev/idVendor" 2>/dev/null)" = "$VID" ] || continue
        for pid in $PIDS; do
            if [ "$(cat "$dev/idProduct" 2>/dev/null)" = "$pid" ]; then
                USB_DEV=$(basename "$dev")
                return 0
            fi
        done
    done
    return 1
}

# Kill any stale OpenOCD processes still holding the USB device
kill_stale_openocd() {
    local killed=0
    for pid in $(pgrep -x "openocd" 2>/dev/null); do
        if lsof -p "$pid" 2>/dev/null | grep -q "/dev/bus/usb"; then
            echo "[wrapper] Killing stale openocd (pid=$pid)..."
            kill "$pid" 2>/dev/null || true
            killed=1
        fi
    done
    if [ "$killed" = "1" ]; then
        sleep 1
    fi
}

# Release the USB interface from any kernel driver
release_usb_interface() {
    local iface_path="$1"
    local driver
    driver=$(readlink "$iface_path/driver" 2>/dev/null)
    if [ -n "$driver" ]; then
        local drv_name iface_name
        drv_name=$(basename "$driver")
        iface_name=$(basename "$iface_path")
        echo "[wrapper] Unbinding $iface_name from $drv_name..."
        echo -n "$USB_DEV:$iface_name" | sudo tee "/sys/bus/usb/drivers/$drv_name/unbind" >/dev/null 2>&1 || true
        return 0
    fi
    return 1
}

setup_device() {
    # Step 0: Kill any stale openocd holding the device
    kill_stale_openocd

    if ! auto_detect_device; then
        echo "[wrapper] CMSIS-DAP device (${VID}:${PID}) not found, running openocd directly..."
        exec "$REAL_OPENOCD" "$@"
    fi
    echo "[wrapper] Found CMSIS-DAP device at $USB_DEV"

    # Step 1: Save which interfaces had cdc_acm bound (for later rebind)
    NEED_REBIND=""
    for iface_path in "/sys/bus/usb/devices/$USB_DEV/${USB_DEV}:"*; do
        [ -d "$iface_path" ] || continue
        local iface driver
        iface=$(basename "$iface_path")
        driver=$(readlink "$iface_path/driver" 2>/dev/null || true)
        if echo "$driver" | grep -q "cdc_acm"; then
            NEED_REBIND="$NEED_REBIND $iface"
        fi
    done

    # Step 2: USB device hard reset
    echo "[wrapper] Resetting USB device..."
    if [ -n "$PROBE_HELPER" ]; then
        sudo "$PROBE_HELPER" reset
        auto_detect_device >/dev/null 2>&1 || true
    else
        echo -n "0" | sudo tee "/sys/bus/usb/devices/$USB_DEV/authorized" >/dev/null 2>&1 || true
        sleep 1
        echo -n "1" | sudo tee "/sys/bus/usb/devices/$USB_DEV/authorized" >/dev/null 2>&1 || true
    fi
    sleep 2

    # Step 3: Wait for device to fully re-enumerate
    for i in $(seq 1 10); do
        if [ -e "/sys/bus/usb/devices/$USB_DEV/idVendor" ]; then
            break
        fi
        sleep 0.5
    done

    # Step 4: Unbind cdc_acm from all interfaces
    if [ -n "$PROBE_HELPER" ]; then
        sudo "$PROBE_HELPER" unbind-cdc || true
    else
        for iface_path in "/sys/bus/usb/devices/$USB_DEV/${USB_DEV}:"*; do
            [ -d "$iface_path" ] || continue
            release_usb_interface "$iface_path"
        done
    fi
    sleep 0.5
}

cleanup_device() {
    [ -n "$USB_DEV" ] || return
    [ -n "$NEED_REBIND" ] || return

    echo "[wrapper] Rebinding cdc_acm..."
    if [ -n "$PROBE_HELPER" ]; then
        sudo "$PROBE_HELPER" bind-cdc || true
    else
        for iface in $NEED_REBIND; do
            if [ ! -L "/sys/bus/usb/devices/$USB_DEV/$iface/driver" ]; then
                echo -n "$USB_DEV:$iface" | sudo tee "/sys/bus/usb/drivers/cdc_acm/bind" >/dev/null 2>&1 || true
            fi
        done
    fi
}

# Main
setup_device "$@"

# Run the real openocd
"$REAL_OPENOCD" "$@"
RET=$?

cleanup_device
exit $RET
