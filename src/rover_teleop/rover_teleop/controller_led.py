#!/usr/bin/env python3
"""
Drive-state indicator on a PlayStation controller's light bar.

Deliberately free of any ROS import: the colour rules and the device writing
are the parts worth testing, and they test far more cheaply on their own.

Two backends are tried, in order:

  1. sysfs LED class (`/sys/class/leds/*:rgb:indicator` for DualSense,
     `/sys/class/leds/*:global` + per-channel nodes for DualShock 4). This is
     what the in-kernel `hid-playstation` driver exposes, present on kernel
     5.12+ when the driver is built. This is the "normal" desktop/laptop
     case.

  2. Raw USB HID output reports written directly to `/dev/hidrawN`. This
     needs no `hid-playstation` support at all — only the generic `hidraw`
     subsystem, present since kernel 2.6.34 — so it covers systems where
     `hid-playstation` is missing or was never built: NVIDIA Jetson's L4T
     kernel tree at the time of writing, and any older kernel that predates
     the driver. The controller is claimed by `hid-generic` in this case,
     which handles ordinary input fine but exposes no LED class node.

Everything here is best-effort. A controller with no light bar, a missing
udev rule, a light bar that has not reappeared yet after a reconnect — none
of these may disturb teleop, so every failure path ends in a log line and
nothing else.
"""

import glob
import os


# --- Backend 1: sysfs LED class (requires hid-playstation) -----------------

# DualSense (and DualSense Edge): one multicolor LED carrying an RGB triple.
DUALSENSE_GLOB = '/sys/class/leds/*:rgb:indicator'
# DualShock 4: three single-colour LEDs plus a ':global' on/off gate. Same
# hid-playstation driver, different sysfs shape — see docs/hardware/joystick.md.
DUALSHOCK_GLOB = '/sys/class/leds/*:global'

DUALSHOCK_CHANNELS = ('red', 'green', 'blue')

# The light bar index changes on every reconnect (input19 → input21 → …),
# hence the globbing on each repaint rather than a path resolved once.


# --- Backend 2: raw hidraw output reports (no driver required) -------------

# Sony's USB vendor ID, and the product IDs of every pad this fallback knows
# how to address. Bus type isn't checked — only whether hidraw HID_ID reports
# this vendor/product pair — so a Bluetooth-connected pad will match here too
# even though the report layout below is the USB one; see the caveat on
# _build_hidraw_report.
SONY_VENDOR_ID = '054c'
DUALSENSE_PRODUCT_IDS = frozenset({'0ce6', '0df2'})   # DualSense, DualSense Edge
DUALSHOCK4_PRODUCT_IDS = frozenset({'05c4', '09cc'})  # DualShock 4 v1, v2


LED_MOTORS_OFF       = (255, 0, 0)      # red     — hardware inactive
LED_MOTORS_INHIBITED = (255, 0, 255)    # magenta — faults cleared, needs a motor-button cycle
LED_CONTROLLER_OFF   = (255, 120, 0)    # orange  — motors on, controller inactive
LED_NAV              = (0, 0, 255)      # blue    — yielding to nav
LED_DRIVING          = (0, 255, 0)      # green   — joystick in command

# Not a drive state — the absence of one. Painted on the way out so the bar
# stops advertising whatever was true when the interpreter was last alive.
LED_NODE_DOWN        = (255, 255, 255)  # white   — interpreter not running


def led_colour(motors_enabled: bool, motors_inhibited: bool,
               controller_active: bool, joystick_active: bool):
    """Resolve the drive state to a colour, most-serious state first.

    The ordering is the whole point: the light bar may only ever show a state
    at least as permissive as reality. Green in particular must mean the
    joystick can actually move the rover right now.
    """
    if not motors_enabled:
        return LED_MOTORS_OFF
    if motors_inhibited:
        # Motors report enabled, but the hardware was told to drop its latched
        # faults and will not drive until the motor button is cycled. Showing
        # green here would promise control that does not exist.
        return LED_MOTORS_INHIBITED
    if not controller_active:
        return LED_CONTROLLER_OFF
    if not joystick_active:
        return LED_NAV
    return LED_DRIVING


def _max_brightness(base: str, fallback: str = '255') -> str:
    """The device's full-scale brightness, as a string ready to write."""
    try:
        with open(os.path.join(base, 'max_brightness')) as handle:
            value = handle.read().strip()
    except OSError:
        return fallback
    return value or fallback


class ControllerLed:
    """Paints a PlayStation light bar with the current drive state.

    The colour is rewritten on a timer rather than only when the state
    changes, because this node is not the only thing that touches the bar:

      * the kernel driver picks its own colour when a controller enumerates;
      * SDL — which `game_controller_node` is built on — sets the light bar
        when it opens the device, blue for player 1;
      * udev may not have handed the new LED device to `plugdev` yet at the
        moment a reconnect is first noticed, so the first write can fail;
      * on the hidraw fallback path there is no kernel-side state at all —
        every repaint is the only thing keeping the bar showing the truth.

    Any of those leaves the bar showing something the rover never chose —
    default blue reads as 'yielding to navigation' to an operator. Rewriting
    unconditionally means the bar converges on the truth within one period no
    matter who painted over it, and no matter why.

    Writes are cheap (a couple of small sysfs writes, or one hidraw write) and
    idempotent, so this costs nothing worth measuring.
    """

    def __init__(self, logger,
                 dualsense_glob: str = DUALSENSE_GLOB,
                 dualshock_glob: str = DUALSHOCK_GLOB,
                 sony_vendor_id: str = SONY_VENDOR_ID,
                 dualsense_product_ids=DUALSENSE_PRODUCT_IDS,
                 dualshock4_product_ids=DUALSHOCK4_PRODUCT_IDS):
        self._log = logger
        self._dualsense_glob = dualsense_glob
        self._dualshock_glob = dualshock_glob
        self._sony_vendor_id = sony_vendor_id
        self._dualsense_product_ids = dualsense_product_ids
        self._dualshock4_product_ids = dualshock4_product_ids

        self._colour = None
        # Failure kinds already reported at warn level. Keyed by errno rather
        # than by path so a permission problem is announced once, not once per
        # reconnect — the path carries an input index that keeps changing, and
        # with a repaint timer running an un-deduped warning would be endless.
        self._warned = set()

    @property
    def colour(self):
        """Colour last asked for, or None if nothing has been painted yet."""
        return self._colour

    def set(self, colour) -> bool:
        """Paint `colour` now and remember it for subsequent repaints."""
        self._colour = colour
        return self._attempt()

    def repaint(self) -> bool:
        """Rewrite the last colour. Driven by a timer; see the class docstring
        for why this is unconditional rather than only-when-stale."""
        if self._colour is None:
            return False
        return self._attempt()

    def _attempt(self) -> bool:
        sysfs_targets = self._sysfs_targets()

        if sysfs_targets:
            ok = True
            for path, value in sysfs_targets:
                try:
                    with open(path, 'w') as handle:
                        handle.write(value)
                except OSError as exc:
                    ok = False
                    self._report(path, exc)
            return ok

        # No sysfs LED class node found. Either there is genuinely no pad
        # plugged in, or `hid-playstation` isn't loaded — the pad is claimed
        # by `hid-generic` instead, which handles input fine but creates no
        # LED class node at all. Fall back to hidraw, which needs no driver
        # support beyond the generic hidraw subsystem.
        hidraw_targets = self._hidraw_targets()

        if not hidraw_targets:
            # Covers both: no light bar at all (any Xbox controller), or a
            # PlayStation pad that has not enumerated yet under either
            # backend. Both are normal and neither is worth a warning; the
            # next repaint picks the device up as soon as it appears.
            return False

        ok = True
        for path, product_id in hidraw_targets:
            try:
                self._write_hidraw(path, product_id)
            except (OSError, ValueError) as exc:
                ok = False
                self._report(path, exc)
        return ok

    def _report(self, path: str, exc: Exception):
        """A found-but-unwritable light bar is a real misconfiguration — say so
        out loud, but only the first time each kind of failure shows up.

        Deduped rather than throttled: with a repaint running on a timer this
        would otherwise repeat forever, at debug level as much as at warn.
        """
        errno = getattr(exc, 'errno', None)
        key = (path.startswith('/dev/hidraw'), errno)
        if key in self._warned:
            return
        self._log.debug(f'controller LED write to {path} failed: {exc!r}')
        self._warned.add(key)

        if path.startswith('/dev/hidraw'):
            self._log.warning(
                f'controller LED found at {path} (hidraw fallback) but not '
                f'writable: {exc!r} — the light bar will not track the drive '
                f'state. Add a udev rule granting write access to this '
                f'device (MODE="0666" or GROUP="plugdev" on '
                f'SUBSYSTEM=="hidraw" matched to the pad\'s idVendor/'
                f'idProduct), then reconnect the pad.')
        else:
            self._log.warning(
                f'controller LED found at {path} but not writable: {exc!r} — '
                f'the light bar will not track the drive state. Install the '
                f'udev rule with: scripts/setup_host.sh <rover|local> '
                f'--joystick-led (and make sure this user is in the plugdev '
                f'group)')

    def _sysfs_targets(self):
        """(path, value) pairs to write via the sysfs LED class backend, in
        the order they must be written. Empty if hid-playstation has created
        no LED node — the caller falls back to hidraw in that case."""
        if self._colour is None:
            return []

        red, green, blue = self._colour
        targets = []

        for base in sorted(glob.glob(self._dualsense_glob)):
            # Output is intensity × brightness, and the driver resets
            # brightness on reconnect, so pinning the intensities alone can
            # leave the bar dark. brightness goes last: writing it is what
            # latches the new colour out to the controller.
            targets.append((os.path.join(base, 'multi_intensity'),
                            f'{red} {green} {blue}'))
            targets.append((os.path.join(base, 'brightness'),
                            _max_brightness(base)))

        for gate in sorted(glob.glob(self._dualshock_glob)):
            prefix = gate[: -len(':global')]
            channels = [(f'{prefix}:{name}', value) for name, value
                        in zip(DUALSHOCK_CHANNELS, (red, green, blue))]
            # ':global' is not a PlayStation-only name; only treat it as a
            # light bar when the three colour channels sit beside it.
            if not all(os.path.isdir(path) for path, _ in channels):
                continue
            targets.extend((os.path.join(path, 'brightness'), str(value))
                           for path, value in channels)
            targets.append((os.path.join(gate, 'brightness'),
                            _max_brightness(gate)))

        return targets

    def _hidraw_targets(self):
        """(path, product_id) pairs for every hidraw node that looks like a
        PlayStation pad, identified by vendor/product ID from the kernel's
        HID_ID uevent property rather than by name (names aren't stable
        across reconnects any more than the sysfs LED index is)."""
        if self._colour is None:
            return []

        targets = []
        for uevent_path in sorted(glob.glob('/sys/class/hidraw/hidraw*/device/uevent')):
            vendor, product = self._read_hid_id(uevent_path)
            if vendor != self._sony_vendor_id:
                continue
            if product not in self._dualsense_product_ids | self._dualshock4_product_ids:
                continue
            hidraw_name = os.path.basename(os.path.dirname(os.path.dirname(uevent_path)))
            targets.append((f'/dev/{hidraw_name}', product))
        return targets

    @staticmethod
    def _read_hid_id(uevent_path: str):
        """Parse HID_ID=<bus>:<vendor>:<product> out of a hidraw uevent file.
        Returns (vendor, product) as lowercase hex strings, or (None, None)
        if the file is unreadable or the line is missing."""
        try:
            with open(uevent_path) as handle:
                text = handle.read()
        except OSError:
            return None, None

        for line in text.splitlines():
            if line.startswith('HID_ID='):
                parts = line.split('=', 1)[1].split(':')
                if len(parts) == 3:
                    return parts[1][-4:].lower(), parts[2][-4:].lower()
        return None, None

    def _write_hidraw(self, path: str, product_id: str):
        """Write one raw USB HID output report setting the light bar colour.

        Caveat: this report layout is the USB one. A pad connected over
        Bluetooth uses a different report ID and a CRC32 trailer, which this
        does not build — a Bluetooth-connected pad matched here will accept
        the write (no OSError) but the bar will not change. Wired/USB is the
        expected connection for a teleop base station, so this is left
        unhandled rather than guessed at; revisit if BT teleop is ever added.
        """
        red, green, blue = self._colour
        report = self._build_hidraw_report(product_id, red, green, blue)
        with open(path, 'wb') as handle:
            handle.write(report)

    @staticmethod
    def _build_hidraw_report(product_id: str, red: int, green: int, blue: int) -> bytes:
        """Build the USB output report for the given Sony product ID.

        Byte offsets are from community reverse-engineering (no official
        spec exists for either pad's HID protocol). The DualSense layout has
        seen more independent confirmation than the DualShock 4 one below;
        treat the latter as best-effort until confirmed on real hardware.
        """
        if product_id in DUALSENSE_PRODUCT_IDS:
            report = bytearray(48)
            report[0] = 0x02  # report ID
            report[1] = 0xFF  # valid_flag0: enable motor/audio fields (unused here, but several
                               # firmware revisions ignore flag2 unless flag0 is also non-zero)
            report[2] = 0x04  # valid_flag2: enable lightbar colour control specifically
            report[45] = red
            report[46] = green
            report[47] = blue
            return bytes(report)

        if product_id in DUALSHOCK4_PRODUCT_IDS:
            report = bytearray(32)
            report[0] = 0x05  # report ID
            report[1] = 0xFF  # flags: enable rumble + LED + flash fields
            report[6] = red
            report[7] = green
            report[8] = blue
            return bytes(report)

        raise ValueError(f'no known HID output report layout for Sony product {product_id}')
