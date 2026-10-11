#!/usr/bin/env python3
"""Send keyboard keys to a running X11 window by title or PID.

Used to drive the Genetic Car viewer (keys G/V/R/D/N) when a bug shows up
only after a key press. By default events are delivered with XSendEvent
directly to the target window, without touching focus; --focus falls back to
XTEST, briefly taking and then restoring the input focus.
"""

import argparse
import sys
import time

try:
    from Xlib import X, XK, display
    from Xlib.ext import xtest
    from Xlib.protocol import event as xevent
except ImportError as exc:  # pragma: no cover - environment guard
    print("press_keys: python-xlib is not available: %s" % exc, file=sys.stderr)
    print("press_keys: install it with 'pip install python-xlib'", file=sys.stderr)
    sys.exit(1)

DEFAULT_TITLE = "Genetic Car - Frame Viewer"

NAMED_KEYS = {
    "space": "space",
    "return": "Return",
    "enter": "Return",
    "esc": "Escape",
    "escape": "Escape",
    "tab": "Tab",
    "left": "Left",
    "right": "Right",
    "up": "Up",
    "down": "Down",
}


class WindowNotFound(Exception):
    pass


def _atom(d, name):
    return d.intern_atom(name)


def _window_title(d, win):
    try:
        prop = win.get_full_property(_atom(d, "_NET_WM_NAME"), _atom(d, "UTF8_STRING"))
        if prop and prop.value:
            value = prop.value
            if isinstance(value, bytes):
                return value.decode("utf-8", "replace")
            return str(value)
    except Exception:
        pass
    try:
        name = win.get_wm_name()
        if name:
            return name
    except Exception:
        pass
    return ""


def _window_pid(d, win):
    try:
        prop = win.get_full_property(_atom(d, "_NET_WM_PID"), X.AnyPropertyType)
        if prop and prop.value:
            return int(prop.value[0])
    except Exception:
        pass
    return None


def _ewmh_windows(d, root):
    try:
        prop = root.get_full_property(_atom(d, "_NET_CLIENT_LIST"), X.AnyPropertyType)
        if prop and prop.value:
            return list(prop.value)
    except Exception:
        pass
    return []


def _tree_windows(root, limit=10000):
    found = []
    stack = [root]
    seen = set()
    while stack and len(found) < limit:
        win = stack.pop()
        key = getattr(win, "id", None)
        if key in seen:
            continue
        seen.add(key)
        found.append(win)
        try:
            stack.extend(win.query_tree().children)
        except Exception:
            continue
    return found


def list_windows(d):
    root = d.screen().root
    windows = []
    seen = set()
    for wid in _ewmh_windows(d, root):
        if wid in seen:
            continue
        seen.add(wid)
        windows.append(d.create_resource_object("window", wid))
    for win in _tree_windows(root):
        wid = getattr(win, "id", None)
        if wid in seen:
            continue
        seen.add(wid)
        windows.append(win)

    result = []
    for win in windows:
        title = _window_title(d, win)
        pid = _window_pid(d, win)
        if title or pid is not None:
            result.append((win, title, pid))
    return result


def find_window(d, title_substr, pid):
    matches = []
    for win, title, wpid in list_windows(d):
        if title_substr and title_substr.lower() not in title.lower():
            continue
        if pid is not None and wpid != pid:
            continue
        matches.append((win, title, wpid))
    return matches


def resolve_target(d, args):
    deadline = time.time() + max(0.0, args.wait)
    while True:
        matches = find_window(d, args.window, args.pid)
        if len(matches) == 1:
            return matches[0]
        if len(matches) > 1 and args.wait <= 0:
            raise WindowNotFound(_ambiguous_message(matches))
        if time.time() >= deadline:
            if len(matches) > 1:
                raise WindowNotFound(_ambiguous_message(matches))
            raise WindowNotFound(_not_found_message(d, args))
        time.sleep(0.2)


def _not_found_message(d, args):
    lines = ["press_keys: no window matched."]
    filters = []
    if args.window:
        filters.append("title contains %r" % args.window)
    if args.pid is not None:
        filters.append("pid == %d" % args.pid)
    lines.append("  filters: " + ("; ".join(filters) if filters else "none"))
    lines.append("  available windows:")
    for win, title, pid in list_windows(d):
        lines.append("    0x%08x pid=%s title=%r" % (win.id, pid, title))
    return "\n".join(lines)


def _ambiguous_message(matches):
    lines = ["press_keys: %d windows matched, narrow the filter:" % len(matches)]
    for win, title, pid in matches:
        lines.append("    0x%08x pid=%s title=%r" % (win.id, pid, title))
    return "\n".join(lines)


def keysym_for(token):
    t = token.strip()
    low = t.lower()
    if low in NAMED_KEYS:
        name = NAMED_KEYS[low]
    elif len(t) == 1:
        name = t
    else:
        return None
    keysym = XK.string_to_keysym(name)
    if keysym == 0:
        return None
    return keysym


def send_xsend(d, root, win, keycode, hold):
    press = xevent.KeyPress(
        time=X.CurrentTime, root=root, window=win, same_screen=1, child=X.NONE,
        root_x=0, root_y=0, event_x=0, event_y=0, state=0, detail=keycode)
    release = xevent.KeyRelease(
        time=X.CurrentTime, root=root, window=win, same_screen=1, child=X.NONE,
        root_x=0, root_y=0, event_x=0, event_y=0, state=0, detail=keycode)
    win.send_event(press, event_mask=X.KeyPressMask, propagate=False)
    d.sync()
    time.sleep(hold)
    win.send_event(release, event_mask=X.KeyReleaseMask, propagate=False)
    d.sync()


def send_focus(d, win, keycode, hold):
    previous = d.get_input_focus().focus
    try:
        win.configure(stack_mode=X.Above)
    except Exception:
        pass
    d.set_input_focus(win, X.RevertToParent, X.CurrentTime)
    d.sync()
    focused = d.get_input_focus().focus
    if getattr(focused, "id", focused) != win.id:
        d.set_input_focus(previous, X.RevertToParent, X.CurrentTime)
        d.sync()
        raise RuntimeError("could not focus window 0x%08x" % win.id)
    try:
        xtest.fake_input(d, X.KeyPress, keycode)
        d.sync()
        time.sleep(hold)
        xtest.fake_input(d, X.KeyRelease, keycode)
        d.sync()
    finally:
        d.set_input_focus(previous, X.RevertToParent, X.CurrentTime)
        d.sync()


def parse_keys(raw):
    tokens = [t for t in (part.strip() for part in raw.split(",")) if t]
    if not tokens:
        print("press_keys: --keys is empty", file=sys.stderr)
        sys.exit(2)
    return tokens


def build_parser():
    parser = argparse.ArgumentParser(
        description="Send keys to a running X11 window (Genetic Car viewer).")
    parser.add_argument("--window", default=DEFAULT_TITLE,
                        help="substring of the window title; empty disables the title filter")
    parser.add_argument("--pid", type=int, default=None,
                        help="match the window owning this process id (_NET_WM_PID)")
    parser.add_argument("--keys", default=None,
                        help="comma-separated keys, e.g. G,V")
    parser.add_argument("--delay", type=float, default=0.3,
                        help="seconds between keys")
    parser.add_argument("--hold", type=float, default=0.03,
                        help="seconds a key stays pressed")
    parser.add_argument("--wait", type=float, default=0.0,
                        help="seconds to wait for the window to appear")
    parser.add_argument("--focus", action="store_true",
                        help="use XTEST with focus save/restore instead of XSendEvent")
    parser.add_argument("--list-windows", action="store_true",
                        help="list candidate windows and exit")
    return parser


def main(argv):
    args = build_parser().parse_args(argv)
    d = display.Display()

    if args.list_windows:
        for win, title, pid in list_windows(d):
            print("0x%08x pid=%s title=%r" % (win.id, pid, title))
        return 0

    if not args.keys:
        print("press_keys: --keys is required", file=sys.stderr)
        return 2

    tokens = parse_keys(args.keys)
    keys = []
    for token in tokens:
        keysym = keysym_for(token)
        if keysym is None:
            print("press_keys: unknown key %r" % token, file=sys.stderr)
            return 2
        keycode = d.keysym_to_keycode(keysym)
        if keycode == 0:
            print("press_keys: no keycode for %r" % token, file=sys.stderr)
            return 2
        keys.append((token, keycode))

    try:
        win, title, pid = resolve_target(d, args)
    except WindowNotFound as exc:
        print(str(exc), file=sys.stderr)
        return 3

    root = d.screen().root
    print("press_keys: target 0x%08x pid=%s title=%r (%s)" % (
        win.id, pid, title, "focus/XTEST" if args.focus else "XSendEvent"))

    for index, (token, keycode) in enumerate(keys):
        try:
            if args.focus:
                send_focus(d, win, keycode, args.hold)
            else:
                send_xsend(d, root, win, keycode, args.hold)
        except RuntimeError as exc:
            print("press_keys: %s" % exc, file=sys.stderr)
            return 4
        print("sent %s (keycode %d) to 0x%08x" % (token, keycode, win.id))
        if index + 1 < len(keys):
            time.sleep(max(0.0, args.delay))

    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
