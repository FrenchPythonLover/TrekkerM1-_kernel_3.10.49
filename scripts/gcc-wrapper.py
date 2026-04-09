#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import errno
import re
import os
import sys
import subprocess

allowed_warnings = {
    "return_address.c:63",
    "kprobes.c:1493",
    "rcutree.c:1614",
    "af_unix.c:893",
    "nl80211.c:58",
    "jhash.h:137",
    "cmpxchg.h:162",
    "ping.c:87",
}

ofile = None

warning_re = re.compile(r'''(.*/|)([^/]+\.[a-z]+:\d+):(\d+:)? warning:''')

def interpret_warning(line):
    """Decode gcc warnings and fail if not allowed"""
    global ofile

    line = line.rstrip('\n')
    m = warning_re.match(line)

    if m and m.group(2) not in allowed_warnings:
        print("error, forbidden warning:", m.group(2))

        if ofile:
            try:
                os.remove(ofile)
            except OSError:
                pass

        sys.exit(1)


def run_gcc():
    global ofile

    args = sys.argv[1:]

    try:
        i = args.index('-o')
        ofile = args[i + 1]
    except (ValueError, IndexError):
        pass

    try:
        proc = subprocess.Popen(
            args,
            stderr=subprocess.PIPE,
            text=True  # IMPORTANT: decode bytes → str automatiquement
        )

        for line in proc.stderr:
            print(line, end='')  # équivalent de print line,
            interpret_warning(line)

        result = proc.wait()

    except OSError as e:
        result = e.errno
        if result == errno.ENOENT:
            print(f"{args[0]}: {e.strerror}")
            print("Is your PATH set correctly?")
        else:
            print(' '.join(args), str(e))

    return result


if __name__ == '__main__':
    status = run_gcc()
    sys.exit(status)
