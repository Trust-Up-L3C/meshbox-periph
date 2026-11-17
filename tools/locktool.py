#!/usr/bin/env python3
"""locktool.py -- BENCH tool for the MeshBox lock (design v1, milestone L1 dry run).

Sends one signed lock frame through a radio on USB and waits for the box's signed result.

usage: locktool.py PORT DEST open|cancel [--secret HEX64] [--counter N] [--tamper] [--wait SECS]
  PORT      serial port of the SENDING radio (e.g. COM7)
  DEST      the box, as '!xxxxxxxx' (also used as the box id inside the frame)
  --secret  64 hex chars. Default: the DUMMY bench secret 00..1f from the design document.
            Never provision a deployed box with the dummy secret.
  --counter default: current Unix time
  --tamper  flip one bit of the signature (the box must stay silent)
  --wait    seconds to wait for a result (default 20)

In Git Bash put DEST in single quotes: '!9716fb10'
"""
import hashlib
import hmac
import struct
import sys
import time

DUMMY_SECRET = bytes(range(32)).hex()
RESULTS = {1: "OPENED (lock pulsed / dry run)", 2: "CANCELLED (counter advanced, no pulse)", 3: "STALE (counter not newer, nothing done)"}
TYPES = {"open": 0x20, "cancel": 0x22}


def build(ftype, flags, box_id, counter, secret):
    body = bytes([1, ftype, flags]) + struct.pack("<I", box_id) + struct.pack("<I", counter)
    return body + hmac.new(secret, body, hashlib.sha256).digest()[:8]


def check(frame, box_id, secret):
    """Return (ok, result_code, counter) for a 19-byte result frame."""
    if len(frame) != 19 or frame[0] != 1 or frame[1] != 0x21:
        return False, None, None
    good = hmac.compare_digest(hmac.new(secret, frame[:11], hashlib.sha256).digest()[:8], frame[11:])
    fid, ctr = struct.unpack("<II", frame[3:11])
    return good and fid == box_id, frame[2], ctr


def fingerprint(secret_hex):
    return hashlib.sha256(secret_hex.lower().encode()).hexdigest()[:12]


def main(argv):
    args = list(argv)
    opt = {"--secret": DUMMY_SECRET, "--counter": None, "--wait": "20"}
    tamper = False
    if "--tamper" in args:
        tamper = True
        args.remove("--tamper")
    for k in list(opt):
        if k in args:
            i = args.index(k)
            opt[k] = args[i + 1]
            del args[i:i + 2]
    if len(args) != 3 or args[2] not in TYPES or not (args[1].startswith("!") and len(args[1]) == 9):
        sys.exit(__doc__)
    port, dest, what = args
    secret_hex = opt["--secret"]
    if len(secret_hex) != 64:
        sys.exit("secret must be 64 hex characters")
    secret = bytes.fromhex(secret_hex)
    box_id = int(dest[1:], 16)
    counter = int(opt["--counter"]) if opt["--counter"] is not None else int(time.time())
    frame = bytearray(build(TYPES[what], 0, box_id, counter, secret))
    if tamper:
        frame[18] ^= 0x01
    frame = bytes(frame)

    import meshtastic.serial_interface
    from pubsub import pub

    got = []

    def on_rx(packet, interface):
        d = packet.get("decoded", {})
        if str(d.get("portnum")) not in ("260",):
            return
        payload = bytes(d.get("payload", b""))
        frm = packet.get("fromId") or "!%08x" % packet.get("from", 0)
        if len(payload) == 19 and payload[1] == 0x21 and frm == dest:
            got.append(payload)

    pub.subscribe(on_rx, "meshtastic.receive")
    iface = meshtastic.serial_interface.SerialInterface(port)
    print("secret fingerprint", fingerprint(secret_hex), "(DUMMY bench secret)" if secret_hex == DUMMY_SECRET else "")
    print("sending %s%s to %s  counter=%d" % (what, " TAMPERED" if tamper else "", dest, counter))
    print("frame", frame.hex(" "))
    iface.sendData(frame, destinationId=dest, portNum=260, channelIndex=1)
    end = time.time() + float(opt["--wait"])
    while time.time() < end and not got:
        time.sleep(0.2)
    iface.close()
    if not got:
        print("NO RESULT within %s s" % opt["--wait"], "(expected for a tampered frame)" if tamper else "")
        return 2 if not tamper else 0
    ok, res, ctr = check(got[0], box_id, secret)
    print("result frame", got[0].hex(" "))
    print("signature", "VALID" if ok else "INVALID", "| result", res, RESULTS.get(res, "?"), "| counter", ctr, "(matches)" if ctr == counter else "(DOES NOT MATCH)")
    return 0 if ok and ctr == counter else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
