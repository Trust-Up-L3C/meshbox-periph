# usage: sendto.py PORT DEST pattern r g b secs [group] [--strip N]
#   DEST '^all' or '!nodeid'. With group given, sends cmd=3 (play-to-group, group 0 = all).
#   --strip N: 0 = both strips (default), 1 = strip 1, 2 = strip 2. Needs periph V2-A or later;
#   do not broadcast strip 1/2 while any box is still on M4.
import sys, time, meshtastic.serial_interface as m
args = sys.argv[1:]
strip = 0
if "--strip" in args:
    k = args.index("--strip")
    strip = int(args[k + 1])
    del args[k:k + 2]
if not 0 <= strip <= 2:
    sys.exit("strip must be 0, 1 or 2")
port, dest = args[0], args[1]
pat, r, g, b, secs = map(int, args[2:7])
if not 0 <= pat <= 6:
    sys.exit("pattern must be 0-6")
group = int(args[7]) if len(args) > 7 else None
p2 = (strip << 6) | pat
if group is None:
    frame = bytes([1, 1, p2, r, g, b, secs & 0xFF, (secs >> 8) & 0xFF])
else:
    frame = bytes([1, 3, p2, r, g, b, secs & 0xFF, group & 0xFF])
i = m.SerialInterface(port)
i.sendData(frame, destinationId=dest, portNum=260, channelIndex=1)
print("sent", list(frame), "to", dest)
time.sleep(5)
i.close()
