# usage: sendto.py PORT DEST pattern r g b secs [group]
#   DEST '^all' or '!nodeid'. With group given, sends cmd=3 (play-to-group, group 0 = all).
import sys, time, meshtastic.serial_interface as m
port, dest = sys.argv[1], sys.argv[2]
pat, r, g, b, secs = map(int, sys.argv[3:8])
group = int(sys.argv[8]) if len(sys.argv) > 8 else None
if group is None:
    frame = bytes([1, 1, pat, r, g, b, secs & 0xFF, (secs >> 8) & 0xFF])
else:
    frame = bytes([1, 3, pat, r, g, b, secs & 0xFF, group & 0xFF])
i = m.SerialInterface(port)
i.sendData(frame, destinationId=dest, portNum=260, channelIndex=1)
print("sent", list(frame), "to", dest)
time.sleep(5)
i.close()
