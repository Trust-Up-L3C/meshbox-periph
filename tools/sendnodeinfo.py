import sys, time, meshtastic.serial_interface as m
from meshtastic import portnums_pb2, mesh_pb2
from google.protobuf.json_format import ParseDict
i = m.SerialInterface(sys.argv[1])
u = ParseDict(i.getMyUser(), mesh_pb2.User())
i.sendData(u, destinationId='^all', portNum=portnums_pb2.PortNum.NODEINFO_APP, channelIndex=0)
print("sent NodeInfo for", u.id)
time.sleep(5)
i.close()
