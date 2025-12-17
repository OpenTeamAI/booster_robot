import time
import sys
from booster_robotics_sdk_python import B1LocoClient, ChannelFactory, GetModeResponse

nic = "127.0.0.1"
domain = 0

ChannelFactory.Instance().Init(domain, nic)
print(f"DDS init ok domain={domain} nic={nic}")

client = B1LocoClient()
client.Init()  # 先用无参
print("client Init() ok")

last = None
for i in range(12):
    resp = GetModeResponse()
    rc = client.GetMode(resp)
    if rc == 0:
        print("mode =", resp.mode)
        break
    last = rc
    time.sleep(0.25)
else:
    raise RuntimeError(f"GetMode timeout: err={last} (per T1 manual, 100 = RPC timeout)")
