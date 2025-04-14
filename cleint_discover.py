import socket
import time

DISCOVERY_PORT = 9999
DISCOVERY_MSG = "WHERE_ARE_YOU?"
RESPONSE_MSG = "I_AM_HERE"
TIMEOUT_SEC = 2


def discover_server_ip():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        sock.settimeout(TIMEOUT_SEC)

        # 廣播地址為 255.255.255.255
        broadcast_address = ('255.255.255.255', DISCOVERY_PORT)

        # 發送廣播訊息
        sock.sendto(DISCOVERY_MSG.encode(), broadcast_address)

        try:
            # 接收回應
            data, addr = sock.recvfrom(1024)
            if data.decode() == RESPONSE_MSG:
                print(f"✅ Server 回應來自：{addr[0]}")
                return addr[0]
            else:
                print("❌ 收到未知的回應：", data.decode())
        except socket.timeout:
            print("⌛ 沒有收到 Server 回應（可能不在同一網段或 Broadcast 被封鎖）")
            return None


if __name__ == "__main__":
    server_ip = discover_server_ip()
    if server_ip:
        print("✔️ 你可以用這個 IP 連接 TCP Server:", server_ip)
    else:
        print("⚠️ 無法找到 Server")
