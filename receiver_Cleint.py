import socket
import struct
import numpy as np
import cv2
import sys
import time


def receive_image(conn):
    try:
        data_size = conn.recv(4)
        if not data_size:
            return None
        size = struct.unpack('!I', data_size)[0]

        data = b''
        while len(data) < size:
            packet = conn.recv(size - len(data))
            if not packet:
                return None
            data += packet

        image_np = np.frombuffer(data, dtype=np.uint8)
        image = cv2.imdecode(image_np, cv2.IMREAD_UNCHANGED)
        return image
    except Exception as e:
        print(f"[ERROR] receive_image: {e}")
        return None


def main():
    if len(sys.argv) != 4:
        print("Usage: python client_receiver.py <host_ip> <stream_name> <port>")
        print("Example: python client_receiver.py 192.168.50.112 Color 9001")
        return

    host_ip = sys.argv[1]
    stream_name = sys.argv[2]
    port = int(sys.argv[3])

    while True:
        try:
            print(f"[INFO] Connecting to {host_ip}:{port} for [{stream_name}]...")
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.settimeout(5.0)
                s.connect((host_ip, port))
                print(f"[INFO] Connected to {host_ip}:{port} for [{stream_name}]")

                while True:
                    image = receive_image(s)
                    if image is None:
                        print(f"[INFO] [{stream_name}] Disconnected from server.")
                        break

                    cv2.imshow(stream_name, image)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        print(f"[INFO] [{stream_name}] Quit signal received.")
                        return

                cv2.destroyWindow(stream_name)

        except Exception as e:
            print(f"[ERROR] [{stream_name}] Exception: {e}")

        print("[INFO] Reconnecting in 1 second...\n")
        time.sleep(1)


if __name__ == "__main__":
    main()
