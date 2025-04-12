import socket
import struct
import numpy as np
import cv2
import threading


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


def run_receiver_thread(port, window_name):
    HOST = '192.168.50.80'
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((HOST, port))
        s.listen(1)
        print(f"[INFO] Listening on port {port} ({window_name}) ...")
        conn, addr = s.accept()
        print(f"[INFO] Connection from {addr} on port {port}")

        with conn:
            while True:
                image = receive_image(conn)
                if image is None:
                    print(f"[INFO] [{window_name}] Disconnected.")
                    break

                cv2.imshow(window_name, image)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    print(f"[INFO] [{window_name}] Quit signal.")
                    break
        cv2.destroyWindow(window_name)


def main():
    stream_ports = {
        "Color": 9001,
        "Depth": 9002,
        "IR_Left": 9003,
        "IR_Right": 9004,
        "Post_Depth": 9005
    }

    threads = []
    for name, port in stream_ports.items():
        t = threading.Thread(target=run_receiver_thread, args=(port, name))
        t.start()
        threads.append(t)

    # 等待所有 thread 結束
    for t in threads:
        t.join()


if __name__ == "__main__":
    main()
