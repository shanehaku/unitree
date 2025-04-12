import socket
import struct
import numpy as np
import cv2
import threading
import queue

# 初始化 Queue，讓每個串流畫面各自管理
image_queues = {
    "Color": queue.Queue(),
    "Depth": queue.Queue(),
    "IR_Left": queue.Queue(),
    "IR_Right": queue.Queue(),
    "Post_Depth": queue.Queue()
}


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
    HOST = '0.0.0.0'
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((HOST, port))
        s.listen(1)
        print(f"[INFO] Listening on port {port} ({window_name}) ...")
        conn, addr = s.accept()
        print(f"[INFO] Connection from {addr} on port {port}")

        with conn:
            print(f"[INFO] [{window_name}] Connection.")
            while True:
                image = receive_image(conn)
                if image is None:
                    print(f"[INFO] [{window_name}] Disconnected.")
                    break
                # 將影像放進對應 queue
                image_queues[window_name].put(image)


def main():
    stream_ports = {
        "Color": 9001,
        "Depth": 9002,
        "IR_Left": 9003,
        "IR_Right": 9004,
        "Post_Depth": 9005
    }

    # 啟動每個執行緒，負責接收不同串流
    for name, port in stream_ports.items():
        t = threading.Thread(target=run_receiver_thread, args=(port, name), daemon=True)
        t.start()

    print("[INFO] Main loop started. Displaying images...")

    # 主執行緒負責顯示畫面
    while True:
        for name in stream_ports.keys():
            if not image_queues[name].empty():
                image = image_queues[name].get()
                if image is not None:
                    cv2.imshow(name, image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("[INFO] Quit signal received.")
            break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()