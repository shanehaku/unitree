import socket
import struct
import numpy as np
import cv2


def receive_image(conn):
    # Step 1: 先讀取資料大小（int, 4 bytes）
    data_size = conn.recv(4)
    if not data_size:
        return None
    size = struct.unpack('!I', data_size)[0]

    # Step 2: 接收影像資料
    data = b''
    while len(data) < size:
        packet = conn.recv(size - len(data))
        if not packet:
            return None
        data += packet

    # Step 3: 轉成 numpy array 並解碼
    image_np = np.frombuffer(data, dtype=np.uint8)
    image = cv2.imdecode(image_np, cv2.IMREAD_UNCHANGED)
    return image


def run_receiver(port, window_name="image"):
    HOST = '0.0.0.0'  # 接收所有 IP
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST, port))
        s.listen(1)
        print(f"[INFO] Listening on port {port} ...")
        conn, addr = s.accept()
        print(f"[INFO] Connection from {addr}")

        with conn:
            while True:
                image = receive_image(conn)
                if image is None:
                    print("[INFO] No image received, connection might be closed.")
                    break

                cv2.imshow(window_name, image)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    print("[INFO] Quit.")
                    break
            cv2.destroyAllWindows()
