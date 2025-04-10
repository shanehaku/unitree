import socket
import numpy as np
import cv2

# Set up the UDP socket to receive data





def setup_udp_socket(port):
    sockfd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sockfd.bind(('0.0.0.0', port))  # Bind to all available interfaces
    return sockfd

# Receive and display frames


def receive_and_show_frames(sockfd):
    buffer_size = 65536  # The buffer size for the received data (adjust as needed)

    while True:
        # Receive data from the socket
        data, addr = sockfd.recvfrom(buffer_size)

        # Convert the received bytes into a numpy array (image)
        image = np.frombuffer(data, dtype=np.uint8).reshape((480, 640, 3))  # Assuming 640x480 color images

        if image.size > 0:
            # Display the image using OpenCV
            cv2.imshow("Received Image", image)

# Main function to run the server


def main():
    port = 8080
    sockfd = setup_udp_socket(port)
    fourier_address = ('192.168.50.80', port)
    receive_and_show_frames(sockfd)
    sockfd.close()


if __name__ == "__main__":
    main()
