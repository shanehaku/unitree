import socket

def is_valid_action(action):
    # 检查输入是否是数字且在1到100之间
    if action.isdigit():
        number = int(action)
        return 1 <= number <= 100
    return False

client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_address1 = ("192.168.50.80", 8001)

while True:
    action = input("请输入要发送的动作（1-100之间的数字）：")
    
    if not is_valid_action(action):
        print("动作类型错误，请输入1到100之间的数字。")
        print("-----------------------------")
        continue
    
    msg = f'{{"arm_mode":{action}}}'
    print(f'发送消息: {msg} 到地址: {server_address1}')

    client_socket.sendto(msg.encode(), server_address1)