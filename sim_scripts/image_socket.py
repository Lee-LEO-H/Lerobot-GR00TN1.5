import socket
import numpy as np
import cv2
import struct
import time

class ImageSender:
    """
    配置socket端口，发送np.uint8图像数组的类。
    """
    def __init__(self, host='127.0.0.1', port=64532):
        self.host = host
        self.port = port
        self.client_socket = None
        
        # 创建一个TCP/IP socket
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        
        # 尝试连接到接收端
        try:
            print(f"尝试连接到 {self.host}:{self.port}...")
            self.socket.connect((self.host, self.port))
            print("连接成功！")
        except socket.error as e:
            print(f"连接失败: {e}")
            self.socket.close()
            self.socket = None

    def send_image(self, image_array):
        """
        输入 np.uint8 类型的图像数组，将其发送到配置的端口。
        """
        if self.socket is None:
            print("Socket未连接，无法发送。")
            return False

        try:
            # 1. 对图像进行JPEG编码以压缩和序列化
            # 'image_array' 预期为 np.uint8 类型的 BGR 图像 (H, W, 3)
            # 编码参数可以调整，例如 cv2.IMWRITE_JPEG_QUALITY=95
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
            # 'result' 是一个布尔值, 'encoded_image' 是一个一维 np.uint8 数组 (字节流)
            result, encoded_image = cv2.imencode('.jpg', image_array, encode_param)

            if not result:
                print("图像编码失败。")
                return False

            # 将 numpy 数组转换为字节流
            data = encoded_image.tobytes()
            
            # 2. 准备数据大小 (以字节为单位)
            message_size = struct.pack("!L", len(data)) # !L 表示4字节无符号长整型，网络字节序

            # 3. 发送数据大小，然后发送图像数据
            self.socket.sendall(message_size + data)
            
            return True

        except socket.error as e:
            print(f"发送数据失败: {e}")
            self.close()
            return False
        except Exception as e:
            print(f"发送过程中发生错误: {e}")
            return False

    def close(self):
        """关闭连接"""
        if self.socket:
            print("关闭发送端socket。")
            self.socket.close()
            self.socket = None
class ImageReceiver:
    """
    配置socket端口，接收图像并使用 OpenCV 显示的类。
    """
    def __init__(self, host='127.0.0.1', port=64532):
        self.host = host
        self.port = port
        self.server_socket = None
        self.conn = None
        
        # 创建一个TCP/IP socket
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.bind((self.host, self.port))
        self.socket.listen(1)
        print(f"接收端启动，监听 {self.host}:{self.port}...")

    def _recvall(self, sock, count):
        """辅助函数：从socket接收指定字节数"""
        buf = b''
        while count:
            newbuf = sock.recv(count)
            if not newbuf:
                return None
            buf += newbuf
            count -= len(newbuf)
        return buf

    def start_receiving_and_display(self):
        """
        接收连接，并在一个循环中接收图像并显示。
        """
        try:
            # 接受一个连接
            self.conn, addr = self.socket.accept()
            print(f"接受连接自 {addr}")

            while True:
                # 1. 接收数据大小 (4字节)
                message_size_bytes = self._recvall(self.conn, 4)
                if not message_size_bytes:
                    print("连接断开或未收到数据大小。")
                    break
                    
                # 将字节流转换为整数
                message_size = struct.unpack("!L", message_size_bytes)[0]

                # 2. 接收图像数据
                data = self._recvall(self.conn, message_size)
                if data is None:
                    print("未收到图像数据。")
                    break

                # 3. 将字节流转换为 np.uint8 数组
                nparr = np.frombuffer(data, np.uint8)

                # 4. 使用 cv2.imdecode 解码图像
                # IMREAD_COLOR=1 确保它被解码为彩色图像
                image = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

                if image is None or image.size == 0:
                    print("图像解码失败或为空。")
                    continue
                
                # 5. 使用 OpenCV 显示图像
                image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                cv2.imshow('Received Image (640x480)', image)
                
                # 等待 1 毫秒，处理键盘事件
                # 按 'q' 键退出循环
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    print("用户退出。")
                    break

        except socket.error as e:
            print(f"接收过程中发生Socket错误: {e}")
        except Exception as e:
            print(f"发生错误: {e}")
        finally:
            self.close()

    def close(self):
        """关闭连接和socket"""
        if self.conn:
            self.conn.close()
        if self.socket:
            self.socket.close()
        cv2.destroyAllWindows()
        print("接收端关闭。")