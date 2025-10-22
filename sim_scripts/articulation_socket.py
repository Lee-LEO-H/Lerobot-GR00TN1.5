# import socket
# import numpy as np
# import struct

# # -----------------
# # 1. 数组发送类 (Client)
# # -----------------
# class ArticulationSender:
#     """
#     配置目标IP和端口，负责将Numpy数组发送出去。
#     """
#     def __init__(self, host: str, port: int):
#         self.host = host
#         self.port = port
#         self.sock = None

#     def send_array(self, array: np.ndarray):
#         """
#         输入一个 NumPy 数组 (推荐 np.float32 或 np.float64)，将其发送到配置的端口。

#         发送的数据包结构:
#         [4字节: 数据类型长度] + [4字节: 数组形状长度] + [N字节: 数据类型信息] + [M字节: 数组形状信息] + [K字节: 数组的原始字节数据]
#         """
#         if array.dtype not in [np.float32, np.float64]:
#             print(f"警告：数组类型为 {array.dtype}，可能不兼容。建议使用 np.float32/64。")

#         try:
#             # 建立连接
#             self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#             self.sock.connect((self.host, self.port))
#             print(f"成功连接到 {self.host}:{self.port}，开始发送数组...")

#             # 1. 序列化数组信息 (数据类型, 形状, 原始数据)
            
#             # 数据类型字符串 (e.g., 'float32')
#             dtype_str = array.dtype.str.encode('utf-8')
#             # 数组形状的序列化字符串 (e.g., '(100, 10)')
#             shape_str = str(array.shape).encode('utf-8')
#             # 数组的原始字节数据
#             data_bytes = array.tobytes()

#             # 2. 构造包头: 数据类型长度 (4字节), 形状长度 (4字节), 数据长度 (4字节)
#             # 使用 !I 格式 (网络字节序，无符号整数) 确保跨平台兼容
#             dtype_len_header = struct.pack('!I', len(dtype_str))
#             shape_len_header = struct.pack('!I', len(shape_str))
#             data_len_header = struct.pack('!I', len(data_bytes))

#             # 3. 组装并发送所有数据
#             message = dtype_len_header + shape_len_header + data_len_header + dtype_str + shape_str + data_bytes
#             self.sock.sendall(message)
            
#             print(f"发送完成。数组形状: {array.shape}，总字节数: {len(message)}")

#         except ConnectionRefusedError:
#             print(f"错误：无法连接到 {self.host}:{self.port}，请确保接收端已启动。")
#         except Exception as e:
#             print(f"发送过程中发生错误: {e}")
#         finally:
#             if self.sock:
#                 self.sock.close()


# # -----------------
# # 2. 数组接收类 (Server)
# # -----------------
# class ArticulationReceiver:
#     """
#     配置端口，负责监听和接收Numpy数组。
#     """
#     def __init__(self, host: str, port: int):
#         self.host = host
#         self.port = port
#         self.server_socket = None

#     def receive_array(self) -> np.ndarray:
#         """
#         监听端口并等待接收一个 NumPy 数组，成功接收后返回该数组。
#         """
#         # 内部函数：确保从 socket 接收指定长度的字节
#         def recvall(sock, count):
#             buf = b''
#             while count:
#                 newbuf = sock.recv(count)
#                 if not newbuf:
#                     return None
#                 buf += newbuf
#                 count -= len(newbuf)
#             return buf

#         try:
#             # 初始化 Socket
#             self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#             self.server_socket.bind((self.host, self.port))
#             self.server_socket.listen(1)
#             print(f"接收端启动，正在监听 {self.host}:{self.port}...")
            
#             # 接受连接
#             conn, addr = self.server_socket.accept()
#             print(f"已接受来自 {addr} 的连接。")

#             # 1. 接收包头 (总共 4*3 = 12 字节的长度信息)
#             header_bytes = recvall(conn, 12)
#             if header_bytes is None or len(header_bytes) < 12:
#                 raise Exception("未接收到完整的包头长度信息。")

#             # 2. 解析包头: 字节长度
#             dtype_len, shape_len, data_len = struct.unpack('!III', header_bytes)
#             print(f"等待接收的数据类型长度: {dtype_len}，形状长度: {shape_len}，数组数据长度: {data_len} 字节。")
            
#             # 3. 接收并解析数组信息
#             # 接收数据类型字符串
#             dtype_bytes = recvall(conn, dtype_len)
#             dtype_str = dtype_bytes.decode('utf-8')
            
#             # 接收数组形状字符串
#             shape_bytes = recvall(conn, shape_len)
#             shape_str = shape_bytes.decode('utf-8')
            
#             # 4. 接收原始数组数据
#             data_bytes = recvall(conn, data_len)
#             if data_bytes is None or len(data_bytes) < data_len:
#                 raise Exception("未接收到完整的数组数据。")
            
#             # 5. 重构 NumPy 数组
#             # 解析形状字符串为 tuple
#             shape = tuple(map(int, shape_str.strip('()').split(',')))
            
#             # 从字节和类型信息重构数组
#             received_array = np.frombuffer(data_bytes, dtype=dtype_str).reshape(shape)

#             print(f"成功接收数据。重构数组形状: {received_array.shape}，类型: {received_array.dtype}")
#             return received_array

#         except Exception as e:
#             print(f"接收过程中发生错误: {e}")
#             return np.array([])  # 返回一个空数组表示失败
#         finally:
#             if self.server_socket:
#                 self.server_socket.close()
#             if 'conn' in locals() and conn:
#                 conn.close()
import socket
import numpy as np
import struct
import threading
import time

# -----------------
# 辅助函数：确保接收到所有字节
# -----------------
def recvall(sock, count):
    """确保从 socket 接收指定长度的字节"""
    buf = b''
    while count:
        # 使用 select/poll/epoll 机制来非阻塞地等待数据
        # 在线程环境中，简单的循环 recv 配合短 sleep 也是可行的，
        # 但这里我们依赖外部的 connect/close 机制来同步数据包。
        try:
            newbuf = sock.recv(count)
        except socket.timeout:
            return None # 接收超时或连接断开
        except ConnectionResetError:
            print("警告: 连接被重置 (发送端已关闭)")
            return None

        if not newbuf:
            return None
        buf += newbuf
        count -= len(newbuf)
    return buf

# -----------------
# 1. 数组发送类 (Client) - 保持不变
# -----------------
class ArticulationSender:
    """
    配置目标IP和端口，负责将Numpy数组发送出去。
    每次调用 send_array 都会建立新的连接并关闭。
    """
    def __init__(self, host: str, port: int):
        self.host = host
        self.port = port
        self.sock = None

    def send_array(self, array: np.ndarray):
        """发送 NumPy 数组"""
        if array.dtype not in [np.float32, np.float64]:
            print(f"警告：数组类型为 {array.dtype}，可能不兼容。建议使用 np.float32/64。")

        try:
            # 建立连接
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) # 允许重用地址
            self.sock.connect((self.host, self.port))
            # print(f"成功连接到 {self.host}:{self.port}，开始发送数组...")

            # 1. 序列化数组信息 (数据类型, 形状, 原始数据)
            dtype_str = array.dtype.str.encode('utf-8')
            shape_str = str(array.shape).encode('utf-8')
            data_bytes = array.tobytes()

            # 2. 构造包头: 字节长度
            dtype_len_header = struct.pack('!I', len(dtype_str))
            shape_len_header = struct.pack('!I', len(shape_str))
            data_len_header = struct.pack('!I', len(data_bytes))

            # 3. 组装并发送所有数据
            message = dtype_len_header + shape_len_header + data_len_header + dtype_str + shape_str + data_bytes
            self.sock.sendall(message)
            
            # print(f"发送完成。数组形状: {array.shape}，总字节数: {len(message)}")

        except ConnectionRefusedError:
            print(f"错误：无法连接到 {self.host}:{self.port}，请确保接收端已启动。")
        except Exception as e:
            print(f"发送过程中发生错误: {e}")
        finally:
            if self.sock:
                self.sock.close()
    def close(self):
        """关闭发送器（兼容性方法，实际上不需要做任何事情）"""
        # ArticulationSender 每次发送都会新建连接并自动关闭
        # 所以这个方法只是为了兼容接口，不需要实际实现
        pass


# -----------------
# 2. 线程化的数组接收类 (Server)
# -----------------
class ActionReceiverThread(threading.Thread):
    """
    一个后台线程，持续监听并接收 NumPy 动作数组，不会阻塞主程序。
    """
    def __init__(self, host: str, port: int, num_dof: int):
        super().__init__()
        self.host = host
        self.port = port
        self.num_dof = num_dof
        
        # 存储最新的动作数据，初始化为零动作
        self._latest_action = np.zeros(self.num_dof, dtype=np.float32)
        
        # 线程锁，确保主程序和接收线程安全地访问数据
        self._lock = threading.Lock()
        
        # 线程控制标志
        self._running = True

    def get_latest_action(self) -> np.ndarray:
        """供主程序调用的方法，安全地获取最新的动作数组。"""
        with self._lock:
            # 返回数据的副本，防止主程序修改共享数据
            return self._latest_action.copy()

    def stop(self):
        """安全停止线程的执行。"""
        self._running = False
        # 为了让阻塞在 accept() 的 socket 退出，可以尝试连接再关闭
        try:
            socket.socket(socket.AF_INET, socket.SOCK_STREAM).connect((self.host, self.port))
        except:
            pass # 忽略连接失败，目的只是唤醒 accept()

    def _process_single_reception(self, conn) -> np.ndarray:
        """处理单个连接中的一次数据接收和解析"""
        
        # 1. 接收包头 (总共 12 字节的长度信息)
        header_bytes = recvall(conn, 12)
        if header_bytes is None or len(header_bytes) < 12:
            raise Exception("未接收到完整的包头长度信息或连接中断。")

        # 2. 解析包头: 字节长度
        dtype_len, shape_len, data_len = struct.unpack('!III', header_bytes)
        
        # 3. 接收并解析数组信息
        dtype_bytes = recvall(conn, dtype_len)
        dtype_str = dtype_bytes.decode('utf-8')
        
        shape_bytes = recvall(conn, shape_len)
        shape_str = shape_bytes.decode('utf-8')
        
        # 4. 接收原始数组数据
        data_bytes = recvall(conn, data_len)
        if data_bytes is None or len(data_bytes) < data_len:
            raise Exception("未接收到完整的数组数据。")
        
        # 5. 重构 NumPy 数组
        # 修复了 'invalid literal' 错误可能导致的问题：确保形状字符串是合法的
        shape_parts = [p.strip() for p in shape_str.strip('()').split(',') if p.strip()]
        if not shape_parts:
             raise ValueError(f"无法解析形状字符串: '{shape_str}'")

        shape = tuple(map(int, shape_parts))
        
        # 从字节和类型信息重构数组
        received_array = np.frombuffer(data_bytes, dtype=dtype_str).reshape(shape)

        return received_array

    def run(self):
        """线程的主执行循环，持续监听连接"""
        print(f"动作接收线程启动，正在持续监听 {self.host}:{self.port}...")

        while self._running:
            server_socket = None
            conn = None
            try:
                # 1. 初始化 Socket
                server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                server_socket.bind((self.host, self.port))
                
                # 设置超时以避免无限期阻塞，但主要依赖 accept() 来阻塞
                # server_socket.settimeout(0.5) 

                server_socket.listen(1)
                
                # 2. 接受连接 (阻塞点)
                conn, addr = server_socket.accept()
                if not self._running: break # 如果在 accept 过程中被 stop() 唤醒
                # print(f"接收线程：已接受来自 {addr} 的连接。")

                # 3. 处理数据接收
                received_array = self._process_single_reception(conn)
                
                # 4. 线程安全地更新数据
                if received_array.size == self.num_dof:
                    with self._lock:
                        self._latest_action = received_array
                    # print(f"接收线程：成功更新动作 (DOF: {received_array.size})")
                else:
                    pass
                    # print(f"接收线程：警告，接收到错误的自由度数量 ({received_array.size} != {self.num_dof})")

            except Exception as e:
                if self._running:
                    # 只有在线程仍然运行时才打印错误，忽略因 stop() 导致的连接错误
                    print(f"接收线程错误: {e}")
                time.sleep(0.1) # 休息一下，避免在错误中无限循环占用CPU

            finally:
                # 确保在连接/服务器关闭
                if conn: conn.close()
                if server_socket: server_socket.close()
