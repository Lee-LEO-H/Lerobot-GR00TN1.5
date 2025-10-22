import socket
import numpy as np
import cv2
import struct
import threading
import time
import json

class ObservationSender:
    """
    打包发送端，将关节数据和多个图像数据打包在一起同时发送
    """
    def __init__(self, host: str, port: int, articulation_names: list = None, image_names: list = None):
        self.host = host
        self.port = port
        
        # 设置默认关节名称
        if articulation_names is None:
            self.articulation_names = [
                "shoulder_pan",
                "shoulder_lift", 
                "elbow_flex",
                "wrist_flex",
                "wrist_roll", 
                "gripper"
            ]
        else:
            self.articulation_names = articulation_names
        
        # 设置默认图像名称（空列表）
        if image_names is None:
            self.image_names = []
        else:
            self.image_names = image_names
            
        self.camera_count = len(self.image_names)
        self.sock = None
        self._lock = threading.Lock()
        self._names_sent = False  # 标记名称是否已发送
        
        # 连接状态相关
        self._connection_start_time = None
        self._packets_sent = 0
        self._client_address = None
        
    def connect(self):
        """建立连接并发送名称列表"""
        try:
            with self._lock:
                if self.sock is None:
                    self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                    self.sock.connect((self.host, self.port))
                    
                    # 记录连接信息
                    self._connection_start_time = time.time()
                    self._packets_sent = 0
                    self._client_address = f"{self.host}:{self.port}"
                    
                    print(f"成功连接到 {self.host}:{self.port}")
                    
                    # 连接成功后立即发送名称列表
                    self._send_names()
                    
                return True
        except Exception as e:
            print(f"连接失败: {e}")
            with self._lock:
                self.sock = None
                self._names_sent = False
                self._connection_start_time = None
            return False
    
    def is_connected(self):
        """检查是否已连接"""
        with self._lock:
            if self.sock is None:
                return False
            
            # 通过尝试发送空数据来测试连接状态
            try:
                self.sock.getpeername()
                return True
            except (OSError, AttributeError):
                self.sock = None
                self._names_sent = False
                return False
    
    def get_connection_info(self):
        """获取连接信息"""
        with self._lock:
            if not self.is_connected():
                return None
            
            return {
                'client_address': self._client_address,
                'connection_duration': time.time() - self._connection_start_time if self._connection_start_time else 0,
                'packets_sent': self._packets_sent,
                'articulation_names': self.articulation_names,
                'image_names': self.image_names
            }
    
    def _send_names(self):
        """发送关节名称和图像名称列表"""
        try:
            # 数据包类型标记: 1 表示名称列表数据
            packet_type = struct.pack('B', 1)
            
            # 序列化关节名称
            articulation_names_json = json.dumps(self.articulation_names).encode('utf-8')
            articulation_names_len = struct.pack('!I', len(articulation_names_json))
            
            # 序列化图像名称
            image_names_json = json.dumps(self.image_names).encode('utf-8')
            image_names_len = struct.pack('!I', len(image_names_json))
            
            # 组装名称数据包
            names_message = (packet_type + 
                           articulation_names_len + articulation_names_json +
                           image_names_len + image_names_json)
            
            self.sock.sendall(names_message)
            self._names_sent = True
            self._packets_sent += 1
            print(f"名称列表发送完成 - 关节: {self.articulation_names}, 图像: {self.image_names}")
            
        except Exception as e:
            print(f"发送名称列表失败: {e}")
            self._names_sent = False
            raise
    
    def send_packed_data(self, articulation: np.ndarray, images: list, 
                        image_quality: int = 90, timestamp: float = None) -> bool:
        """
        将关节数据和多个图像数据打包在一起发送
        """
        # 验证输入
        if len(images) != self.camera_count:
            print(f"错误: 期望 {self.camera_count} 个图像，但收到 {len(images)} 个")
            return False
            
        if len(articulation) != len(self.articulation_names):
            print(f"错误: 关节数据长度 {len(articulation)} 与关节名称数量 {len(self.articulation_names)} 不匹配")
            return False
            
        # 确保连接存在且名称已发送
        if self.sock is None or not self._names_sent:
            if not self.connect():
                return False
            
        if timestamp is None:
            timestamp = time.time()
            
        with self._lock:
            try:
                # 数据包类型标记: 2 表示数据包（不含名称）
                packet_type = struct.pack('B', 2)
                
                # 时间戳 (双精度浮点数，8字节)
                timestamp_bytes = struct.pack('d', timestamp)
                
                # 1. 序列化关节数据
                articulation_dtype_str = articulation.dtype.str.encode('utf-8')
                articulation_shape_str = str(articulation.shape).encode('utf-8')
                articulation_data_bytes = articulation.tobytes()
                
                # 关节数据包头
                art_dtype_len = struct.pack('!I', len(articulation_dtype_str))
                art_shape_len = struct.pack('!I', len(articulation_shape_str))
                art_data_len = struct.pack('!I', len(articulation_data_bytes))
                
                # 2. 序列化所有图像数据
                all_image_data = b''
                for i, image in enumerate(images):
                    if image is None:
                        print(f"错误: 第 {i} 个图像为 None")
                        return False
                        
                    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), image_quality]
                    result, encoded_image = cv2.imencode('.jpg', image, encode_param)
                    
                    if not result:
                        print(f"第 {i} 个图像编码失败")
                        return False
                        
                    image_data_bytes = encoded_image.tobytes()
                    image_data_len = struct.pack('!I', len(image_data_bytes))
                    
                    # 将单个图像数据添加到总数据中
                    all_image_data += image_data_len + image_data_bytes
                
                # 3. 组装完整数据包
                message = (packet_type + timestamp_bytes +
                        art_dtype_len + art_shape_len + art_data_len +
                        articulation_dtype_str + articulation_shape_str + articulation_data_bytes +
                        all_image_data)
                
                self.sock.sendall(message)
                self._packets_sent += 1
                print(f"数据发送完成 - 关节: {articulation.shape}, 图像数量: {len(images)}, 总大小: {len(message)} 字节")
                return True
            
            except (BrokenPipeError, ConnectionResetError, OSError) as e:
                print(f"发送失败，连接已断开: {e}")
                self.close()
                # 尝试重新连接并重发
                # if self.connect():
                #     return self.send_packed_data(articulation, images, image_quality, timestamp)
                return False

            except Exception as e:
                print(f"发送数据失败: {e}")
                return False
    
    def close(self):
        """关闭连接"""
        if self.sock:
            self.sock.close()
            self.sock = None
        self._names_sent = False
        self._connection_start_time = None
        self._packets_sent = 0
        self._client_address = None


class ObservationReceiver:
    """
    打包数据接收端，同时接收关节和多个图像数据
    """
    def __init__(self, host: str, port: int):
        self.host = host
        self.port = port
        self.server_socket = None
        
        # 名称存储（在连接时接收）
        self.articulation_names = None
        self.image_names = None
        self.camera_count = 0
        
        # 数据存储
        self._latest_articulation = None
        self._latest_images = None
        self._latest_timestamp = 0
        
        # 连接状态相关
        self._current_connection = None
        self._connection_start_time = None
        self._packets_received = 0
        self._client_address = None
        
        # 线程锁和回调
        self._data_lock = threading.Lock()
        self._data_callback = None
        self._names_callback = None
        
        # 控制标志
        self._running = False
        self._receiver_thread = None
    
    def is_connected(self):
        """检查是否有活跃的连接"""
        with self._data_lock:
            return self._current_connection is not None and self.articulation_names is not None
    
    def get_connection_info(self):
        """获取连接信息"""
        with self._data_lock:
            if not self.is_connected():
                return None
            
            return {
                'client_address': self._client_address,
                'connection_duration': time.time() - self._connection_start_time if self._connection_start_time else 0,
                'packets_received': self._packets_received,
                'articulation_names': self.articulation_names,
                'image_names': self.image_names,
                'camera_count': self.camera_count
            }
    
    def set_data_callback(self, callback):
        """设置数据到达回调函数"""
        self._data_callback = callback
    
    def set_names_callback(self, callback):
        """设置名称列表到达回调函数"""
        self._names_callback = callback
    
    def _recvall(self, sock, count):
        """确保接收指定长度的数据"""
        buf = b''
        while count:
            try:
                newbuf = sock.recv(count)
                if not newbuf:
                    return None
                buf += newbuf
                count -= len(newbuf)
            except (socket.timeout, ConnectionResetError):
                return None
        return buf
    
    def start_receiving(self):
        """开始接收数据"""
        if self._running:
            print("接收器已在运行")
            return
            
        self._running = True
        self._receiver_thread = threading.Thread(target=self._receive_loop)
        self._receiver_thread.daemon = True
        self._receiver_thread.start()
        print(f"打包数据接收器启动，监听 {self.host}:{self.port}")
    
    def _receive_loop(self):
        """接收循环"""
        while self._running:
            try:
                self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self.server_socket.bind((self.host, self.port))
                self.server_socket.listen(1)
                self.server_socket.settimeout(1.0)
                
                print("等待连接...")
                conn, addr = self.server_socket.accept()
                
                # 更新连接状态
                with self._data_lock:
                    self._current_connection = conn
                    self._connection_start_time = time.time()
                    self._packets_received = 0
                    self._client_address = f"{addr[0]}:{addr[1]}"
                
                print(f"接受来自 {addr} 的连接")
                
                # 重置名称状态
                self.articulation_names = None
                self.image_names = None
                self.camera_count = 0
                self._latest_images = None
                
                # 处理该连接的所有数据包
                self._handle_connection(conn)
                
            except socket.timeout:
                continue
            except Exception as e:
                if self._running:
                    print(f"接收循环错误: {e}")
                time.sleep(0.1)
            finally:
                if self.server_socket:
                    self.server_socket.close()
                
                # 清除连接状态
                with self._data_lock:
                    self._current_connection = None
                    self._connection_start_time = None
                    self._client_address = None
    
    def _handle_connection(self, conn):
        """处理单个连接 - 简化版本，永远等待"""
        conn.settimeout(1.0)
        print(f"开始处理连接，客户端: {self._client_address}")
        
        while self._running:
            try:
                # 接收数据包类型标记
                packet_type_bytes = self._recvall(conn, 1)
                if not packet_type_bytes:
                    # 没有数据，继续等待
                    continue
                    
                packet_type = struct.unpack('B', packet_type_bytes)[0]
                
                if packet_type == 1:  # 名称列表数据
                    print("接收到名称列表数据包")
                    self._receive_names_data(conn)
                elif packet_type == 2:  # 数据包（不含名称）
                    self._receive_packed_data(conn)
                else:
                    print(f"未知数据包类型: {packet_type}")
                    continue
                    
            except socket.timeout:
                # 正常超时，继续等待
                continue
                
            except (ConnectionResetError, BrokenPipeError):
                print("客户端主动断开连接")
                break
                
            except Exception as e:
                print(f"处理数据时发生错误: {e}")
                # 记录错误但继续等待
                time.sleep(0.1)
                continue
        
        print("连接处理结束")
    
    def _receive_names_data(self, conn):
        """接收名称列表数据"""
        try:
            # 1. 接收关节名称
            art_names_len_bytes = self._recvall(conn, 4)
            if not art_names_len_bytes:
                raise Exception("关节名称长度数据不完整")
            
            art_names_len = struct.unpack('!I', art_names_len_bytes)[0]
            art_names_json = self._recvall(conn, art_names_len)
            if not art_names_json:
                raise Exception("关节名称数据不完整")
            
            self.articulation_names = json.loads(art_names_json.decode('utf-8'))
            
            # 2. 接收图像名称
            img_names_len_bytes = self._recvall(conn, 4)
            if not img_names_len_bytes:
                raise Exception("图像名称长度数据不完整")
            
            img_names_len = struct.unpack('!I', img_names_len_bytes)[0]
            img_names_json = self._recvall(conn, img_names_len)
            if not img_names_json:
                raise Exception("图像名称数据不完整")
            
            self.image_names = json.loads(img_names_json.decode('utf-8'))
            self.camera_count = len(self.image_names)
            self._latest_images = [None] * self.camera_count
            
            print(f"名称列表接收完成 - 关节: {self.articulation_names}, 图像: {self.image_names}")
            
            # 调用名称回调函数
            if self._names_callback:
                self._names_callback(self.articulation_names, self.image_names)
                
        except Exception as e:
            print(f"接收名称列表错误: {e}")
    
    def _receive_packed_data(self, conn):
        """接收打包数据"""
        # 检查是否已接收名称列表
        if self.articulation_names is None or self.image_names is None:
            print("错误: 尚未接收名称列表，无法解析数据")
            return
            
        try:
            # 1. 接收时间戳
            timestamp_bytes = self._recvall(conn, 8)
            if not timestamp_bytes:
                raise Exception("时间戳数据不完整")
            
            timestamp = struct.unpack('d', timestamp_bytes)[0]
            
            # 2. 接收关节数据段
            # 关节数据包头
            art_header_bytes = self._recvall(conn, 12)
            if not art_header_bytes or len(art_header_bytes) < 12:
                raise Exception("关节数据包头不完整")
            
            art_dtype_len, art_shape_len, art_data_len = struct.unpack('!III', art_header_bytes)
            
            # 接收关节数据
            art_dtype_bytes = self._recvall(conn, art_dtype_len)
            art_shape_bytes = self._recvall(conn, art_shape_len)
            art_data_bytes = self._recvall(conn, art_data_len)
            
            if not all([art_dtype_bytes, art_shape_bytes, art_data_bytes]):
                raise Exception("关节数据不完整")
            
            # 重构关节数组
            art_dtype_str = art_dtype_bytes.decode('utf-8')
            art_shape_str = art_shape_bytes.decode('utf-8')
            
            art_shape_parts = [p.strip() for p in art_shape_str.strip('()').split(',') if p.strip()]
            if not art_shape_parts:
                raise ValueError(f"无效的关节形状字符串: '{art_shape_str}'")
            
            art_shape = tuple(map(int, art_shape_parts))
            articulation_data = np.frombuffer(art_data_bytes, dtype=art_dtype_str).reshape(art_shape)
            
            # 3. 接收所有图像数据
            images_data = []
            for i in range(self.camera_count):
                # 图像数据长度
                image_len_bytes = self._recvall(conn, 4)
                if not image_len_bytes:
                    raise Exception(f"第 {i} 个图像数据长度不完整")
                
                image_data_len = struct.unpack('!I', image_len_bytes)[0]
                
                # 接收图像数据
                image_data_bytes = self._recvall(conn, image_data_len)
                if not image_data_bytes or len(image_data_bytes) < image_data_len:
                    raise Exception(f"第 {i} 个图像数据不完整")
                
                # 解码图像
                nparr = np.frombuffer(image_data_bytes, np.uint8)
                image_data = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                
                if image_data is None or image_data.size == 0:
                    raise Exception(f"第 {i} 个图像解码失败")
                
                images_data.append(image_data)
            
            # 4. 更新数据
            with self._data_lock:
                self._latest_articulation = articulation_data
                self._latest_images = images_data
                self._latest_timestamp = timestamp
                self._packets_received += 1
            
            # 5. 调用回调函数
            if self._data_callback:
                self._data_callback(articulation_data, images_data, timestamp)
            
            # print(f"数据接收完成 - 时间: {timestamp:.3f}, 关节: {articulation_data.shape}, 图像数量: {len(images_data)}")
                
        except Exception as e:
            print(f"接收数据错误: {e}")
    
    def get_latest_data(self):
        """获取最新的所有数据"""
        with self._data_lock:
            return {
                'articulation': self._latest_articulation.copy() if self._latest_articulation is not None else None,
                'images': [img.copy() if img is not None else None for img in self._latest_images] if self._latest_images is not None else None,
                'articulation_names': self.articulation_names,
                'image_names': self.image_names,
                'timestamp': self._latest_timestamp
            }
    
    def get_names(self):
        """获取名称列表"""
        return {
            'articulation_names': self.articulation_names,
            'image_names': self.image_names
        }
    
    def stop(self):
        """停止接收"""
        self._running = False
        
        # 关闭当前连接
        with self._data_lock:
            if self._current_connection:
                self._current_connection.close()
                self._current_connection = None
        
        # 关闭服务器socket
        if self.server_socket:
            self.server_socket.close()
        
        # 唤醒阻塞的accept
        try:
            wake_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            wake_socket.connect((self.host, self.port))
            wake_socket.close()
        except:
            pass
        
        if self._receiver_thread and self._receiver_thread.is_alive():
            self._receiver_thread.join(timeout=2.0)
        
        print("打包数据接收器已停止")