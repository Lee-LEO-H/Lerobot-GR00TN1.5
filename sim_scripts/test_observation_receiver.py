from observation_socket import ObservationReceiver
import time
import cv2

HOST = '127.0.0.1'
PORT = 65435

def on_data_received(articulation, images, timestamp):
    print(f"回调: 时间 {timestamp:.3f}, 关节 {articulation.shape}, 图像数量: {len(images)}")
    print(f" 最新动作 -> {articulation}")

    # 显示图像
    for i, image in enumerate(images):
        if image is not None and image.size > 0:
            cv2.imshow(f"image {i}", image)
    cv2.waitKey(1)  

if __name__ == "__main__":
    receiver = ObservationReceiver(HOST, PORT)
    receiver.set_data_callback(on_data_received)
    
    try:
        receiver.start_receiving()
        print(f"接收端启动，监听 {HOST}:{PORT}...")
        # 等待用户按 Ctrl+C
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\n正在关闭接收器...")
    finally:
        receiver.close()
        cv2.destroyAllWindows()
        print("接收端关闭。")