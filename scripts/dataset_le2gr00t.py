import os
import sys
import argparse
import json
import shutil
import subprocess
from glob import glob

def get_set_dir(args):
    if args.set_dir:
        set_dir = os.path.abspath(args.set_dir)
    else:
        set_dir = os.path.dirname(os.path.abspath(__file__))
        print(f"\033[33m未指定 --set-dir，使用当前脚本所在目录：{set_dir}\033[0m")
    if not os.path.exists(set_dir):
        print(f"\033[31m目录不存在: {set_dir}\033[0m")
        sys.exit(1)
    return set_dir

def check_dataset_structure(set_dir):
    required = [
        os.path.join(set_dir, "meta", "info.json"),
        os.path.join(set_dir, "videos"),
        os.path.join(set_dir, "data"),
        os.path.join(set_dir, "meta", "episodes"),
    ]
    for path in required:
        if not os.path.exists(path):
            return False, f"缺少 {path}"
    return True, "必需文件齐全"

def reorganize_videos(set_dir):
    videos_dir = os.path.join(set_dir, "videos")
    if not os.path.exists(videos_dir):
        return False, f"未找到 {videos_dir}"
    try:
        moves = 0
        # 找到所有 observation.images.xxx
        for obs_dir in os.listdir(videos_dir):
            obs_path = os.path.join(videos_dir, obs_dir)
            if not (os.path.isdir(obs_path) and obs_dir.startswith("observation.images.")):
                continue
            # 只处理 observation.images.xxx/chunk-000/
            for chunk in os.listdir(obs_path):
                chunk_path = os.path.join(obs_path, chunk)
                if not os.path.isdir(chunk_path):
                    continue
                # 目标目录 videos/chunk-000/observation.images.xxx/
                target_dir = os.path.join(videos_dir, chunk, obs_dir)
                os.makedirs(target_dir, exist_ok=True)
                for fname in os.listdir(chunk_path):
                    src = os.path.join(chunk_path, fname)
                    dst = os.path.join(target_dir, fname)
                    shutil.move(src, dst)
                    moves += 1
                # 移动后删除空目录
                os.rmdir(chunk_path)
            # 删除 observation.images.xxx 空目录
            if not os.listdir(obs_path):
                os.rmdir(obs_path)
        detail = "无文件需要移动" if moves == 0 else f"已整理 {moves} 个文件"
        return True, detail
    except Exception as e:
        return False, str(e)

def fix_info_json(set_dir):
    info_path = os.path.join(set_dir, "meta", "info.json")
    if not os.path.exists(info_path):
        return False, f"未找到 {info_path}"
    try:
        with open(info_path, "r", encoding="utf-8") as f:
            info = json.load(f)
        info["data_path"] = "data/chunk-{episode_chunk:03d}/file-{episode_index:03d}.parquet"
        info["video_path"] = "videos/chunk-{episode_chunk:03d}/{video_key}/file-{episode_index:03d}.mp4"
        with open(info_path, "w", encoding="utf-8") as f:
            json.dump(info, f, indent=2, ensure_ascii=False)
        return True, "data_path / video_path 已更新"
    except Exception as e:
        return False, str(e)

def convert_episodes_to_jsonl(set_dir):
    try:
        import pandas as pd
    except ImportError:
        return False, "缺少 pandas,请先安装"
    meta_dir = os.path.join(set_dir, "meta")
    episodes_dir = os.path.join(meta_dir, "episodes", "chunk-000")
    output_jsonl = os.path.join(meta_dir, "episodes.jsonl")
    parquet_files = sorted(glob(os.path.join(episodes_dir, "*.parquet")))
    if not parquet_files:
        return False, f"未找到 {episodes_dir} 下的 parquet 文件"
    try:
        with open(output_jsonl, "w", encoding="utf-8") as fout:
            for pf in parquet_files:
                df = pd.read_parquet(pf)
                row = df.iloc[0]
                record = {
                    "episode_index": int(row["episode_index"]),
                    "tasks": row["tasks"].tolist() if hasattr(row["tasks"], "tolist") else ([row["tasks"]] if not isinstance(row["tasks"], list) else row["tasks"]),
                    "length": int(row["length"])
                }
                fout.write(json.dumps(record, ensure_ascii=False) + "\n")
        return True, f"输出 {output_jsonl}"
    except Exception as e:
        return False, str(e)

def create_tasks_jsonl(set_dir):
    tasks_path = os.path.join(set_dir, "meta", "tasks.jsonl")
    # task = {"task_index": 0, "task": "Grab the pens and eraser, place them into the white bowl"}
    task = {"task_index": 0, "task": "place the pens and eraser into the white bowl,then use the cloth to clean the table,finally place the cloth next to the white bowl"}
    try:
        with open(tasks_path, "w", encoding="utf-8") as f:
            f.write(json.dumps(task, ensure_ascii=False) + "\n")
        return True, f"输出 {tasks_path}"
    except Exception as e:
        return False, str(e)

def ensure_modality_json(set_dir):
    meta_dir = os.path.join(set_dir, "meta")
    modality_path = os.path.join(meta_dir, "modality.json")
    if os.path.exists(modality_path):
        return True, "modality.json 已存在"
    
    # 根据数据集名称和实际视频文件夹内容自动选择模板
    dataset_name = os.path.basename(set_dir)
    videos_dir = os.path.join(set_dir, "videos")
    
    # 检查实际存在的摄像头类型
    has_front = False
    has_side = False
    has_wrist = False
    
    if os.path.exists(videos_dir):
        for chunk_dir in os.listdir(videos_dir):
            chunk_path = os.path.join(videos_dir, chunk_dir)
            if os.path.isdir(chunk_path):
                video_types = os.listdir(chunk_path)
                has_front = any("front" in vt for vt in video_types)
                has_side = any("side" in vt for vt in video_types)
                has_wrist = any("wrist" in vt for vt in video_types)
                break
    
    # 根据摄像头配置选择模板
    if has_side and has_wrist and not has_front:
        # 使用 side+wrist 配置的自定义模板
        template_content = {
            "state": {
                "single_arm": {"start": 0, "end": 5},
                "gripper": {"start": 5, "end": 6}
            },
            "action": {
                "single_arm": {"start": 0, "end": 5},
                "gripper": {"start": 5, "end": 6}
            },
            "video": {
                "side": {"original_key": "observation.images.side"},
                "wrist": {"original_key": "observation.images.wrist"}
            },
            "annotation": {
                "human.task_description": {"original_key": "task_index"}
            }
        }
        try:
            with open(modality_path, 'w', encoding='utf-8') as f:
                import json
                json.dump(template_content, f, indent=4, ensure_ascii=False)
            return True, f"已创建 side+wrist 配置的 modality.json"
        except Exception as e:
            return False, str(e)
    else:
        # 使用默认的 front+wrist 模板
        src = "examples/SO-100/so100_dualcam__modality.json"
        if not os.path.exists(src):
            return False, f"未找到源文件 {src}"
        try:
            shutil.copyfile(src, modality_path)
            return True, f"已复制 front+wrist 模板到 {modality_path}"
        except Exception as e:
            return False, str(e)

def load_dataset(set_dir):
    root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    # script = os.path.join(root, "scripts", "load_dataset.py")
    script = "scripts/load_dataset.py"
    if not os.path.exists(script):
        print("未找到 scripts/load_dataset.py")
        return
    cmd = [
        sys.executable, script,
        "--dataset-path", set_dir,
        "--plot-state-action",
        "--video-backend", "torchvision_av"
    ]
    print("正在加载数据集 ...")
    subprocess.run(cmd, check=True)
    print("数据集加载完成。")

def print_step(desc, idx, success, detail=""):
    prefix = f"{idx}. {desc}"
    if success:
        message = f"{prefix} 成功"
        if detail:
            message = f"{message} - {detail}"
        print(f"\033[32m{message}\033[0m")
    else:
        message = f"{prefix} 失败"
        if detail:
            message = f"{message} - {detail}"
        print(f"\033[31m{message}\033[0m")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--set-dir", type=str, help="数据集目录（如 ./demo_data/set-2025-10-3-1）")
    args = parser.parse_args()
    set_dir = get_set_dir(args)
    steps = [
        (check_dataset_structure, "数据集结构检查"),
        (ensure_modality_json, "准备 modality.json"),
        (reorganize_videos, "整理视频文件夹结构"),
        (fix_info_json, "修正 info.json 路径字段"),
        (convert_episodes_to_jsonl, "parquet 转 episodes.jsonl"),
        (create_tasks_jsonl, "创建 tasks.jsonl"),
    ]
    results = []

    # print("\033[34m 请使用 gr00t-server 环境运行此脚本！！！\033[0m")

    # for idx, (func, desc) in enumerate(steps, 1):
    #     ok, detail = func(set_dir)
    #     print_step(desc, idx, ok, detail)
    #     results.append(ok)
    # if all(results):
    #     print("\033[34m7. 加载数据集 ...\033[0m")
    #     load_dataset(set_dir)
    #     print(f"\033[92mlerobot数据集\033[94m{set_dir}\033[92m格式对齐GR00T完成，可以炼丹了！\033[0m")
    #     print("\033[34m 请使用 gr00t-server 环境进行炼丹！！！\033[0m")
    # else:
    #     print("\033[31m前置步骤有失败，未执行 load_dataset。\033[0m")

def list_available_datasets():
    """列出可用的数据集"""
    demo_data_dir = "./demo_data"
    if not os.path.exists(demo_data_dir):
        return []
    
    datasets = []
    for item in os.listdir(demo_data_dir):
        item_path = os.path.join(demo_data_dir, item)
        if os.path.isdir(item_path):
            # 检查是否包含必要的数据集文件
            required_dirs = ["meta", "videos", "data"]
            if all(os.path.exists(os.path.join(item_path, req)) for req in required_dirs):
                datasets.append(item)
    return sorted(datasets)

def select_dataset_interactively():
    """交互式选择数据集"""
    datasets = list_available_datasets()
    
    if not datasets:
        print("\033[33m未找到可用的数据集目录\033[0m")
        print("请确保数据集位于 ./demo_data/ 目录下，且包含 meta/, videos/, data/ 子目录")
        return None
    
    print("\033[36m可用的数据集：\033[0m")
    for i, dataset in enumerate(datasets, 1):
        print(f"  {i}. {dataset}")
    
    while True:
        try:
            choice = input("\n请选择数据集 (输入编号或名称): ").strip()
            
            if choice.isdigit():
                # 按编号选择
                idx = int(choice) - 1
                if 0 <= idx < len(datasets):
                    return datasets[idx]
                else:
                    print(f"\033[31m请输入 1-{len(datasets)} 之间的数字\033[0m")
            else:
                # 按名称选择
                if choice in datasets:
                    return choice
                else:
                    print(f"\033[31m数据集 '{choice}' 不存在，请重新选择\033[0m")
                    
        except (KeyboardInterrupt, EOFError):
            print("\n\n操作已取消")
            return None
        except Exception as e:
            print(f"\033[31m输入错误: {e}\033[0m")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--set-dir", type=str, help="数据集目录（如 ./demo_data/set-2025-10-3-1）")
    args = parser.parse_args()
    
    # 如果未通过参数指定数据集，则交互式选择
    if not args.set_dir:
        print("\033[34m请使用 gr00t-server 环境运行此脚本！\033[0m")
        set_dir_name = select_dataset_interactively()
        if not set_dir_name:
            return
        args.set_dir = os.path.join("./demo_data", set_dir_name)
    
    set_dir = get_set_dir(args)
    
    # 显示选中的数据集信息
    print(f"\033[32m选中的数据集: {os.path.basename(set_dir)}\033[0m")
    print(f"\033[36m完整路径: {set_dir}\033[0m")
    print("-" * 50)
    
    steps = [
        (check_dataset_structure, "数据集结构检查"),
        (ensure_modality_json, "准备 modality.json"),
        (reorganize_videos, "整理视频文件夹结构"),
        (fix_info_json, "修正 info.json 路径字段"),
        (convert_episodes_to_jsonl, "parquet 转 episodes.jsonl"),
        (create_tasks_jsonl, "创建 tasks.jsonl"),
    ]
    results = []

    for idx, (func, desc) in enumerate(steps, 1):
        ok, detail = func(set_dir)
        print_step(desc, idx, ok, detail)
        results.append(ok)
        
    if all(results):
        print(f"\n\033[92m✓ 数据集格式转换完成: {set_dir}\033[0m")
        print("\033[34m可以开始训练: python scripts/gr00t_finetune.py ...\033[0m")
    else:
        print("\n\033[31m✗ 格式转换失败，请检查错误信息\033[0m")

if __name__ == "__main__":
    main()