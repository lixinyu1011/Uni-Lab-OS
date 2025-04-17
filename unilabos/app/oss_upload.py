import argparse
import os
import time
from typing import Dict, Optional, Tuple

import requests

from unilabos.config.config import OSSUploadConfig


def _init_upload(file_path: str, oss_path: str, filename: Optional[str] = None,
                process_key: str = "file-upload", device_id: str = "default",
                expires_hours: int = 1) -> Tuple[bool, Dict]:
    """
    初始化上传过程

    Args:
        file_path: 本地文件路径
        oss_path: OSS目标路径
        filename: 文件名，如果为None则使用file_path的文件名
        process_key: 处理键
        device_id: 设备ID
        expires_hours: 链接过期小时数

    Returns:
        (成功标志, 响应数据)
    """
    if filename is None:
        filename = os.path.basename(file_path)

    # 构造初始化请求
    url = f"{OSSUploadConfig.api_host}{OSSUploadConfig.init_endpoint}"
    headers = {
        "Authorization": OSSUploadConfig.authorization,
        "Content-Type": "application/json"
    }

    payload = {
        "device_id": device_id,
        "process_key": process_key,
        "filename": filename,
        "path": oss_path,
        "expires_hours": expires_hours
    }

    try:
        response = requests.post(url, headers=headers, json=payload)
        if response.status_code == 201:
            result = response.json()
            if result.get("code") == "10000":
                return True, result.get("data", {})

        print(f"初始化上传失败: {response.status_code}, {response.text}")
        return False, {}
    except Exception as e:
        print(f"初始化上传异常: {str(e)}")
        return False, {}


def _put_upload(file_path: str, upload_url: str) -> bool:
    """
    执行PUT上传

    Args:
        file_path: 本地文件路径
        upload_url: 上传URL

    Returns:
        是否成功
    """
    try:
        with open(file_path, "rb") as f:
            response = requests.put(upload_url, data=f)
            if response.status_code == 200:
                return True

        print(f"PUT上传失败: {response.status_code}, {response.text}")
        return False
    except Exception as e:
        print(f"PUT上传异常: {str(e)}")
        return False


def _complete_upload(uuid: str) -> bool:
    """
    完成上传过程

    Args:
        uuid: 上传的UUID

    Returns:
        是否成功
    """
    url = f"{OSSUploadConfig.api_host}{OSSUploadConfig.complete_endpoint}"
    headers = {
        "Authorization": OSSUploadConfig.authorization,
        "Content-Type": "application/json"
    }

    payload = {
        "uuid": uuid
    }

    try:
        response = requests.post(url, headers=headers, json=payload)
        if response.status_code == 200:
            result = response.json()
            if result.get("code") == "10000":
                return True

        print(f"完成上传失败: {response.status_code}, {response.text}")
        return False
    except Exception as e:
        print(f"完成上传异常: {str(e)}")
        return False


def oss_upload(file_path: str, oss_path: str, filename: Optional[str] = None,
              process_key: str = "file-upload", device_id: str = "default") -> bool:
    """
    文件上传主函数，包含重试机制

    Args:
        file_path: 本地文件路径
        oss_path: OSS目标路径
        filename: 文件名，如果为None则使用file_path的文件名
        process_key: 处理键
        device_id: 设备ID

    Returns:
        是否成功上传
    """
    max_retries = OSSUploadConfig.max_retries
    retry_count = 0

    while retry_count < max_retries:
        try:
            # 步骤1：初始化上传
            init_success, init_data = _init_upload(
                file_path=file_path,
                oss_path=oss_path,
                filename=filename,
                process_key=process_key,
                device_id=device_id
            )

            if not init_success:
                print(f"初始化上传失败，重试 {retry_count + 1}/{max_retries}")
                retry_count += 1
                time.sleep(1)  # 等待1秒后重试
                continue

            # 获取UUID和上传URL
            uuid = init_data.get("uuid")
            upload_url = init_data.get("upload_url")

            if not uuid or not upload_url:
                print(f"初始化上传返回数据不完整，重试 {retry_count + 1}/{max_retries}")
                retry_count += 1
                time.sleep(1)
                continue

            # 步骤2：PUT上传文件
            put_success = _put_upload(file_path, upload_url)
            if not put_success:
                print(f"PUT上传失败，重试 {retry_count + 1}/{max_retries}")
                retry_count += 1
                time.sleep(1)
                continue

            # 步骤3：完成上传
            complete_success = _complete_upload(uuid)
            if not complete_success:
                print(f"完成上传失败，重试 {retry_count + 1}/{max_retries}")
                retry_count += 1
                time.sleep(1)
                continue

            # 所有步骤都成功
            print(f"文件 {file_path} 上传成功")
            return True

        except Exception as e:
            print(f"上传过程异常: {str(e)}，重试 {retry_count + 1}/{max_retries}")
            retry_count += 1
            time.sleep(1)

    print(f"文件 {file_path} 上传失败，已达到最大重试次数 {max_retries}")
    return False


if __name__ == "__main__":
    # python -m unilabos.app.oss_upload -f /path/to/your/file.txt
    # 命令行参数解析
    parser = argparse.ArgumentParser(description='文件上传测试工具')
    parser.add_argument('--file', '-f', type=str, required=True, help='要上传的本地文件路径')
    parser.add_argument('--path', '-p', type=str, default='/HPLC1/Any', help='OSS目标路径')
    parser.add_argument('--device', '-d', type=str, default='test-device', help='设备ID')
    parser.add_argument('--process', '-k', type=str, default='HPLC-txt-result', help='处理键')

    args = parser.parse_args()

    # 检查文件是否存在
    if not os.path.exists(args.file):
        print(f"错误：文件 {args.file} 不存在")
        exit(1)

    print("=" * 50)
    print(f"开始上传文件: {args.file}")
    print(f"目标路径: {args.path}")
    print(f"设备ID: {args.device}")
    print(f"处理键: {args.process}")
    print("=" * 50)

    # 执行上传
    success = oss_upload(
        file_path=args.file,
        oss_path=args.path,
        filename=None,  # 使用默认文件名
        process_key=args.process,
        device_id=args.device
    )

    # 输出结果
    if success:
        print("\n√ 文件上传成功！")
        exit(0)
    else:
        print("\n× 文件上传失败！")
        exit(1)

