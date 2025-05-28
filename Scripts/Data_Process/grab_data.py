from pypylon import pylon
import os
import cv2
import time

#! 图像保存路径
base_img_save_path = "/home/zyh/Desktop/ORB-SLAM3_byZyh/Datasets/Mydataset/5_23_desktop/"
euroc_mav_path = os.path.join(base_img_save_path, "mav0")
euroc_cam0_path = os.path.join(euroc_mav_path, "cam0")
euroc_data_path = os.path.join(euroc_cam0_path, "data") # EuRoC风格数据路径

if not os.path.exists(base_img_save_path):
    os.makedirs(base_img_save_path)
    print(f"Created base directory: {base_img_save_path}")
if not os.path.exists(euroc_mav_path):
    os.makedirs(euroc_mav_path)
    print(f"Created mav0 directory: {euroc_mav_path}")
if not os.path.exists(euroc_cam0_path):
    os.makedirs(euroc_cam0_path)
    print(f"Created cam0 directory: {euroc_cam0_path}")
if not os.path.exists(euroc_data_path):
    os.makedirs(euroc_data_path)
    print(f"Created directory for EuRoC style images: {euroc_data_path}")

#! --- 设置外部的时间戳文件的保存路径 ---
timestamps_save_dir = "/home/zyh/Desktop/ORB-SLAM3_byZyh/Examples/Monocular/Basler_TimeStamps/"
#! 提取数据集名称 Extract the dataset name from base_img_save_path (e.g., "5_22_desktop")
dataset_name_for_timestamp_file = os.path.basename(os.path.normpath(base_img_save_path))
external_timestamp_file_path = os.path.join(timestamps_save_dir, f"{dataset_name_for_timestamp_file}.txt")
os.makedirs(timestamps_save_dir, exist_ok=True) # Ensure the directory exists
print(f"External timestamp log will be saved to: {external_timestamp_file_path}")


#! 要采集的图像数量
num_images_to_grab = 1200

# ? --- 生成EuRoC数据集的data.csv文件 ---
def generate_euroc_data_csv(images_directory_path, output_csv_path):
    """
    Generates a data.csv file in EuRoC format.
    Reads .png images from 'images_directory_path' (e.g., .../mav0/cam0/data/),
    extracts nanosecond timestamps from filenames, and writes them to 'output_csv_path'
    (e.g., .../mav0/cam0/data.csv).
    """
    print(f"\n--- Generating EuRoC data.csv ---")
    print(f"Reading images from: {images_directory_path}")
    print(f"Output CSV will be: {output_csv_path}")

    if not os.path.isdir(images_directory_path):
        print(f"Error: Images directory not found: {images_directory_path}")
        return

    image_files = []
    for filename in os.listdir(images_directory_path):
        if filename.lower().endswith(".png"):
            try:
                # Filename without extension is the nanosecond timestamp
                timestamp_ns = int(os.path.splitext(filename)[0])
                image_files.append({"timestamp_ns": timestamp_ns, "filename": filename})
            except ValueError:
                print(f"Skipping file with non-integer name: {filename}")
                continue
    
    if not image_files:
        print("No .png image files found to process in the directory.")
        return

    # 
    image_files.sort(key=lambda x: x["timestamp_ns"])

    try:
        with open(output_csv_path, "w") as f:
            # f.write("#timestamp [ns],filename\n") # 表头已被移除
            for img_data in image_files:
                f.write(f"{img_data['timestamp_ns']},{img_data['filename']}\n")
        print(f"Successfully generated 'data.csv' with {len(image_files)} entries at: {output_csv_path}")
    except OSError as e:
        print(f"Error writing 'data.csv' file at {output_csv_path}: {e}")

# --- End of new function ---

try:
    # 获取tl factory的实例
    tl_factory = pylon.TlFactory.GetInstance()

    # 获取所有可用的设备
    devices = tl_factory.EnumerateDevices()
    if len(devices) == 0:
        raise pylon.RuntimeException("No camera present.")

    # 创建一个InstantCamera实例，使用找到的第一个设备
    camera = pylon.InstantCamera(tl_factory.CreateFirstDevice())
    # 延时4秒
    time.sleep(4)
    camera.Open()
    print(f"Using device {camera.GetDeviceInfo().GetModelName()}")

    # 设置采集模式为连续采集 (Continuous)
    camera.AcquisitionMode.SetValue("Continuous")

    # --- 尝试设置帧率 --- 
    desired_fps = 30.0
    print(f"\n--- Attempting to set Frame Rate to {desired_fps} FPS ---")
    try:
        # 确保相机不在触发模式，这可能影响帧率设置
        if camera.IsFeatureAvailable("TriggerSelector") and camera.GetFeature("TriggerSelector").IsWritable():
            camera.GetFeature("TriggerSelector").SetValue("FrameStart") # 或者 AcquisitionStart
            if camera.IsFeatureAvailable("TriggerMode") and camera.GetFeature("TriggerMode").IsWritable():
                camera.GetFeature("TriggerMode").SetValue("Off")
                print("Set TriggerMode to Off for FrameStart.")

        if camera.IsFeatureAvailable("AcquisitionFrameRate") and camera.GetFeature("AcquisitionFrameRate").IsWritable():
            # 有些相机可能需要先启用帧率控制
            if camera.IsFeatureAvailable("AcquisitionFrameRateEnable") and camera.GetFeature("AcquisitionFrameRateEnable").IsWritable():
                 camera.GetFeature("AcquisitionFrameRateEnable").SetValue(True)
                 print("Set AcquisitionFrameRateEnable to True.")
            
            min_fps, max_fps = camera.GetFeature("AcquisitionFrameRate").GetRange()
            print(f"Camera supported FPS range: {min_fps} - {max_fps}")
            if desired_fps < min_fps:
                print(f"Desired FPS {desired_fps} is below minimum {min_fps}. Setting to minimum.")
                actual_fps_to_set = min_fps
            elif desired_fps > max_fps:
                print(f"Desired FPS {desired_fps} is above maximum {max_fps}. Setting to maximum.")
                actual_fps_to_set = max_fps
            else:
                actual_fps_to_set = desired_fps

            camera.GetFeature("AcquisitionFrameRate").SetValue(actual_fps_to_set)
            print(f"Successfully set AcquisitionFrameRate to {camera.GetFeature('AcquisitionFrameRate').GetValue()} FPS.")
        else:
            print(f"Camera does not support settable AcquisitionFrameRate or it's not writable now.")

    except pylon.GenericException as e:
        print(f"Pylon exception while trying to set frame rate: {e}")
        print("Continuing with camera's default/current frame rate.")
    except Exception as e:
        print(f"General exception while trying to set frame rate: {e}")
        print("Continuing with camera's default/current frame rate.")
    print("--- End of Frame Rate Setting Attempt ---\n")

    # 开始采集图像，使用" 最新图像优先"策略
    # num_images_to_grab 在文件顶部定义
    camera.StartGrabbingMax(num_images_to_grab, pylon.GrabStrategy_LatestImageOnly, pylon.GrabLoop_ProvidedByUser)
    
    print(f"\n--- Starting image acquisition for {num_images_to_grab} images ---")

    # 创建一个图像格式转换器，转换为BGR，OpenCV使用此格式
    converter = pylon.ImageFormatConverter()
    converter.OutputPixelFormat = pylon.PixelType_BGR8packed
    converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned

    images_saved_count = 0
    # 采集图像
    while camera.IsGrabbing():
        try:
            # 等待并抓取一张图像，超时时间为5000毫秒
            grabResult = camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)

            if grabResult.GrabSucceeded():
                images_saved_count += 1
                # Access the image data
                image = converter.Convert(grabResult)
                img_bgr = image.GetArray()

                # --- MODIFIED IMAGE SAVING (EuRoC format) ---
                timestamp_ns = time.time_ns() # 为EuRoC格式获取纳秒时间戳

                # --- ADDED: Save timestamp to external file (MH01.txt format) ---
                try:
                    with open(external_timestamp_file_path, 'a') as ts_file:
                        ts_file.write(f"{timestamp_ns}\n")
                except OSError as e:
                    print(f"Error writing to external timestamp file {external_timestamp_file_path}: {e}")
                # --- END OF ADDED: Save timestamp to external file ---

                # 使用 euroc_data_path 进行保存
                new_img_filename = os.path.join(euroc_data_path, f"{timestamp_ns}.png")
                cv2.imwrite(new_img_filename, img_bgr)
                # --- END OF MODIFIED IMAGE SAVING ---

                # 显示状态，每10张图像更新一次
                if images_saved_count % 10 == 0:
                    print(f"Saved image {images_saved_count}/{num_images_to_grab}: {new_img_filename}")
                
                # 可选: 显示实时图像
                cv2.imshow('Camera Feed', img_bgr)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'): # 按 'q' 停止采集
                    print("Acquisition stopped by user.")
                    break 
                # elif key == ord('p'): # 按 'p' 暂停/继续采集
                #     print("Acquisition paused. Press 'p' again to resume.")
                #     while True:
                #         if cv2.waitKey(0) & 0xFF == ord('p'):
                #             print("Resuming acquisition.")
                #             break
            else:
                print(f"Error: {grabResult.GetErrorCode()} {grabResult.GetErrorDescription()}")

            grabResult.Release() # 释放抓取结果

            if images_saved_count >= num_images_to_grab:
                print(f"\nSuccessfully acquired {images_saved_count} images.")
                break

        except pylon.GenericException as e:
            # Log Pylon-specific exceptions
            print(f"A Pylon exception occurred: {e.GetDescription()}")
            if camera.IsGrabbing(): # Check if still grabbing before trying to stop
                 camera.StopGrabbing()
            break # Exit the loop on Pylon error
        except Exception as e:
            # Log other exceptions
            print(f"An unexpected error occurred: {e}")
            if camera.IsGrabbing(): # Check if still grabbing
                 camera.StopGrabbing()
            break # Exit the loop

    print("--- Image acquisition finished ---")

    # --- Manual confirmation step ---
    if images_saved_count > 0:
        print(f"\n=== 图片采集完成！共采集了 {images_saved_count} 张图片 ===")
        print(f"图片保存位置: {euroc_data_path}")
        print(f"时间戳文件: {external_timestamp_file_path}")
        print("\n请检查采集的图片质量，如有需要可以手动修正。")
        print("完成检查后，请输入 'yes' 开始生成EuRoC数据集格式:")
        
        while True:
            user_input = input(">>> ").strip().lower()
            if user_input == 'yes':
                print("开始处理EuRoC数据集格式...")
                break
            elif user_input in ['no', 'n', 'exit', 'quit']:
                print("用户取消了数据处理。脚本结束。")
                print(f"图片已保存在: {euroc_data_path}")
                print("您可以稍后使用专门的EuRoC处理脚本来生成data.csv文件。")
                exit(0)
            else:
                print("请输入 'yes' 继续处理，或 'no' 退出。")
        
        # data.csv should be in .../mav0/cam0/
        csv_output_location = os.path.join(euroc_cam0_path, "data.csv")
        generate_euroc_data_csv(images_directory_path=euroc_data_path, output_csv_path=csv_output_location)
    else:
        print("\nNo images were saved, so EuRoC data.csv was not generated.")

except pylon.GenericException as e:
    print(f"An exception occurred: {e}")
    if hasattr(e, 'GetDescription'):
        print(e.GetDescription())
    if hasattr(e, 'GetSourceFileName'):
        print(f"Source file: {e.GetSourceFileName()}")
    if hasattr(e, 'GetSourceLine'):
        print(f"Source line: {e.GetSourceLine()}")

finally:
    # 不论发生什么，都关闭相机并清理资源
    if 'camera' in locals() and camera.IsOpen():
        if camera.IsGrabbing():
            camera.StopGrabbing()
        camera.Close()
        print("Camera closed.")
    cv2.destroyAllWindows() # Ensure any OpenCV windows are closed here

    # --- REMOVED TUM PROCESSING PROMPT SECTION ---
    # The user prompt for TUM processing has been removed as per new requirements.
    # Automatic EuRoC CSV generation is handled above.
    # --- END OF REMOVED TUM PROCESSING PROMPT SECTION ---

    print("\nScript finished.")