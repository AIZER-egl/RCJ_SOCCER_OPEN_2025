import cv2
import os


def frames_to_video(input_dir, output_video, fps=30):
    # Get the list of files in the input directory
    files = sorted(os.listdir(input_dir))

    # Get the first frame to read its dimensions
    first_frame = cv2.imread(os.path.join(input_dir, files[0]))

    if first_frame is None:
        print("No frames found in the input directory")
        return

    height, width, _ = first_frame.shape

    # Define the codec and create VideoWriter object
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(output_video, fourcc, fps, (width, height))

    # sort files by name
    files.sort(key=lambda x: int(x.split('_')[1].split('.')[0]))

    # Write frames to the video
    for i, file in enumerate(files):
        frame = cv2.imread(os.path.join(input_dir, file))
        print(f"frame: {i}/{len(files)}")
        out.write(frame)

    # Release the VideoWriter object
    out.release()

    print(f"Video saved as {output_video}")


# Example usage
input_directory = "/home/aizer/frontcamera"
output_video = "output_video_front.mp4"
frames_to_video(input_directory, output_video)

input_directory = "/home/aizer/omnicamera"
output_video = "output_video_omni.mp4"
frames_to_video(input_directory, output_video)
