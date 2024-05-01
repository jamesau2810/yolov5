import sys
# Standard Library Dependency
import os
# Other package
if 'PIL' not in sys.modules:
    from PIL import Image, ImageDraw
if 'cv2' not in sys.modules:
    import cv2
if 'numpy' not in sys.modules:
    import numpy as np

# Get the current working directory
current_path = os.getcwd()
print(current_path)

# Folder containing images
img_dir = "images"

def compute_average_dimensions(folder):
    """Compute average width and height of all images in the specified folder."""
    total_width = 0
    total_height = 0
    img_count = 0

    # Iterate over each file in the directory
    for img_file in os.listdir(folder):
        if img_file.endswith((".jpg", ".jpeg", ".png")):
            image = Image.open(os.path.join(folder, img_file))
            w, h = image.size
            total_width += w
            total_height += h
            img_count += 1

    avg_width = int(total_width / img_count)
    avg_height = int(total_height / img_count)
    return avg_width, avg_height

def resize_images_to_average(folder, avg_width, avg_height):
    """Resize all images in the specified folder to the provided average dimensions."""
    for img_file in os.listdir(folder):
        if img_file.endswith((".jpg", ".jpeg", ".png")):
            image_path = os.path.join(folder, img_file)
            img = Image.open(image_path)
            resized_img = img.resize((avg_width, avg_height), Image.ANTIALIAS)
            resized_img.save(image_path, 'JPEG', quality=95)

def create_video_from_images(folder):
    """Generate a video from all images in the specified folder."""
    video_filename = 'created_video.mp4'
    valid_images = [i for i in os.listdir(folder) if i.endswith((".jpg", ".jpeg", ".png"))]

    first_image = cv2.imread(os.path.join(folder, valid_images[0]))
    h, w, _ = first_image.shape

    codec = cv2.VideoWriter_fourcc(*'mp4v')
    vid_writer = cv2.VideoWriter(video_filename, codec, 30, (w, h))

    for img in valid_images:
        loaded_img = cv2.imread(os.path.join(folder, img))
        for _ in range(20):
            vid_writer.write(loaded_img)

    vid_writer.release()
def resize_images_if_non_uniform(valid_images,h1,w1):
    arr = []
    for img in valid_images:
        h, w, _ = img.shape
        if h == h1 and w == w1:
            arr.append(img)
        else:
            resized_img = img.resize((w1, h1),Image.ANTIALIAS)
            arr.append(resized_img)
    return arr

def create_video_from_images_2(valid_images,w, h):
    video_filename = 'created_video.mp4'
    codec = cv2.VideoWriter_fourcc(*'mp4v')
    vid_writer = cv2.VideoWriter(video_filename, codec, 30, (w, h))

    # for img in valid_images:
        # loaded_img = cv2.imread(os.path.join(folder, img))
    for loaded_img in valid_images:
        for _ in range(20):
            vid_writer.write(loaded_img)
    vid_writer.release()
def display_video_from_images(folder):
    """Display the video from all images in the specified folder."""
    video_filename = 'created_video.mp4'
    valid_images = [i for i in os.listdir(folder) if i.endswith((".jpg", ".jpeg", ".png"))]

    first_image = cv2.imread(os.path.join(folder, valid_images[0]))
    h, w, _ = first_image.shape

    vid_reader = cv2.VideoCapture(video_filename)

    while True:
        ret, frame = vid_reader.read()
        if not ret:
            # Restart the video when it reaches the end
            vid_reader.set(cv2.CAP_PROP_POS_FRAMES, 0)
            continue
        cv2.imshow("Video", frame)
        if cv2.waitKey(30) & 0xFF == ord('q'):  # Exit when 'q' key is pressed
            break

    vid_reader.release()
    cv2.destroyAllWindows()
# if __name__ == '__main__':
#     # Calculate average dimensions of images
#     avg_width, avg_height = compute_average_dimensions(img_dir)
#     # Resize images to average dimensions
#     resize_images_to_average(img_dir, avg_width, avg_height)
#     # Create video from resized images
#     create_video_from_images(img_dir)
#     # Display the video as output
#     display_video_from_images(img_dir)
def draw_bounding_box(img_A,boxes):
    vsample = Image.fromarray(img_A)
    draw = ImageDraw.Draw(vsample)
    for box in boxes:
        draw.rectangle(list(box),fill = None,outline = "red")
    final_img = cv2.cvtColor(np.asarray(vsample),cv2.COLOR_RGB2BGR)
    return final_img

