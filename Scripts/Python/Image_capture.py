from pypylon import pylon
import cv2
import os

# Define save path
save_dir = "/home/fikus/via_point_movement_primitive/pictures"
os.makedirs(save_dir, exist_ok=True)

# Initialize camera
camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())

try:
    camera.Open()

    # Ensure we are not grabbing images while setting parameters
    if camera.IsGrabbing():
        camera.StopGrabbing()

    # List supported pixel formats
    supported_formats = camera.PixelFormat.Symbolics
    camera.PixelFormat.SetValue("BayerBG8")
    print("Supported Pixel Formats:", supported_formats)

    """
    # Set pixel format to BGR8 if supported
    if "BGR8" in supported_formats:
        camera.PixelFormat.SetValue("BGR8")
        print("Pixel format set to BGR8")
    elif "RGB8" in supported_formats:
        camera.PixelFormat.SetValue("RGB8")
        print("Pixel format set to RGB8")
    else:
        print("No color format supported, using default.")
    """ 

    # Start grabbing after setting pixel format
    camera.StartGrabbing()
    grab_result = camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)

    if grab_result.GrabSucceeded():
        # Convert to OpenCV format
        
        img = grab_result.Arary  # This should be in BGR format
        cv2.imshow('image',img)
        # Define image path
        image_path = os.path.join(save_dir, "captured_color_image.png")

        # Save image using OpenCV
        cv2.imwrite(image_path, img)
        print(f"Color image saved at {image_path}")

    grab_result.Release()
    camera.Close()

except Exception as e:
    print(f"Error: {e}")

finally:
    camera.Close()
