"""
Handy conversions for CARLA images.

The functions here are provided for real-time display, if you want to save the
converted images, save the images from Python without conversion and convert
them afterwards with the C++ implementation at "Util/ImageConverter" as it
provides considerably better performance.

Ref: https://github.com/carla-simulator/carla/blob/69f7e8ea68046340b1174e8c867381721852b8e1/PythonClient/carla/image_converter.py
"""

import numpy as np


def to_bgra_array(image):
    """Convert a CARLA raw image to a BGRA numpy array."""
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image.height, image.width, 4))
    return array


def to_rgb_array(image):
    """Convert a CARLA raw image to a RGB numpy array."""
    array = to_bgra_array(image)
    array = array[:, :, :3]
    array = array[:, :, ::-1]
    return array

def depth_to_array(image):
    """
    Convert an image containing CARLA encoded depth-map to a 2D array containing
    the depth value of each pixel normalized between [0.0, 1.0].
    """
    array = to_bgra_array(image)
    array = array.astype(np.float32)
    # Apply (R + G * 256 + B * 256 * 256) / (256 * 256 * 256 - 1).
    grayscale = np.dot(array[:, :, :3], [256.0 * 256.0, 256.0, 1.0])
    grayscale /= (256.0 * 256.0 * 256.0 - 1.0)
    return grayscale


def depth_to_logarithmic_grayscale(image):
    """
    Convert an image containing CARLA encoded depth-map to a logarithmic
    grayscale image array.
    """
    grayscale = depth_to_array(image)
    # Convert to logarithmic depth.
    logdepth = np.ones(grayscale.shape) + (np.log(grayscale) / 5.70378)
    logdepth = np.clip(logdepth, 0.0, 1.0)
    logdepth *= 255.0
    # Expand to three colors.
    return np.repeat(logdepth[:, :, np.newaxis], 3, axis=2)
