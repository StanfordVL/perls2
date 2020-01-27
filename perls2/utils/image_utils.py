"""
Common image utilities

author: Danfei Xu
"""
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import logging
import numpy as np
import cv2
import numpy as np
import scipy.misc
import scipy.signal
import PIL.Image

class Display(object):
    """
    Wraps image display function from cv2
    """
    def __init__(self, name="display", verbose=False):
        self._verbose = verbose
        self._name = name
        self._points = []
        cv2.namedWindow(name, cv2.CV_WINDOW_AUTOSIZE)

    def _cursor_position_callback(self, event, x, y, flags, params):
        if event == 1:
            self._points.append((x, y))
            if self._verbose:
                logging.info('new point')

    def imshow(self, im, is_bgr=False, waitkey=1):
        """ Display image

        Args:
            im: image array (H, W, C)
            is_bgr: if the image is in BGR
            waitkey: waitkey for the display function
        """
        if is_bgr:
            im = bgr2rgb(im)

        cv2.imshow(self._name, im)

    def close(self):
        """ Close the current window
        """
        cv2.destroyWindow(self._name)

    def get_cursor_points(self, image, n_points, is_bgr=False):
        """ Get points from cursor click event

        Args:
            image: image array to display
            n_points: number of points to collect
            is_bgr: if the image is in bgr

        """
        self._points = []
        cv2.setMouseCallback(self._name, self._cursor_position_callback)
        self.imshow(image, is_bgr, waitkey=None)
        while len(self._points) < n_points:
            try:
                cv2.waitKey(1)
            except KeyboardInterrupt:
                self.close()
        self.close()
        return self._points


def bgr2rgb(im):
    """ Convert a BGR image to RGB

    args:
        im: [H, W, C] numpy array

    returns:
        [H, W, C] numpy array
    """
    return cv2.cvtColor(im, cv2.COLOR_BGR2RGB)


def crop_im(img, upper_left, lower_right):
    """ Crop an image by a rectangle

    args:
        im: [H, W, C] numpy array
        upper_left: crop region upper left
        lower_right: crop region lower right

    returns:
        [H', W', C] numpy array
    """
    return img[upper_left[0]:lower_right[0],
               upper_left[1]:lower_right[1], :]


def normalize_ims(ims, scale=255.):
    """ Normalize images by dividing a constant

    args:
        im: [H, W, C] numpy array

    returns:
        [H, W, C] numpy array (nomralized image)
    """
    ims = ims.astype(np.float32) / scale
    return ims


def transform(data, translation, theta):
    """Create a new image by translating and rotating the current image.

    Args:
        translation: The XY translation vector.
        theta: Rotation angle in radians, with positive meaning
            counter-clockwise.

    Returns:
        An image of the same type that has been rotated and translated.
    """
    translation_map = np.float32([[1, 0, translation[1]],
                                  [0, 1, translation[0]]])
    translation_map_affine = np.r_[translation_map, [[0, 0, 1]]]

    theta = np.rad2deg(theta)
    rotation_map = cv2.getRotationMatrix2D(
            (data.shape[1] / 2, data.shape[0] / 2), theta, 1)
    rotation_map_affine = np.r_[rotation_map, [[0, 0, 1]]]

    full_map = rotation_map_affine.dot(translation_map_affine)
    full_map = full_map[:2, :]

    transformed_data = cv2.warpAffine(
            data, full_map, (data.shape[1], data.shape[0]),
            flags=cv2.INTER_NEAREST)

    return transformed_data.astype(data.dtype)


def crop(data, height, width, c0=None, c1=None):
    """Crop the image centered around c0, c1.

    Args:
        height: The height of the desired image.
        width: The width of the desired image.
        c0: The center height point at which to crop. If not specified, the
            center of the image is used.
        c1: The center width point at which to crop. If not specified, the
            center of the image is used.

    Returns:
        A cropped Image of the same type.
    """
    # compute crop center px
    height = int(np.round(height))
    width = int(np.round(width))

    if c0 is None:
        c0 = float(data.shape[0]) / 2

    if c1 is None:
        c1 = float(data.shape[1]) / 2

    # crop using PIL
    desired_start_row = int(np.floor(c0 - float(height) / 2))
    desired_end_row = int(np.floor(c0 + float(height) / 2))
    desired_start_col = int(np.floor(c1 - float(width) / 2))
    desired_end_col = int(np.floor(c1 + float(width) / 2))

    pil_image = PIL.Image.fromarray(data)
    cropped_pil_image = pil_image.crop(
            (desired_start_col,
             desired_start_row,
             desired_end_col,
             desired_end_row))
    crop_data = np.array(cropped_pil_image)

    if crop_data.shape[0] != height or crop_data.shape[1] != width:
        raise ValueError('Crop dims are incorrect.')

    return crop_data.astype(data.dtype)


def inpaint(data, rescale_factor=0.5):
    """Fills in the zero pixels in the depth image.

    Parameters:
        data: The raw depth image.
        rescale_factor: Amount to rescale the image for inpainting, smaller
            numbers increase speed.

    Returns:
        new_data: The inpainted depth imaga.
    """
    # Form inpaint kernel.
    inpaint_kernel = np.array([[1, 1, 1], [1, 0, 1], [1, 1, 1]])
    height = np.shape(data)[0]
    width = np.shape(data)[1]
    #data = np.reshape(data, )
    # data = np.stack((data,)*3, axis=-1)
    print(np.shape(data))
    # Resize the image.
    resized_data = np.array(PIL.Image.fromarray(data).resize(
           (int(width*rescale_factor), int(height*rescale_factor)), PIL.Image.BILINEAR))


    # Inpaint the smaller image.
    cur_data = resized_data.copy()
    zeros = (cur_data == 0)

    while np.any(zeros):
        neighbors = scipy.signal.convolve2d(
                (cur_data != 0), inpaint_kernel, mode='same', boundary='symm')
        avg_depth = scipy.signal.convolve2d(
                cur_data, inpaint_kernel, mode='same', boundary='symm')
        avg_depth[neighbors > 0] = (avg_depth[neighbors > 0] /
                                    neighbors[neighbors > 0])
        avg_depth[neighbors == 0] = 0
        avg_depth[resized_data > 0] = resized_data[resized_data > 0]
        cur_data = avg_depth

        zeros = (cur_data == 0)

    inpainted_data = cur_data

    # Fill in zero pixels with inpainted and resized image.
    filled_data =  np.array(PIL.Image.fromarray(inpainted_data).resize(
            (width, height), PIL.Image.BILINEAR))

    # scipy.misc.imresize(inpainted_data, 1.0 / rescale_factor,
    #                                  interp='bilinear')

    new_data = np.copy(data)
    new_data[data == 0] = filled_data[data == 0]

    return new_data


def threshold_gradients(data, threshold):
    """Get the threshold gradients.

    Creates a new DepthImage by zeroing out all depths
    where the magnitude of the gradient at that point is
    greater than threshold.

    Args:
        data: The raw depth image.
        threhold: A threshold for the gradient magnitude.

    Returns:
        A new DepthImage created from the thresholding operation.
    """
    data = np.copy(data)
    gx, gy = np.gradient(data.astype(np.float32))
    gradients = np.zeros([gx.shape[0], gx.shape[1], 2])
    gradients[:, :, 0] = gx
    gradients[:, :, 1] = gy
    gradient_magnitudes = np.linalg.norm(gradients, axis=2)
    ind = np.where(gradient_magnitudes > threshold)
    data[ind[0], ind[1]] = 0.0
    return data


def apply_gamma_noise(data, gamma_shape=1000):
    """Apply multiplicative denoising to the images.

    Args:
        data: A numpy array of 3 or 4 dimensions.

    Returns:
        The corrupted data with the applied noise.
    """
    if data.ndim == 3:
        images = data[np.newaxis, :, :, :]
    else:
        images = data

    num_images = images.shape[0]
    gamma_scale = 1.0 / gamma_shape

    mult_samples = scipy.stats.gamma.rvs(gamma_shape, scale=gamma_scale,
                                         size=num_images)
    mult_samples = mult_samples[:, np.newaxis, np.newaxis, np.newaxis]
    new_images = data * mult_samples

    if data.ndim == 3:
        return new_images[0]
    else:
        return new_images


def apply_gaussian_noise(data,
                         prob=0.5,
                         rescale_factor=4.0,
                         sigma=0.005):
    """Add correlated Gaussian noise.

    Args:
        data: A numpy array of 3 or 4 dimensions.

    Returns:
        The corrupted data with the applied noise.
    """
    if data.ndim == 3:
        images = data[np.newaxis, :, :, :]
    else:
        images = data

    num_images = images.shape[0]
    image_height = images.shape[1]
    image_width = images.shape[2]
    sample_height = int(image_height / rescale_factor)
    sample_width = int(image_width / rescale_factor)
    num_pixels = sample_height * sample_width

    new_images = []

    for i in range(num_images):
        image = images[i, :, :, 0]

        if np.random.rand() < prob:
            gp_noise = scipy.stats.norm.rvs(scale=sigma, size=num_pixels)
            gp_noise = gp_noise.reshape(sample_height, sample_width)
            gp_noise = scipy.misc.imresize(gp_noise, rescale_factor,
                                           interp='bicubic', mode='F')
            image[image > 0] += gp_noise[image > 0]

        new_images.append(image[:, :, np.newaxis])

    new_images = np.stack(new_images)

    if data.ndim == 3:
        return new_images[0]
    else:
        return new_images

def remove_large_outliers(depth_crop,percentile=95):
    
    thresh = np.percentile(depth_crop,95)
    invalid = depth_crop > thresh
    indeces = np.transpose(np.nonzero(invalid))
    for index in indeces:
        x,y = index
        add = 1
        try:
            if x > depth_crop.shape[0]/2:
                add = -1
            while invalid[x,y]:
                x+=add
            depth_crop[tuple(index)] = depth_crop[(x,y)]
        except:
            pass
    return depth_crop

def intrinsic_to_projection_matrix(K, height, width, near, far,
                                   upside_down=True):
        """Convert the camera intrinsics to the projection matrix.

        Takes a Hartley-Zisserman intrinsic matrix and returns a Bullet/OpenGL
        style projection matrix. We pad with zeros on right and bottom and a 1
        in the corner.

        Uses algorithm found at:
        https://strawlab.org/2011/11/05/augmented-reality-with-OpenGL#note-about-image-coordinates

        Parameters
        ----------
        K :
            The camera intrinsincs matrix.
        height :
            The image height.
        width :
            The image width.
        near :
            The distance to the near plane.
        far :
            The distance to the far plane.
        upside_down :
            (Default value = True)
        """
        projection_matrix = np.empty((4, 4), dtype=np.float32)

        f_x = K[0, 0]
        f_y = K[1, 1]
        x_0 = K[0, 2]
        y_0 = K[1, 2]
        s = K[0, 1]

        if upside_down:
            x_0 = width - x_0
            y_0 = height - y_0

        projection_matrix[0, 0] = 2 * f_x / width
        projection_matrix[0, 1] = -2 * s / width
        projection_matrix[0, 2] = (width - 2 * x_0) / width
        projection_matrix[0, 3] = 0

        projection_matrix[1, 0] = 0
        projection_matrix[1, 1] = 2 * f_y / height
        projection_matrix[1, 2] = (-height + 2 * y_0) / height
        projection_matrix[1, 3] = 0

        projection_matrix[2, 0] = 0
        projection_matrix[2, 1] = 0
        projection_matrix[2, 2] = (-far - near) / (far - near)
        projection_matrix[2, 3] = -2 * far * near / (far - near)

        projection_matrix[3, 0] = 0
        projection_matrix[3, 1] = 0
        projection_matrix[3, 2] = -1
        projection_matrix[3, 3] = 0

        projection_matrix = list(projection_matrix.transpose().flatten())

        return projection_matrix
