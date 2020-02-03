""" Library of helper functions for saving experiences
"""
import datetime
import cv2
import os
import logging


def save_image(frame,  ep_num, step, folder_path='output', invert=False):
    """ Record frame to image to be converted to video

    Args:
        frame : numpy array
            A single image frame to be saved
        folderpath: string
            Absolute directory to save the image
        ep_num: int
            Episode number for name
        step: int
            step number for identifying steps (used for creating mp4 files)
        invert: bool
            whether to invert color, applies to OpenGL rendered images.

    Returns:
    None

    """
    #logging.debug('saving image')
    # directory strings common to all images
    currentDT = datetime.datetime.now()
    date_ep_string = (str(currentDT.year) + str(currentDT.month) +
                      str(currentDT.day) + '_Episode_' + str(ep_num))
    step_string = '/step' + str(step) + '.jpg'

    # Convert to bgr for opencv
    if invert:
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

    # frame directory
    frame_dir = folder_path + date_ep_string

    # Check if directory exists for this episode. If not
    # make one.
    if not os.path.exists(frame_dir):
        os.mkdir(frame_dir)
    frame_filepath = frame_dir + step_string

    # Save the image to file
    cv2.imwrite(frame_filepath, frame)


def convert_jpgs_to_vid(folder_path, ep_num):
    """Convert a folder of frames to mp4 and gif via ffmepg
        Args:
            folder_path: str
                absolute path directory to find jpeg files
            ep_num: int
                episode number identifying the series of jpeg files.
        Returns: None

    """
    # Construct date and time string for naming
    # TODO: make this read the files/folderpath
    currentDT = datetime.datetime.now()
    date_ep_string = (str(currentDT.year) + str(currentDT.month) +
                      str(currentDT.day) + '_Episode_' + str(ep_num))

    # Convert frame images to movie
    frame_dir = (folder_path + str(currentDT.year) + str(currentDT.month) +
                 str(currentDT.day) + '_Episode_' + str(ep_num))
    os.chdir(frame_dir)

    # Save mp4
    frame_output = (str(currentDT.year) + str(currentDT.month) +
                    str(currentDT.day) + '_Episode_' + str(ep_num) + '.mp4')
    cmd_string = ("ffmpeg -r 10  -i step%d.jpg -vcodec libx264 -crf 25  -pix_fmt yuv444p "
                  + frame_output)
    os.system(cmd_string)

    # Save gif (converted from mp4)
    frame_gif_output = (str(currentDT.year) + str(currentDT.month) +
                        str(currentDT.day) + '_Episode_' + str(ep_num) + '.gif')
    cmd_string = ("ffmpeg  -t 10 -i " + frame_output
                  + " -filter_complex \"[0:v] fps=12,scale=480:-1,split [a][b];[a] palettegen [p];[b][p] paletteuse\" "
                  + frame_gif_output)
    os.system(cmd_string)
