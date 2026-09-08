import os
import cv2
import json
import datetime
import numpy as np
import time
from .rerun_visualizer import RerunLogger
from queue import Queue, Empty
from threading import Thread
import logging_mp
logger_mp = logging_mp.getLogger(__name__)

class EpisodeWriter():
    def __init__(self, task_dir, task_goal=None, task_desc = None, task_steps = None, frequency=30, image_size=[640, 480], rerun_log = True,
                 color_format = None, jpeg_quality = None):
        """
        image_size: [width, height]

        color_format: ".jpg" (default) or ".png". Colour frames are written with
            cv2.imwrite, whose JPEG default is quality 95 -- lossy, and applied per frame
            before anything downstream sees the footage. Measured on real 1920x1080
            footage from this rig: q95 = 46.4 dB PSNR at 0.43 MB/frame, q100 = 49.8 dB at
            0.98 MB/frame, PNG = lossless at 1.93 MB/frame (0.26 / 0.59 / 1.16 GB for a
            20 s episode). JPEG artifacts concentrate on high-contrast edges, which is
            exactly where hand and object boundaries live -- i.e. what the downstream
            segmentation and inpainting models key off. Override via the
            XR_COLOR_FORMAT / XR_JPEG_QUALITY environment variables.

        jpeg_quality: 1-100, used only when color_format is ".jpg". Defaults to 100 here
            rather than OpenCV's 95, because these recordings are training data and the
            extra ~0.3 GB per episode is cheaper than re-shooting.
        """
        logger_mp.info("==> EpisodeWriter initializing...\n")
        self.task_dir = task_dir
        self.text = {
            "goal": "Pick up the red cup on the table.",
            "desc": "task description",
            "steps":"step1: do this; step2: do that; ...",
        }
        if task_goal is not None:
            self.text['goal'] = task_goal
        if task_desc is not None:
            self.text['desc'] = task_desc
        if task_steps is not None:
            self.text['steps'] = task_steps

        self.frequency = frequency
        self.image_size = image_size

        self.color_format = (color_format or os.environ.get("XR_COLOR_FORMAT") or ".jpg").lower()
        if not self.color_format.startswith("."):
            self.color_format = "." + self.color_format
        if self.color_format not in (".jpg", ".png"):
            raise ValueError(f"color_format must be '.jpg' or '.png', got {self.color_format!r}")
        self.jpeg_quality = int(jpeg_quality if jpeg_quality is not None
                                else os.environ.get("XR_JPEG_QUALITY", 100))
        logger_mp.info(f"==> colour frames: {self.color_format}"
                       + (f" q{self.jpeg_quality}" if self.color_format == ".jpg" else " (lossless)"))

        self.rerun_log = rerun_log
        if self.rerun_log:
            logger_mp.info("==> RerunLogger initializing...\n")
            self.rerun_logger = RerunLogger(prefix="online/", IdxRangeBoundary = 60, memory_limit = "300MB")
            logger_mp.info("==> RerunLogger initializing ok.\n")
        
        self.item_id = -1
        self.episode_id = -1
        if os.path.exists(self.task_dir):
            episode_dirs = [episode_dir for episode_dir in os.listdir(self.task_dir) if 'episode_' in episode_dir and not episode_dir.endswith('.zip')]
            episode_last = sorted(episode_dirs)[-1] if len(episode_dirs) > 0 else None
            self.episode_id = 0 if episode_last is None else int(episode_last.split('_')[-1])
            logger_mp.info(f"==> task_dir directory already exist, now self.episode_id is:{self.episode_id}\n")
        else:
            os.makedirs(self.task_dir)
            logger_mp.info(f"==> episode directory does not exist, now create one.\n")
        self.data_info()

        self.is_available = True  # Indicates whether the class is available for new operations
        # Initialize the queue and worker thread
        self.item_data_queue = Queue(-1)
        self.stop_worker = False
        self.need_save = False  # Flag to indicate when save_episode is triggered
        self.worker_thread = Thread(target=self.process_queue)
        self.worker_thread.start()

        logger_mp.info("==> EpisodeWriter initialized successfully.\n")
    
    def is_ready(self):
        return self.is_available

    def data_info(self, version='1.0.0', date=None, author=None):
        self.info = {
                "version": "1.0.0" if version is None else version, 
                "date": datetime.date.today().strftime('%Y-%m-%d') if date is None else date,
                "author": "unitree" if author is None else author,
                "image": {"width":self.image_size[0], "height":self.image_size[1], "fps":self.frequency},
                "depth": {"width":self.image_size[0], "height":self.image_size[1], "fps":self.frequency},
                "audio": {"sample_rate": 16000, "channels": 1, "format":"PCM", "bits":16},    # PCM_S16
                "joint_names":{
                    "left_arm":   [],
                    "left_ee":  [],
                    "right_arm":  [],
                    "right_ee": [],
                    "body":       [],
                },

                "tactile_names": {
                    "left_ee": [],
                    "right_ee": [],
                }, 
                "sim_state": ""
            }

 
    def create_episode(self):
        """
        Create a new episode.
        Returns:
            bool: True if the episode is successfully created, False otherwise.
        Note:
            Once successfully created, this function will only be available again after save_episode complete its save task.
        """
        if not self.is_available:
            logger_mp.info("==> The class is currently unavailable for new operations. Please wait until ongoing tasks are completed.")
            return False  # Return False if the class is unavailable

        # Reset episode-related data and create necessary directories
        self.item_id = -1
        self.episode_id = self.episode_id + 1
        
        self.episode_dir = os.path.join(self.task_dir, f"episode_{str(self.episode_id).zfill(4)}")
        self.color_dir = os.path.join(self.episode_dir, 'colors')
        self.depth_dir = os.path.join(self.episode_dir, 'depths')
        self.audio_dir = os.path.join(self.episode_dir, 'audios')
        self.json_path = os.path.join(self.episode_dir, 'data.json')
        os.makedirs(self.episode_dir, exist_ok=True)
        os.makedirs(self.color_dir, exist_ok=True)
        os.makedirs(self.depth_dir, exist_ok=True)
        os.makedirs(self.audio_dir, exist_ok=True)
        with open(self.json_path, "w", encoding="utf-8") as f:
            f.write('{\n')
            f.write('"info": ' + json.dumps(self.info, ensure_ascii=False, indent=4) + ',\n')
            f.write('"text": ' + json.dumps(self.text, ensure_ascii=False, indent=4) + ',\n')
            f.write('"data": [\n')
        self.first_item = True   # Flag to handle commas in JSON array

        if self.rerun_log:
            self.online_logger = RerunLogger(prefix="online/", IdxRangeBoundary = 60, memory_limit="300MB")

        self.is_available = False  # After the episode is created, the class is marked as unavailable until the episode is successfully saved
        logger_mp.info(f"==> New episode created: {self.episode_dir}")
        return True  # Return True if the episode is successfully created
        
    def add_item(self, colors, depths=None, states=None, actions=None, tactiles=None, audios=None, sim_state=None,
                 head_pose=None, timestamp=None, wrist_pose=None):
        """
        Queue one frame for writing.

        head_pose: (4,4) SE(3) pose of the operator's head, i.e. televuer's
            `TeleData.head_pose`. REQUIRED for any rig where the camera is mounted on the
            headset rather than on the robot: the recorded joint angles carry no trace of
            where the operator was looking, so without this the render camera cannot be
            made to pan with the real footage, and it is not recoverable after the fact
            (short of running SLAM on the video). Harmless to omit for a robot-mounted
            camera. Accepts a numpy array or a nested list.

        timestamp: wall-clock seconds for THIS frame, captured at acquisition time by the
            caller -- not here, because items are written by a background queue thread and
            the write time can lag acquisition by an unbounded amount. The nominal
            `frequency` is only a best-effort `time.sleep` target, so real frame intervals
            drift; anything aligning this episode against a separately-recorded stream
            needs the true times.

        wrist_pose: {"left": (4,4), "right": (4,4)} -- the wrist TARGETS handed to
            `solve_ik`, i.e. televuer's `TeleData.{left,right}_wrist_pose` in the waist
            frame. Without these an episode records only `sol_q`, the IK's OUTPUT, so a
            robot whose reach looks smaller than the operator's is undiagnosable: you
            cannot tell whether XR reported small hand motion or whether XR reported large
            motion and the IK failed to follow it. Those need opposite fixes. Episode 0030
            hit exactly that wall -- 2-7 cm of retargeted eef range against roughly 15-20 cm
            of real hand travel, with head motion too small to explain the gap.
            Accepts numpy arrays or nested lists.
        """
        # Increment the item ID
        self.item_id += 1
        # numpy arrays are not JSON-serializable, and this dict is json.dumps'd verbatim.
        if head_pose is not None and hasattr(head_pose, "tolist"):
            head_pose = head_pose.tolist()
        if wrist_pose is not None:
            wrist_pose = {k: (v.tolist() if hasattr(v, "tolist") else v)
                          for k, v in wrist_pose.items() if v is not None}
        # Create the item data dictionary
        item_data = {
            'idx': self.item_id,
            'colors': colors,
            'depths': depths,
            'states': states,
            'actions': actions,
            'tactiles': tactiles,
            'audios': audios,
            'sim_state': sim_state,
            'head_pose': head_pose,
            'timestamp': timestamp,
            'wrist_pose': wrist_pose,
        }
        # Enqueue the item data
        self.item_data_queue.put(item_data)

    def process_queue(self):
        while not self.stop_worker or not self.item_data_queue.empty():
            # Process items in the queue
            try:
                item_data = self.item_data_queue.get(timeout=1)
                try:
                    self._process_item_data(item_data)
                except Exception as e:
                    logger_mp.info(f"Error processing item_data (idx={item_data['idx']}): {e}")
                self.item_data_queue.task_done()
            except Empty:
                pass
        
            # Check if save_episode was triggered
            if self.need_save and self.item_data_queue.empty():
                self._save_episode()

    def _safe_imwrite(self, directory, filename, image, encode_params, kind):
        """
        Write one image, returning its filename, or None if there was nothing to write.

        Never raises: an absent camera frame is a normal condition on a station whose
        camera is not up yet, and it must not take the frame's joint data down with it
        (see the note in _process_item_data). Missing frames are counted and reported once
        rather than once per frame, because at 30 Hz a per-frame log buries everything else.
        """
        if image is None or not hasattr(image, "size") or image.size == 0:
            self._missing_frames = getattr(self, "_missing_frames", 0) + 1
            if self._missing_frames in (1, 100) or self._missing_frames % 1000 == 0:
                logger_mp.warning(
                    f"No {kind} frame to save (count={self._missing_frames}). Joint data is "
                    f"still being recorded; the episode will simply have no images.")
            return None
        try:
            ok = (cv2.imwrite(os.path.join(directory, filename), image, encode_params)
                  if encode_params else cv2.imwrite(os.path.join(directory, filename), image))
        except Exception as e:
            logger_mp.warning(f"Failed to save {kind} image {filename}: {e}")
            return None
        if not ok:
            logger_mp.warning(f"Failed to save {kind} image {filename}.")
            return None
        return filename

    def _process_item_data(self, item_data):
        idx = item_data['idx']
        colors = item_data.get('colors', {})
        depths = item_data.get('depths', {})
        audios = item_data.get('audios', {})

        # Save images
        #
        # A missing or empty frame must NOT cost us the frame's joint data. cv2.imwrite
        # RAISES on an empty array (it does not return False), that exception used to
        # propagate out of this method, and process_queue caught it and moved on -- so the
        # item was never appended to data.json at all. Result: with a camera that produced
        # no frames, entire episodes came back with zero frames despite the arm and hand
        # trajectories having been recorded perfectly. Images are optional here; states and
        # actions are the point.
        if colors:
            encode_params = ([int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
                             if self.color_format == ".jpg" else
                             [int(cv2.IMWRITE_PNG_COMPRESSION), 3])
            for color_key, color in list(colors.items()):
                saved = self._safe_imwrite(self.color_dir,
                                           f'{str(idx).zfill(6)}_{color_key}{self.color_format}',
                                           color, encode_params, "color")
                if saved:
                    item_data['colors'][color_key] = os.path.join('colors', saved)
                else:
                    item_data['colors'].pop(color_key, None)

        # Save depths
        if depths:
            for depth_key, depth in list(depths.items()):
                saved = self._safe_imwrite(self.depth_dir,
                                           f'{str(idx).zfill(6)}_{depth_key}.jpg',
                                           depth, None, "depth")
                if saved:
                    item_data['depths'][depth_key] = os.path.join('depths', saved)
                else:
                    item_data['depths'].pop(depth_key, None)

        # Save audios
        if audios:
            for mic, audio in audios.items():
                audio_name = f'audio_{str(idx).zfill(6)}_{mic}.npy'
                np.save(os.path.join(self.audio_dir, audio_name), audio.astype(np.int16))
                item_data['audios'][mic] = os.path.join('audios', audio_name)

        # Update episode data
        with open(self.json_path, "a", encoding="utf-8") as f:
            if not self.first_item:
                f.write(",\n")
            f.write(json.dumps(item_data, ensure_ascii=False, indent=4))
            self.first_item = False

        # Log data if necessary
        if self.rerun_log:
            curent_record_time = time.time()
            logger_mp.info(f"==> episode_id:{self.episode_id}  item_id:{idx}  current_time:{curent_record_time}")
            self.rerun_logger.log_item_data(item_data)

    def save_episode(self):
        """
        Trigger the save operation. This sets the save flag, and the process_queue thread will handle it.
        """
        self.need_save = True  # Set the save flag
        logger_mp.info(f"==> Episode saved start...")

    def _save_episode(self):
        """
        Save the episode data to a JSON file.
        """
        with open(self.json_path, "a", encoding="utf-8") as f:
            f.write("\n]\n}")      # Close the JSON array and object

        self.need_save = False     # Reset the save flag
        self.is_available = True   # Mark the class as available after saving
        logger_mp.info(f"==> Episode saved successfully to {self.json_path}.")

    def close(self):
        """
        Stop the worker thread and ensure all tasks are completed.
        """
        self.item_data_queue.join()
        if not self.is_available:  # If self.is_available is False, it means there is still data not saved.
            self.save_episode()
        while not self.is_available:
            time.sleep(0.01)
        self.stop_worker = True
        self.worker_thread.join()