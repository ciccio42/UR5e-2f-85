import copy
import pickle
from pathlib import Path

import cv2
import numpy as np


def _compress_obs(
    obs,
    compress_camera_images=False,
    image_jpeg_quality=95,
):
    """
    Compress observation images before storing them inside the trajectory.

    Backward compatibility:
      - the legacy exact key ``image`` is compressed exactly as before;
      - the legacy exact key ``depth`` keeps the previous behavior.

    Human-dataset mode:
      - when ``compress_camera_images`` is True, every ndarray whose key ends
        with ``_image`` (e.g. camera_front_image) is JPEG-compressed.

    The compressed representation is transparent to users because
    ``Trajectory.get()`` calls ``_decompress_obs`` before returning a step.
    """

    # ------------------------------------------------------------------
    # Legacy generic RGB key
    # ------------------------------------------------------------------
    if 'image' in obs:
        okay, im_string = cv2.imencode(
            '.jpg',
            obs['image'],
            [cv2.IMWRITE_JPEG_QUALITY, int(image_jpeg_quality)],
        )
        assert okay, 'image encoding failed!'
        obs['image'] = im_string

    # ------------------------------------------------------------------
    # Human / named camera RGB keys
    # ------------------------------------------------------------------
    if compress_camera_images:
        for key in list(obs.keys()):
            if key == 'image' or not key.endswith('_image'):
                continue

            value = obs[key]

            if not isinstance(value, np.ndarray):
                continue

            # A raw OpenCV image is expected to be HxW or HxWxC. Avoid
            # accidentally recompressing an already encoded 1-D buffer.
            if value.ndim not in (2, 3):
                continue

            okay, im_string = cv2.imencode(
                '.jpg',
                value,
                [cv2.IMWRITE_JPEG_QUALITY, int(image_jpeg_quality)],
            )

            if not okay:
                raise RuntimeError(
                    f'image encoding failed for observation key {key!r}'
                )

            obs[key] = im_string

    # ------------------------------------------------------------------
    # Legacy generic depth key
    # ------------------------------------------------------------------
    if 'depth' in obs:
        assert (
            len(obs['depth'].shape) == 2
            and obs['depth'].dtype == np.uint8
        ), 'assumes uint8 greyscale depth image!'

        depth_im = np.tile(
            obs['depth'][:, :, None],
            (1, 1, 3),
        )

        okay, depth_string = cv2.imencode(
            '.jpg',
            depth_im,
        )

        assert okay, 'depth encoding failed!'
        obs['depth'] = depth_string

    return obs


def _decompress_obs(obs):
    """
    Restore compressed images to numpy arrays.

    This function is deliberately tolerant so that both old trajectories and
    the new human trajectories can be loaded with the same Trajectory class.
    """

    # ------------------------------------------------------------------
    # Legacy generic RGB key
    # ------------------------------------------------------------------
    if 'image' in obs:
        value = obs['image']
        if isinstance(value, np.ndarray) and value.ndim == 1:
            obs['image'] = cv2.imdecode(
                value,
                cv2.IMREAD_COLOR,
            )

    # ------------------------------------------------------------------
    # Named camera RGB keys
    # ------------------------------------------------------------------
    for key in list(obs.keys()):
        if not key.endswith('_image'):
            continue

        value = obs[key]

        if (
            isinstance(value, np.ndarray)
            and value.ndim == 1
            and value.dtype == np.uint8
        ):
            decoded = cv2.imdecode(
                value,
                cv2.IMREAD_COLOR,
            )

            if decoded is None:
                raise RuntimeError(
                    f'could not decode observation image {key!r}'
                )

            obs[key] = decoded

    # ------------------------------------------------------------------
    # Legacy generic depth key
    # ------------------------------------------------------------------
    if 'depth' in obs:
        value = obs['depth']
        if isinstance(value, np.ndarray) and value.ndim == 1:
            obs['depth'] = (
                cv2.imdecode(
                    value,
                    cv2.IMREAD_GRAYSCALE,
                ).astype(np.float32)
                / 255
            )

    return obs


class Trajectory:
    def __init__(
        self,
        config_str=None,
        compress_camera_images=False,
        image_jpeg_quality=95,
    ):
        self._data = []
        self._raw_state = []
        self._compress_camera_images = bool(
            compress_camera_images
        )
        self._image_jpeg_quality = int(
            image_jpeg_quality
        )
        self.set_config_str(config_str)

    def append(
        self,
        obs,
        reward=None,
        done=None,
        info=None,
        action=None,
        raw_state=None,
    ):
        """
        Logs observation and rewards taken by environment as well as action
        taken.
        """

        obs, reward, done, info, action, raw_state = [
            copy.deepcopy(x)
            for x in [
                obs,
                reward,
                done,
                info,
                action,
                raw_state,
            ]
        ]

        obs = _compress_obs(
            obs,
            compress_camera_images=(
                self._compress_camera_images
            ),
            image_jpeg_quality=(
                self._image_jpeg_quality
            ),
        )

        self._data.append(
            (
                obs,
                reward,
                done,
                info,
                action,
            )
        )

        self._raw_state.append(
            raw_state
        )

    @property
    def T(self):
        """
        Returns number of states.
        """
        return len(self._data)

    def __getitem__(self, t):
        return self.get(t)

    def get(
        self,
        t,
        decompress=True,
    ):
        assert (
            0 <= t < self.T
            or -self.T < t <= 0
        ), 'index should be in (-T, T)'

        (
            obs_t,
            reward_t,
            done_t,
            info_t,
            action_t,
        ) = copy.deepcopy(
            self._data[t]
        )

        if decompress:
            obs_t = _decompress_obs(
                obs_t
            )

        ret_dict = dict(
            obs=obs_t,
            reward=reward_t,
            done=done_t,
            info=info_t,
            action=action_t,
        )

        for key in list(
            ret_dict.keys()
        ):
            if ret_dict[key] is None:
                ret_dict.pop(key)

        return ret_dict

    def change_obs(
        self,
        t,
        obs,
    ):
        (
            _,
            reward_t,
            done_t,
            info_t,
            action_t,
        ) = self._data[t]

        self._data[t] = (
            obs,
            reward_t,
            done_t,
            info_t,
            action_t,
        )

    def mark_last_done(self):
        """
        Mark the final trajectory step as terminal without altering any other
        field. This is used by human demonstrations to mirror the reference
        dataset, where only the last step has ``done=True``.
        """

        if not self._data:
            raise RuntimeError(
                'Cannot mark last step done: trajectory is empty.'
            )

        (
            obs_t,
            reward_t,
            _,
            info_t,
            action_t,
        ) = self._data[-1]

        self._data[-1] = (
            obs_t,
            reward_t,
            True,
            info_t,
            action_t,
        )

    def __len__(self):
        return self.T

    def __iter__(self):
        for d in range(self.T):
            yield self.get(d)

    def get_raw_state(self, t):
        assert (
            0 <= t < self.T
            or -self.T < t <= 0
        ), 'index should be in (-T, T)'

        return copy.deepcopy(
            self._raw_state[t]
        )

    def set_config_str(
        self,
        config_str,
    ):
        self._config_str = config_str

    def save(
        self,
        path,
        **metadata,
    ):
        path = Path(path).expanduser()

        path.parent.mkdir(
            parents=True,
            exist_ok=True,
        )

        payload = {
            'traj': self,
        }

        if self._config_str is not None:
            payload['config_str'] = (
                self._config_str
            )

        payload.update(
            metadata
        )

        with path.open('wb') as file_handle:
            pickle.dump(
                payload,
                file_handle,
            )

        return path

    @property
    def config_str(self):
        return self._config_str
