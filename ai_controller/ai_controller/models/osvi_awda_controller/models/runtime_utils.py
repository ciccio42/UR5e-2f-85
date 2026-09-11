"""Inference-only tensor helpers copied from OSVI-AWDA's ``pyutil.py``."""

from einops import rearrange


def to_channel_first(images):
    return rearrange(images, "... r col c -> ... c r col")


def to_channel_last(images):
    return rearrange(images, "... c r col -> ... r col c")
