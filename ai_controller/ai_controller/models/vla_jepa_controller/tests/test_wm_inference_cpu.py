"""CPU tests of causal image history, first-8-token prediction, concatenation.

Run: python -m unittest discover -s tests -v
No checkpoint, LeRobot, GPU or robot required for these tests.
"""
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

import torch
from torch import nn

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from vla_jepa_wm_conditioning import (  # noqa E402
    build_observed_pair, predict_s1_from_observed_pair, build_wm_conditioning,
)


class FakeVideoEncoder(nn.Module):
    def __init__(self):
        super().__init__()
        self.register_buffer("dummy", torch.zeros(1))

    @property
    def device(self):
        return self.dummy.device

    def get_vision_features(self, pixel_values_videos):
        # One temporal token-grid of 16x16 patches, each 1024D.
        b = pixel_values_videos.shape[0]
        return pixel_values_videos.mean(dim=(1,2,3,4))[:, None, None].expand(b, 256, 1024)


class FakeVideoProcessor:
    def __call__(self, videos, *, do_rescale, return_tensors, device):
        assert do_rescale is False
        assert return_tensors == "pt"
        return {"pixel_values_videos": torch.stack(videos).to(device)}


class FakePredictor(nn.Module):
    def __init__(self):
        super().__init__()
        self.calls = []

    def forward(self, frames, actions):
        self.calls.append((tuple(frames.shape), actions.detach().clone()))
        assert actions.shape[1:] == (8, 2048)
        return frames + actions.mean(dim=(1,2), keepdim=True)


class FakeModel:
    def __init__(self):
        self.video_encoder = FakeVideoEncoder()
        self.video_processor = FakeVideoProcessor()
        self.video_predictor = FakePredictor()
        self.config = SimpleNamespace(num_action_tokens_per_timestep=8)
        self.readouts = []

    @staticmethod
    def _merge_views(embeddings, b, v):
        n, h = embeddings.shape[1:]
        return embeddings.reshape(b,v,n,h).permute(0,2,1,3).reshape(b,n,v*h)

    def _encode_qwen(self, images, instructions, *, need_action_tokens):
        assert need_action_tokens
        self.readouts.append((images, instructions))
        b = len(instructions)
        emb = torch.full((b,32,2048), 0.5, dtype=torch.bfloat16)
        actions = torch.arange(24, dtype=torch.bfloat16).view(1,24,1).expand(b,24,2048)
        return emb, actions


class TestObservedPairs(unittest.TestCase):
    def setUp(self):
        self.front = torch.zeros((3,224,224), dtype=torch.float32)
        self.wrist = torch.full((3,224,224),0.2)
        self.previous_front = torch.full((3,224,224),0.1)
        self.previous_wrist = torch.full((3,224,224),0.3)

    def test_first_observation_is_duplicated(self):
        video = build_observed_pair(None,(self.front,self.wrist))
        self.assertEqual(video.shape,(1,2,2,3,224,224))
        self.assertTrue(torch.equal(video[0,0,0],video[0,0,1]))
        self.assertTrue(torch.equal(video[0,1,0],video[0,1,1]))

    def test_view_order_and_previous_current_order(self):
        video = build_observed_pair((self.previous_front,self.previous_wrist),
                                    (self.front,self.wrist))
        self.assertAlmostEqual(float(video[0,0,0].mean()),0.1,places=6)
        self.assertAlmostEqual(float(video[0,0,1].mean()),0.0,places=6)
        self.assertAlmostEqual(float(video[0,1,0].mean()),0.3,places=6)
        self.assertAlmostEqual(float(video[0,1,1].mean()),0.2,places=6)

    def test_rejects_non_normalized_or_wrong_shape_images(self):
        with self.assertRaises(ValueError):
            build_observed_pair(None,(self.front,self.wrist*6))
        with self.assertRaises(ValueError):
            build_observed_pair(None,(torch.zeros(3,200,224),self.wrist))

    def test_future_qwen_tokens_are_unused(self):
        model = FakeModel()
        video = build_observed_pair(None,(self.front,self.wrist))
        tokens = torch.arange(24,dtype=torch.float32).view(1,24,1).expand(1,24,2048)
        out1 = predict_s1_from_observed_pair(model,video,tokens)
        changed=tokens.clone()
        changed[:,8:]+=100
        out2 = predict_s1_from_observed_pair(model,video,changed)
        self.assertEqual(tuple(out1.shape),(1,256,2048))
        self.assertTrue(torch.equal(out1,out2))
        self.assertEqual(len(model.video_predictor.calls),2)
        self.assertEqual(model.video_predictor.calls[0][0],(1,256,2048))

    def test_one_qwen_pass_and_288_token_concat(self):
        model = FakeModel()
        video = build_observed_pair(None,(self.front,self.wrist))
        cond = build_wm_conditioning(model,[[self.front,self.wrist]],["task"],video)
        self.assertEqual(cond.shape,(1,288,2048))
        self.assertEqual(len(model.readouts),1)
        self.assertEqual(len(model.video_predictor.calls),1)
        self.assertEqual(cond.dtype,torch.bfloat16)
        self.assertTrue(torch.all(cond[:,:32] == 0.5))
        self.assertFalse(cond.requires_grad)


if __name__ == "__main__":
    unittest.main()
